/* 
 * Small Bot Motor Controller Firmware
 * 
 * Matt Ruffner @ KORA, 2020
 *
 * The controller firmware drives two motors to a setpoint in m/s using a separate PI
 * controller for each. Wheel odometry is interpolated from quadrature motor encoder
 * feedback.
 */
#include <FreeRTOS_SAMD51.h>
#include <semphr.h>
#include <PID_v1.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <rover_msgs/SBMCTelemReading.h>
#include <rover_msgs/SBMCServoSetting.h>
#include <DPEng_ICM20948_AK09916.h>
#include <Mahony_DPEng.h>
#include <Madgwick_DPEng.h>

// rtos delay helpers
#include "include/delay_helpers.h"
// sbmc shield control and motor driver
#include "include/sbmcs.h"

#define DEBUG 1


// IMU CONFIG PARAMS
#define SERIAL Serial
#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL   0
#define IMU_ADDR 0x68 // aka AD0 = LOW/0
#define IMU_PERIOD 10 // milliseconds

// ROBOT PARAMS
// for 165 rpm servo city plantetary gear motor with encoder
// counts per revolution
#define ENCODER_CPR 2442.96 // in counts
// wheel circumference
#define WHEEL_CIRC 0.468 // meters
// distance between drive wheels
#define BASE_WIDTH 0.352 // meters
// encoder counts per meter
#define ENCODER_CPM 5220.0 // in counts
#define MIN_SPD_THRESH 0.005 // slowest we are willing to go in m/s

// shield library interface
SBMCS shield;
Servo servo;
MC33932 motors;

// IMU reading and orientation filtering
// Create sensor instance and variables to hold IMU 
// calibration offsets
DPEng_ICM20948 myICM = DPEng_ICM20948(0x948A, 0x948B, 0x948C);
// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { -1.76F, 22.54F, 4.43F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.954,  -0.019,  0.003 },
                                    {  -0.019,  1.059, -0.009 },
                                    {  0.003,  -0.009,  0.990 } };

float mag_field_strength        = 29.85F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

// Mahony is lighter weight as a filter and should be used
// on slower systems
Mahony_DPEng filter;
//Madgwick_DPEng filter;

// motor safety timestamp, cut off motors if we havent received a twist in a while
// needs work on ROS side,  need to send a twist every second to keep this happy
// so just dont use it for now
//volatile unsigned long lastTwistStamp = 0;

// begin PID control interface and tunings
// need mutexes for these 
double m1Setpoint, m1Input, m1Output;
double m2Setpoint, m2Input, m2Output;
volatile double m1TargetVel, m2TargetVel;

//Define the aggressive and conservative Tuning Parameters
double consKp=0.1, consKi=4, consKd=0.0;

//Specify the links and initial tuning parameters
PID m1PID(&m1Input, &m1Output, &m1Setpoint, consKp, consKi, consKd, DIRECT);
PID m2PID(&m2Input, &m2Output, &m2Setpoint, consKp, consKi, consKd, DIRECT);
// end pid

//**************************************************************************
// freertos task handles
TaskHandle_t Handle_telemTask;
TaskHandle_t Handle_speedTask;
TaskHandle_t Handle_imuTask;
TaskHandle_t Handle_monitorTask;
TaskHandle_t Handle_odomTask;
// ros serial publish semaphore (Serial1)
SemaphoreHandle_t rsSem;
// serial debug logging (Serial)
SemaphoreHandle_t dbSem;


//*****************************************************************
// rosserial callback for setting drive velocity
void twistCb( const geometry_msgs::Twist &cmdVel ){
  // joy_twist_remap was need to send linear.x and angular.z
  
  float linear_x = cmdVel.linear.x;

  float vr = linear_x + cmdVel.angular.z * BASE_WIDTH / 2.0;
  float vl = linear_x - cmdVel.angular.z * BASE_WIDTH / 2.0;

  m1TargetVel = vr; //cmdVel.linear.y;
  m2TargetVel = vl; //cmdVel.linear.y;
  //lastTwistStamp = millis();
  ledErr(); // TODO: not an erro, just want red blink
            // need rename/rework led indication
}
//*****************************************************************




//*****************************************************************
// rosserial callback for setting servo position
void servoCb( const rover_msgs::SBMCServoSetting &pos) {
  servo.write(max(0, min(pos.position,180))); 
}
//*****************************************************************



// rosserial pubs, subs and messages
ros::NodeHandle nh; //_<SBMCS_Hardware,5, 5, 512, 512>
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
geometry_msgs::PoseStamped imu_msg;
nav_msgs::Odometry odom_msg;
rover_msgs::SBMCTelemReading telem_msg;
ros::Publisher pub_imu( "imu", &imu_msg);
ros::Publisher pub_odom( "odom", &odom_msg);
ros::Publisher pub_telem( "telem", &telem_msg);
ros::Subscriber<geometry_msgs::Twist> sub_drive("/drive_setting", twistCb);
ros::Subscriber<rover_msgs::SBMCServoSetting> sub_servo("/servo_setting", servoCb);

// IMU frame, need a transformer for this to base_link or odom??
char imu_frame_id[] = "sbmcs_imu";
// odom frame
char base_link_frame_id[] = "base_link";
char odom_frame_id[] = "odom";

void ledErr() {
  digitalWrite(STAT_LED, HIGH);
}
void ledOk() {
  digitalWrite(STAT_LED, LOW);
  ledOk(false);
}
void ledOk(bool toggle) {
  digitalWrite(STAT_LED, LOW);
  digitalWrite(ACT_LED, !digitalRead(ACT_LED));
}

float normalizeAngle(float angle) {
  while( angle > PI )
    angle -= 2.0 * PI;
  
  while( angle < -PI )
    angle += 2.0 * PI;
  
  return angle;
}

// set servo position in degrees
void setServoPosition(int pos) {
  servo.write(pos);
};

void initializeIMU()
{
  // setup ros message
  imu_msg.header.frame_id =  odom_frame_id;
   
  // TODO: return initialized 
  bool initialized = false;
  while( !initialized ){
    ledErr();
    if( !myICM.begin(ICM20948_ACCELRANGE_4G, GYRO_RANGE_1000DPS, ICM20948_ACCELLOWPASS_50_4_HZ, IMU_ADDR) ){
      ledErr();
      delay(500);
    }else{
      initialized = true;
    }
    ledOk();
  }
  ledOk();

  // expecting data at 100hz
  filter.begin();
}

//*****************************************************************
// Task will periodically print out useful information about the tasks running
// Is a useful tool to help figure out stack sizes being used
// Run time stats are generated from all task timing collected since startup
// No easy way yet to clear the run time stats yet
//*****************************************************************
static char ptrTaskList[400]; //temporary string bufer for task stats

void taskMonitor(void *pvParameters)
{
  int x;
  int measurement;
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    SERIAL.println("Task Monitor: Started");
    xSemaphoreGive( dbSem );
  }
  // run this task afew times before exiting forever
  while(1)
  {
  	myDelayMs(10000); // print every 10 seconds

    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    	SERIAL.println("****************************************************");
    	SERIAL.print("Free Heap: ");
    	SERIAL.print(xPortGetFreeHeapSize());
    	SERIAL.println(" bytes");

    	SERIAL.print("Min Heap: ");
    	SERIAL.print(xPortGetMinimumEverFreeHeapSize());
    	SERIAL.println(" bytes");

    	SERIAL.println("****************************************************");
    	SERIAL.println("Task            ABS             %Util");
    	SERIAL.println("****************************************************");

    	vTaskGetRunTimeStats(ptrTaskList); //save stats to char array
    	SERIAL.println(ptrTaskList); //prints out already formatted stats

	    SERIAL.println("****************************************************");
	    SERIAL.println("Task            State   Prio    Stack   Num     Core" );
	    SERIAL.println("****************************************************");

	    vTaskList(ptrTaskList); //save stats to char array
	    SERIAL.println(ptrTaskList); //prints out already formatted stats

	    SERIAL.println("****************************************************");
	    SERIAL.println("[Stacks Free Bytes Remaining] ");

	    measurement = uxTaskGetStackHighWaterMark( Handle_telemTask );
	    SERIAL.print("Motor Controller Monitoring: ");
	    SERIAL.println(measurement);

	    measurement = uxTaskGetStackHighWaterMark( Handle_imuTask );
	    SERIAL.print("IMU fusion and publishing: ");
	    SERIAL.println(measurement);
  
	    measurement = uxTaskGetStackHighWaterMark( Handle_speedTask );
	    SERIAL.print("Speed + Position Control: ");
	    SERIAL.println(measurement);
	    
	    measurement = uxTaskGetStackHighWaterMark( Handle_odomTask );
	    SERIAL.print("Odom publish Task: ");
	    SERIAL.println(measurement);
	    
	    measurement = uxTaskGetStackHighWaterMark( Handle_monitorTask );
	    SERIAL.print("Monitor Stack: ");
	    SERIAL.println(measurement);

	    SERIAL.println("****************************************************");
      xSemaphoreGive( dbSem );
    }
  }

  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  vTaskDelete( NULL );
}


//******************************************************************************
// PID control loop to keep motors at the set speed in m/s
// speed calibration must be preformed to create mapping between PID output in m/s
// and the PWM value necessary to achieve that speed
//******************************************************************************
static void speedThread( void *pvParameters )
{
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
    Serial.println("Speed thread started");
    xSemaphoreGive( dbSem );
  }
  #endif

  int32_t enc1LastValue = shield.enc1(), enc2LastValue = shield.enc2();
  int32_t enc1Value = enc1LastValue, enc2Value = enc2LastValue;

  //turn the PID on
  m1PID.SetMode(AUTOMATIC);
  m2PID.SetMode(AUTOMATIC);

  // enable to walk through pwm values so that a mapping between
  // motor pwm and m/s can be made
  //bool doSpeedCal = false;
  //int pos = 0;

  // zero out initial setpoints just to be sure
  m1Setpoint = 0.0;
  m2Setpoint = 0.0;
  m1Input = 0.0;
  m2Input = 0.0;

  motors.setM1Dir(REVERSE);
  motors.setM2Dir(REVERSE);
  
  while(1) {
    /*
    if( doSpeedCal ){
      Serial.print(pos); Serial.print('\t');
      motors.setM1Pwm(pos);
      motors.setM2Pwm(pos);
      pos += 2;
      if(pos >= 256) pos = 0;
    }*/
    
    myDelayMs(20);
    
    // TODO: impliment better heartbeat to avoid motors running 
    // during a comms blackout
    // check that we get at least one twist per second
    //if (millis() - lastTwistStamp > 1000) {
    //  m1Setpoint = 0;
    //  m2Setpoint = 0;
    //}

    // keep the PID controller from whining the motors
    if( abs(m1Setpoint) < MIN_SPD_THRESH )
      motors.disableM1();
    else 
      motors.enableM1();
    
    if( abs(m2Setpoint) < MIN_SPD_THRESH )
      motors.disableM2();
    else
      motors.enableM2();

    enc1Value = shield.enc1();
    enc2Value = shield.enc2();
    int enc1Diff = -1 * (enc1Value  - enc1LastValue);
    int enc2Diff = (enc2Value  - enc2LastValue);
    enc1LastValue = enc1Value;
    enc2LastValue = enc2Value;

    // calculate speeds in m/s
    float m1Spd = 50 * enc1Diff / ENCODER_CPR * WHEEL_CIRC;
    float m2Spd = 50 * enc2Diff / ENCODER_CPR * WHEEL_CIRC;

    m1Setpoint = m1TargetVel;
    m2Setpoint = m2TargetVel;
    m1Input = (double)m1Spd;
    m2Input = (double)m2Spd;

    m1PID.Compute();
    m2PID.Compute();


    // TODO: do another speed calibration that includes both
    //       directions. inverting  a one directional mapping like
    //       this jerks the motors on speeds close to zero sometimes 
    float m1PwmEstimate = 195.414*m1Output*2.0 + 6.73-255;
    float m2PwmEstimate = 189.719*m2Output*2.0 + 5.49-255;
    m1PwmEstimate = max(-255, min(m1PwmEstimate, 255));
    m2PwmEstimate = max(-255, min(m2PwmEstimate, 255));

    // set motor direction based on sign of setpoint
    // this method might cause jerkiness
    if( m1PwmEstimate > 0 ) {
      motors.setM1Dir(FORWARD);
    } else {
      motors.setM1Dir(REVERSE);
    }
    if( m2PwmEstimate > 0 ){
      motors.setM2Dir(FORWARD);
    } else {
      motors.setM2Dir(REVERSE);
    }
    
    motors.setM1Pwm(abs(m1PwmEstimate));
    motors.setM2Pwm(abs(m2PwmEstimate));
  }
  
  vTaskDelete( NULL );
}


//*************************************************************************************
// Odometry calculation and publishing thread
//*************************************************************************************
static void odomThread( void *pvParameters )
{
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Odom thread started");
    xSemaphoreGive( dbSem ); // Now free or "Give" the Serial Port for others.
  }
  #endif
  
  int32_t enc1LastValue = shield.enc1(), enc2LastValue = shield.enc2();
  int32_t enc1Value = enc1LastValue, enc2Value = enc2LastValue;
  float curX = 0.0;
  float curY = 0.0;
  float curTheta = 0.0;
  
  while(1) {
    // odometry calculation
    // TODO: make shield mutex, and encoder reading helper func
    enc1Value = shield.enc1();
    enc2Value = shield.enc2();

    // invert so that encoder direction is wrt motor 2
    int enc1Diff = -1 * (enc1Value  - enc1LastValue);
    int enc2Diff =  (enc2Value  - enc2LastValue);

    //Serial.print("enc1val: "); Serial.println(enc1Value);
    //Serial.print("enc2val: "); Serial.println(enc2Value);
    //Serial.print("enc1lastval: "); Serial.println(enc1LastValue);;
    //Serial.print("enc2lastVal: "); Serial.println(enc2LastValue);
    //Serial.print("enc1diff: "); Serial.println(enc1Diff);
    //Serial.print("enc2diff: "); Serial.println(enc2Diff);
    
    // keep track of encoder values
    enc1LastValue = enc1Value;
    enc2LastValue = enc2Value;
    
    float enc1Dist = (float)enc1Diff / ENCODER_CPM;
    float enc2Dist = (float)enc2Diff / ENCODER_CPM;
    float dist = (enc2Dist + enc1Dist) / 2.0;


    //Serial.print("dist: "); Serial.println(dist);
    
    float dTime = 0.1;
    float dTheta = 0.0;
    float r;
    
    // TODO find better what to determine going straight, this means slight deviation is accounted
    if( enc1Diff == enc2Diff ){
      dTheta = 0.0;
      curX += dist * cos(curTheta);
      curY += dist * sin(curTheta);
    } else {
      dTheta = (enc2Dist - enc1Dist) / BASE_WIDTH;
      r = dist / dTheta;
      curX += r * (sin(dTheta + curTheta) - sin(curTheta));
      curY -= r * (cos(dTheta + curTheta) - cos(curTheta));
      curTheta = normalizeAngle(curTheta + dTheta);
    }
    
    
    float velX = dist / dTime;
    float velTheta = dTheta / dTime;

    //Serial.print("velX: "); Serial.print(velX);
    //Serial.print("\nvelTheta: "); Serial.println(velTheta);
    
    
    // fill in transform fields
    t.header.frame_id = odom_frame_id;
    t.child_frame_id = base_link_frame_id;
  
    t.transform.translation.x = curX;
    t.transform.translation.y = curY;
    
    t.transform.rotation = tf::createQuaternionFromYaw(curTheta);
    
    odom_msg.header.frame_id = odom_frame_id;

    odom_msg.pose.pose.position.x = curX;
    odom_msg.pose.pose.position.y = curY;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.covariance[0] = 0.01;
    odom_msg.pose.covariance[7] = 0.01;
    odom_msg.pose.covariance[14] = 99999;
    odom_msg.pose.covariance[21] = 99999;
    odom_msg.pose.covariance[28] = 99999;
    odom_msg.pose.covariance[35] = 0.01;

    odom_msg.child_frame_id = base_link_frame_id;
    odom_msg.twist.twist.linear.x = velX;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = velTheta;
    //odom_msg.twist.covariance = odom.pose.covariance;
    odom_msg.twist.covariance[0] = 0.01;
    odom_msg.twist.covariance[7] = 0.01;
    odom_msg.twist.covariance[14] = 99999;
    odom_msg.twist.covariance[21] = 99999;
    odom_msg.twist.covariance[28] = 99999;
    odom_msg.twist.covariance[35] = 0.01;
    
    // acquire semaphore for rosserial and send transform message
    // TODO: how long to wait ? posssibly just as long as we dont
    //       overrun the following delay?
    // Timestamps are set inside this mutex because the the now() 
    //        function uses the node handle object
    if ( xSemaphoreTake( rsSem, ( TickType_t ) 10 ) == pdTRUE ) {
      // publish the transform from base_link to odom
      t.header.stamp = nh.now();
      broadcaster.sendTransform(t);
      
      // publish an Odometry message in the odom frame
      odom_msg.header.stamp = nh.now();
      // odom pose estimate is just our current theta estimate
      odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(curTheta);
      pub_odom.publish(&odom_msg);
      
      nh.spinOnce();
      xSemaphoreGive( rsSem );
    }
    
    // delay 1/dTime seconds
    // not as precise as with timer but with 120Mhz processor
    // its not too bad
    myDelayMs(100);
  }

}

//*************************************************************************************
// Telemetry thread, creates telemetry on motor currents and battery voltage, as well
// as system 5v current draw
//*************************************************************************************

static void telemThread( void *pvParameters )
{
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Telemetry thread started");
    xSemaphoreGive( dbSem ); // Now free or "Give" the Serial Port for others.
  }
  #endif
  
  // enable motor controller
  motors.enable();

  while(1) {
    myDelayMs(200);
    motors.update();
    
    telem_msg.m1_current = motors.getM1Current();
    telem_msg.m2_current = motors.getM2Current();
    telem_msg.battery_voltage = shield.getBatteryVoltage();
    telem_msg.system_current = shield.get5vCurrent();
    
    if( motors.fault() ){
      motors.disable();
    }
    
    if ( xSemaphoreTake( dbSem, ( TickType_t ) 300 ) == pdTRUE ) {
    
      // TODO: make message format and publisher for battry voltage
      // and motor 1 and 2 current reading, and 5v current draw
      // maybe power_telemetry or something?
      Serial.print("M1 current: ");
      Serial.print(motors.getM1Current());
      Serial.println(" amps");
      
      Serial.print("M2 current: ");
      Serial.print(motors.getM2Current());
      Serial.println(" amps");
      
      if( motors.fault() ){
        Serial.println("motor fault, disabling both motors");
      }
      

      Serial.print("Vbat voltage: ");
      Serial.print(shield.getBatteryVoltage());
      Serial.println(" volts");
      
      xSemaphoreGive( dbSem );
    }
    
    // check rosserial publish semaphore wait 5 ticks if not available
    if ( xSemaphoreTake( rsSem, ( TickType_t ) 5 ) == pdTRUE ) {
      // publish the imu message 
      pub_telem.publish(&telem_msg);
      nh.spinOnce();
      xSemaphoreGive( rsSem );
    }
      
    // tasks can override each other with blinks without semaphore
    // but eh more blinks
    ledOk(true);
  }
  
  motors.disable();
  vTaskDelete( NULL );  
}

static void imuThread( void *pvParameters )
{
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("IMU thread started");
    xSemaphoreGive( dbSem ); // Now free or "Give" the Serial Port for others.
  }
  #endif
  
  unsigned long now = 0, lastNow = 0;

  // continually get most recent IMU data and update AHRS filter
  while(1) {
  
    unsigned long now = millis();
    
    if( now - lastNow > IMU_PERIOD ){
      lastNow = now;
      
      sensors_event_t accel_event;
      sensors_event_t gyro_event;
      sensors_event_t mag_event;

      // Get new data samples
      myICM.getEvent(&accel_event, &gyro_event, &mag_event);

      // Apply mag offset compensation (base values in uTesla)
      float x = mag_event.magnetic.x - mag_offsets[0];
      float y = mag_event.magnetic.y - mag_offsets[1];
      float z = mag_event.magnetic.z - mag_offsets[2];

      // Apply mag soft iron error compensation
      float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
      float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
      float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

      // Apply gyro zero-rate error compensation
      float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
      float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
      float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

      // Update the filter
      filter.update(gx, gy, gz,
                    accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                    mx, my, mz);
      
      float roll = filter.getRoll();
      float pitch = filter.getPitch();
      float heading = filter.getYaw();
        
      geometry_msgs::Quaternion q;
      
      float c1 = cos(pitch/2);
      float c2 = cos(heading/2);
      float c3 = cos(roll/2);

      float s1 = sin(pitch/2);
      float s2 = sin(heading/2);
      float s3 = sin(roll/2);

      q.x = c1 * c2 * c3 - s1 * s2 * s3;
      q.y = s1 * s2 * c3 + c1 * c2 * s3;
      q.z = s1 * c2 * c3 + c1 * s2 * s3;
      q.w = c1 * s2 * c3 - s1 * c2 * s3;
      
      // check rosserial publish semaphore wait 5 ticks if not available
      if ( xSemaphoreTake( rsSem, ( TickType_t ) 5 ) == pdTRUE ) {
        // set timestamp inside semaphore since it uses the nh object
        imu_msg.header.stamp = nh.now();
      
        // TODO: quick and dirty way to not have a separate mutex just for the quat
        //       keeps odom thread from possibly interfering while
        //       grabbing the most recent orientation for the Odometry message
        imu_msg.pose.orientation = q;
        
        // publish the imu message 
        pub_imu.publish(&imu_msg);
        nh.spinOnce();
        xSemaphoreGive( rsSem );
      }
      
      // tasks can override each other with blinks without semaphore
      // but eh more blinks
      ledOk(true);
    } else {
      myDelayMs(min(IMU_PERIOD-5, max( 0, IMU_PERIOD - (now - lastNow))));
    }
  }
    
  vTaskDelete( NULL );  
}

void setup()
{
  Serial.begin(115200);
  delay(3000);
  
  Serial.println("Starting..");
  
  pinMode(STAT_LED, OUTPUT);
  pinMode(ACT_LED, OUTPUT);
  
  ledErr();
  nh.getHardware()->setBaud(115000);  
  nh.initNode();
  broadcaster.init(nh);
  nh.advertise(pub_imu);
  nh.advertise(pub_odom);
  nh.advertise(pub_telem);
  nh.subscribe(sub_drive);
  nh.subscribe(sub_servo);
  ledOk();

  
  Serial.print("Starting shield...");
  ledErr();
  shield.begin();
  ledOk();
  Serial.println("Done");
  
  delay(1000);

  // init servo library
  // must be using the servo library that comes with adafruits
  // board support package. Other servo libs do not support the
  // samd51 timer structure
  ledErr();
  Serial.print("Starting servo...");
  servo.attach(SERVO_PWM);
  ledOk();
  Serial.println("done");
  
  // init mc33932 dual h bridge control library
  ledErr();
  Serial.print("Starting motors...");
  motors.begin();    
  motors.disable();
  ledOk();
  Serial.println("done");


  ledErr();
  Serial.print("starting IMU");
  initializeIMU();
  ledOk();
  Serial.println("done");
  
  
  // setup rosserial publish semaphore
  if ( rsSem == NULL ) {
    rsSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( rsSem ) != NULL )
      xSemaphoreGive( ( rsSem ) );  // make available
  }
  
  
  // setup debug log semaphore
  if ( dbSem == NULL ) {
    dbSem = xSemaphoreCreateMutex();  // create mutex
    if ( ( dbSem ) != NULL )
      xSemaphoreGive( ( dbSem ) );  // make available
  }
  
  // Begin RTOS Setup
  vSetErrorLed(STAT_LED, 1);

  xTaskCreate(telemThread, "Telemetry Logging", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_telemTask);
  xTaskCreate(speedThread, "Speed and Position Control", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_speedTask);
  xTaskCreate(odomThread, "Odometry Publishing", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_odomTask);
  xTaskCreate(imuThread, "IMU Processing", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_imuTask);
  xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 3, &Handle_monitorTask);

  Serial.println("Created Tasks");

  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();

  
  // error scheduler failed to start
  while(1)
  {
	  SERIAL.println("Scheduler Failed! \n");
	  delay(1000);
  }
}

void loop()
{  

}
