/* 
 * SBMCS Motor Firmware
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
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <rover_msgs/SBMCTelemReading.h>
#include <rover_msgs/SBMCServoSetting.h>

#include <MadgwickAHRS.h>
#include "ICM_20948.h"

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

// shield library interface
SBMCS shield;
Servo servo;
MC33932 motors;

// IMU reading and orientation filtering
ICM_20948_I2C myICM;

Madgwick filter;


// begin PID control interface and tunings
// need mutexes for these 
double m1Setpoint, m1Input, m1Output;
double m2Setpoint, m2Input, m2Output;

//Define the aggressive and conservative Tuning Parameters
double consKp=0.1, consKi=4, consKd=0.0;

//Specify the links and initial tuning parameters
PID m1PID(&m1Input, &m1Output, &m1Setpoint, consKp, consKi, consKd, DIRECT);
PID m2PID(&m2Input, &m2Output, &m2Setpoint, consKp, consKi, consKd, DIRECT);
// end pid

//**************************************************************************
// freertos task handles
TaskHandle_t Handle_motorTask;
TaskHandle_t Handle_speedTask;
TaskHandle_t Handle_imuTask;
TaskHandle_t Handle_monitorTask;
TaskHandle_t Handle_odomTask;
// ros serial publish semaphore (Serial1)
SemaphoreHandle_t rsSem;
// serial debug logging (Serial)
SemaphoreHandle_t dbSem;



// callback for setting drive velocity
void twistCb( const geometry_msgs::Twist &cmdVel ){
  // joy_twist_remap was need to send linear.x and angular.z
  
  float linear_x = cmdVel.linear.x;

  float vr = linear_x + cmdVel.angular.z * BASE_WIDTH / 2.0;
  float vl = linear_x - cmdVel.angular.z * BASE_WIDTH / 2.0;

  m1Setpoint = vr; //cmdVel.linear.y;
  m2Setpoint = vl; //cmdVel.linear.y;
}

void servoCb( const rover_msgs::SBMCServoSetting &pos) {
  servo.write(max(0, min(pos.position,180))); 
}

// rosserial pubs, subcs and messages
ros::NodeHandle nh; //_<SBMCS_Hardware,5, 5, 512, 512>
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
sensor_msgs::Imu imu_msg;
nav_msgs::Odometry odom_msg;
rover_msgs::SBMCTelemReading telem_msg;
ros::Publisher pub_imu( "imu_data", &imu_msg);
ros::Publisher pub_odom( "odom", &odom_msg);
ros::Publisher pub_telem( "sbmc_telem", &telem_msg);
ros::Subscriber<geometry_msgs::Twist> sub_drive("/drive_setting", twistCb);
ros::Subscriber<rover_msgs::SBMCServoSetting> sub_servo("/servo_setting", servoCb);

// IMU frame, need a transformer for this to base_link or odom??
char imu_frame_id[] = "sbmcs_imu";
// odom frame
char base_link[] = "base_link";
char odom[] = "odom";

void ledErr() {
  digitalWrite(STAT_LED, HIGH);
  digitalWrite(ACT_LED, LOW);
}
void ledOk() {
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
  imu_msg.header.frame_id =  imu_frame_id;
  imu_msg.linear_acceleration.x = 0;
  imu_msg.linear_acceleration.y = 0;
  imu_msg.linear_acceleration.z = 0;
  imu_msg.angular_velocity.x = 0;
  imu_msg.angular_velocity.y = 0;
  imu_msg.angular_velocity.z = 0;
  
  WIRE_PORT.begin();
  //WIRE_PORT.setClock(400000);
   
  // TODO: return initialized 
  bool initialized = false;
  while( !initialized ){
    ledErr();
    myICM.begin( WIRE_PORT, AD0_VAL );

    //SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
    //SERIAL_PORT.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      //SERIAL_PORT.println( "Trying again..." );
      ledErr();
      delay(500);
    }else{
      initialized = true;
    }
    ledOk();
  }
  ledOk();
    
    
  // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
  //SERIAL_PORT.println("Device connected!");

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset( );
  if( myICM.status != ICM_20948_Stat_Ok){
    //SERIAL_PORT.print(F("Software Reset returned: "));
    //SERIAL_PORT.println(myICM.statusString());
    ledErr();
  }
  delay(250);
  
  // Now wake the sensor up
  myICM.sleep( false );
  myICM.lowPower( false );

  // setup sample mode and check return status
  ledErr();
  myICM.setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr ), ICM_20948_Sample_Mode_Continuous); 
  if( myICM.status != ICM_20948_Stat_Ok){
    //SERIAL_PORT.print(F("setSampleMode returned: "));
    //SERIAL_PORT.println(myICM.statusString());
    ledErr();
  }
  ledOk();
  
  // 1125 Hz / (1 + SAMPLE_RATE_DIV) where SAMPLE_RATE_DIV=14 => 75Hz output data rate
  ledErr();
  ICM_20948_smplrt_t sample_rates;
  sample_rates.a = 50;
  sample_rates.g = 50;
  myICM.setSampleRate( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr ), sample_rates ); 
  if( myICM.status != ICM_20948_Stat_Ok){
    //SERIAL_PORT.print(F("setSampleRate returned: "));
    //SERIAL_PORT.println(myICM.statusString());
    ledErr();
  }
  ledOk();

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  
  myFSS.a = gpm4;         // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                          // gpm2
                          // gpm4
                          // gpm8
                          // gpm16
                          
  myFSS.g = dps1000;       // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                          // dps250
                          // dps500
                          // dps1000
                          // dps2000
  ledErr();                          
  myICM.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );  
  if( myICM.status != ICM_20948_Stat_Ok){
    //SERIAL_PORT.print(F("setFullScale returned: "));
    //SERIAL_PORT.println(myICM.statusString());
    ledErr();
  }
  ledOk();

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;            // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d23bw9_n34bw4;         // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                          // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                          // acc_d111bw4_n136bw
                                          // acc_d50bw4_n68bw8
                                          // acc_d23bw9_n34bw4
                                          // acc_d11bw5_n17bw
                                          // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                          // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d361bw4_n376bw5;       // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                          // gyr_d196bw6_n229bw8
                                          // gyr_d151bw8_n187bw6
                                          // gyr_d119bw5_n154bw3
                                          // gyr_d51bw2_n73bw3
                                          // gyr_d23bw9_n35bw9
                                          // gyr_d11bw6_n17bw8
                                          // gyr_d5bw7_n8bw9
                                          // gyr_d361bw4_n376bw5
  ledErr();
  myICM.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );
  if( myICM.status != ICM_20948_Stat_Ok){
    //SERIAL_PORT.print(F("setDLPcfg returned: "));
    //SERIAL_PORT.println(myICM.statusString());
    ledErr();
  }
  ledOk();
  
  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF( ICM_20948_Internal_Acc, true );
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF( ICM_20948_Internal_Gyr, true );
  //SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: ")); SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
  //SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: ")); SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));

  //SERIAL_PORT.println();
  //SERIAL_PORT.println(F("Configuration complete!")); 
  ledOk();
  
  // expecting data at 75hz
  filter.begin(23);
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

	    measurement = uxTaskGetStackHighWaterMark( Handle_motorTask );
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

    // keep the PID controller from whining the motors
    if( m1Setpoint == 0 && m2Setpoint == 0)
      motors.disable();
    else
      motors.enable();

    enc1Value = shield.enc1();
    enc2Value = shield.enc2();
    int enc1Diff = -1 * (enc1Value  - enc1LastValue);
    int enc2Diff = (enc2Value  - enc2LastValue);
    enc1LastValue = enc1Value;
    enc2LastValue = enc2Value;

    // calculate speeds in m/s
    float m1Spd = 50 * enc1Diff / ENCODER_CPR * WHEEL_CIRC;
    float m2Spd = 50 * enc2Diff / ENCODER_CPR * WHEEL_CIRC;

    m1Input = (double)m1Spd;
    m2Input = (double)m2Spd;

    m1PID.Compute();
    m2PID.Compute();


    // TODO: this jerks the motors on startup 
    float m1PwmEstimate = 195.414*m1Output*2.0 + 6.73-255;
    float m2PwmEstimate = 189.719*m2Output*2.0 + 5.49-255;
    m1PwmEstimate = max(-255, min(m1PwmEstimate, 255));
    m2PwmEstimate = max(-255, min(m2PwmEstimate, 255));

    //Serial.print("m1 pwm estimate: ");
    //Serial.print(m1PwmEstimate);
    //Serial.print("\nm2 pwm estimate: ");
    //Serial.println(m2PwmEstimate);
    
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
    
    /*    if ( xSemaphoreTake( dbSem, ( TickType_t ) 1000 ) == pdTRUE ) {
      Serial.print(m1Spd);
      Serial.print('\t');
      Serial.print(m2Spd);
      Serial.print('\n');
      xSemaphoreGive( dbSem );
      }*/
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
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
  
    t.transform.translation.x = curX;
    t.transform.translation.y = curY;
    
    t.transform.rotation = tf::createQuaternionFromYaw(curTheta);
    
    odom_msg.header.frame_id = odom;

    odom_msg.pose.pose.position.x = curX;
    odom_msg.pose.pose.position.y = curY;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.covariance[0] = 0.01;
    odom_msg.pose.covariance[7] = 0.01;
    odom_msg.pose.covariance[14] = 99999;
    odom_msg.pose.covariance[21] = 99999;
    odom_msg.pose.covariance[28] = 99999;
    odom_msg.pose.covariance[35] = 0.01;

    odom_msg.child_frame_id = base_link;
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
    if ( xSemaphoreTake( rsSem, ( TickType_t ) 5 ) == pdTRUE ) {
      // publish the transform from base_link to odom
      t.header.stamp = nh.now();
      broadcaster.sendTransform(t);
      
      // publish an Odometry message in the odom frame
      odom_msg.header.stamp = nh.now();
      // set orientation inside the mutex so we dont read a mangled 
      // quat due to imu thread getting time sliced
      odom_msg.pose.pose.orientation = imu_msg.orientation;
      pub_odom.publish(&odom_msg);
      
      nh.spinOnce();
      xSemaphoreGive( rsSem ); // Now free or "Give" the Serial Port for others.
    }
    
    // delay 1/dTime seconds
    // not as precise as with timer but with 120Mhz processor
    // its not too bad
    myDelayMs(100);
  }

}

//*************************************************************************************
// Motor thread, creates telemetry on motor currents and battery voltage, as well
// as system 5v current draw
// TODO: create separate telemetry thread or rename this thread to be more telemetry in
//       general
//*************************************************************************************

static void motorThread( void *pvParameters )
{
  #ifdef DEBUG
  if ( xSemaphoreTake( dbSem, ( TickType_t ) 100 ) == pdTRUE ) {
    Serial.println("Motor thread started");
    xSemaphoreGive( dbSem ); // Now free or "Give" the Serial Port for others.
  }
  #endif
  
  // enable motor controller
  motors.enable();

  while(1) {
    myDelayMs(1000);
    motors.update();
    
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
        motors.disable();
      }
      

      Serial.print("Vbat voltage: ");
      Serial.print(shield.getBatteryVoltage());
      Serial.println(" volts");
      
      xSemaphoreGive( dbSem );
    }
  }
  
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

  while(1) {
    
    // only update IMU and send ros messages when there is new data
    // TODO: protect I2C with semaphore
    if( myICM.dataReady() ){
      //Serial.println("getting data");
      
      myICM.getAGMT(); 
      
      imu_msg.linear_acceleration.x = myICM.accX();
      imu_msg.linear_acceleration.y = myICM.accY();
      imu_msg.linear_acceleration.z = myICM.accZ();
      imu_msg.angular_velocity.x = myICM.gyrX();
      imu_msg.angular_velocity.y = myICM.gyrY();
      imu_msg.angular_velocity.z = myICM.gyrZ();
      
      filter.updateIMU(imu_msg.angular_velocity.x,
                    imu_msg.angular_velocity.y,
                    imu_msg.angular_velocity.z,
                    imu_msg.linear_acceleration.x,
                    imu_msg.linear_acceleration.y,
                    imu_msg.linear_acceleration.z);/*,
                    myICM.magX(),
                    myICM.magY(),
                    myICM.magZ());*/
                    
      float roll = filter.getRoll();
      float pitch = filter.getPitch();
      float heading = filter.getYaw();
      
      imu_msg.header.stamp = nh.now();
      
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
        // TODO: quick and dirty way to not have a separate mutex just for the quat
        //       keeps odom thread from possibly interfering while
        //       grabbing the most recent orientation for the Odometry message
        imu_msg.orientation = q;
        
        // publish the imu message 
        pub_imu.publish(&imu_msg);
        nh.spinOnce();
        xSemaphoreGive( rsSem ); // Now free or "Give" the Serial Port for others.
      }
      
      // tasks can override each other with blinks without semaphore
      // but eh more blinks
      ledOk(true);

      
    } else {
      myDelayMs(5);
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
  nh.getHardware()->setBaud(128000);  
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

  xTaskCreate(motorThread, "Motor Controller Telemetry", 512, NULL, tskIDLE_PRIORITY + 2, &Handle_motorTask);
  xTaskCreate(speedThread, "Speed and Position Control", 512, NULL, tskIDLE_PRIORITY + 3, &Handle_speedTask);
  xTaskCreate(odomThread, "Odometry Publishing", 512, NULL, tskIDLE_PRIORITY + 2, &Handle_odomTask);
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
