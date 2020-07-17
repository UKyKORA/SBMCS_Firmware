// SBMCS Firmware
// v1
// Matt Ruffner
#define DEBUG 1


#include <FreeRTOS_SAMD51.h>
#include <PID_v1.h>

#include "sbmcs.h"

#define SERIAL Serial

// ROBOT PARAMS
// for 165 rpm servo city plantetary gear motor with encoder
#define ENCODER_CPR 2442.96 // counts per revolution
// wheel circumference
#define WHEEL_CIRC 0.468 // meters


// begin PID control interface and tunings
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID m1PID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
//PID m2PID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
// end pid



// freertos task handles
TaskHandle_t Handle_motorTask;
TaskHandle_t Handle_speedTask;
TaskHandle_t Handle_imuTask;
TaskHandle_t Handle_monitorTask;

// rtos thread delay helpers
//**************************************************************************
void myDelayUs(int us)
{
  vTaskDelay( us / portTICK_PERIOD_US );  
}

void myDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );  
}

void myDelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_US );  
}
//**************************************************************************

SBMCS shield;

//Encoder m1(M1_ENC_A, M1_ENC_B);
//Encoder m2(M2_ENC_A, M2_ENC_B);

static void motorThread( void *pvParameters )
{
  #ifdef DEBUG
  Serial.println("Motor thread started");
  #endif

  while(1) {
    myDelayMs(200);
    shield.motors.update();
    /*
    Serial.print("M1 current: ");
    Serial.print(shield.motors.getM1Current());
    Serial.println(" amps");
    
    Serial.print("M2 current: ");
    Serial.print(shield.motors.getM2Current());
    Serial.println(" amps");
    
    if( shield.motors.fault() ){
      Serial.println("motor fault, disabling both motors");
      shield.motors.disable();
    }

    Serial.print("Vbat voltage: ");
    Serial.print(shield.getBatteryVoltage());
    Serial.println(" volts");
    */
  }
  
  vTaskDelete( NULL );  
}

static void speedThread( void *pvParameters )
{
  #ifdef DEBUG
  Serial.println("Speed thread started");
  #endif

  unsigned long enc1LastValue = shield.enc1(), enc2LastValue = shield.enc2();
  unsigned long enc1Value, enc2Value;

  // init pid control of motor pwm values based on encoder readings
  // initialize the variables we're linked to
  Input = 0;
  Setpoint = 100;

  //turn the PID on
  m1PID.SetMode(AUTOMATIC);

  bool doSpeedCal = true;
  int pos = 0;
  
  while(1) {
    
    if( doSpeedCal ){
      Serial.print(pos); Serial.print('\t');
      shield.motors.setM1Pwm(pos);
      shield.motors.setM2Pwm(pos);
      pos += 2;
      if(pos >= 256) pos = 0;
    }

    myDelayMs(1000);

    enc1Value = shield.enc1();
    enc2Value = shield.enc2();
    int enc1diff = enc1Value  - enc1LastValue;
    int enc2diff = enc2Value  - enc2LastValue;
    enc1LastValue = enc1Value;
    enc2LastValue = enc2Value;

    // calculate speeds in m/s
    float m1Spd = enc1diff / ENCODER_CPR * WHEEL_CIRC;
    // invert m2 speed to make relative to m1's direction
    // to account for motors being mounted opposed each other
    float m2Spd = enc2diff / ENCODER_CPR * WHEEL_CIRC * -1;
    
    Serial.print(m1Spd);
    Serial.print('\t');
    Serial.print(m2Spd);
    Serial.print('\n');
  }
  
  vTaskDelete( NULL );
}

static void imuThread( void *pvParameters )
{
  #ifdef DEBUG
  Serial.println("IMU thread started");
  #endif

  while(1) {
    myDelayMs(200);

    // read IMU data and process with madgwick ahrs filter
    // to get yaw/pitch/roll
  }
  
  vTaskDelete( NULL );
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
    
    SERIAL.println("Task Monitor: Started");

    // run this task afew times before exiting forever
    while(1)
    {
    	myDelayMs(10000); // print every 10 seconds

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
		SERIAL.print("Current and fault monitoring: ");
		SERIAL.println(measurement);

		measurement = uxTaskGetStackHighWaterMark( Handle_speedTask );
		SERIAL.print("Speed Control: ");
		SERIAL.println(measurement);

		measurement = uxTaskGetStackHighWaterMark( Handle_speedTask );
		SERIAL.print("IMU Porcessing: ");
		SERIAL.println(measurement);
		
		measurement = uxTaskGetStackHighWaterMark( Handle_monitorTask );
		SERIAL.print("Monitor Stack: ");
		SERIAL.println(measurement);

		SERIAL.println("****************************************************");

    }

    // delete ourselves.
    // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
    SERIAL.println("Task Monitor: Deleting");
    vTaskDelete( NULL );

}

void setup()
{
  Serial.begin(115200);

  delay(1000);
  
  shield.begin();
  
  delay(1000);

  digitalWrite(19, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second

  shield.motors.enable();
  delay(1000);
  
  shield.motors.setM1Pwm(40);
  shield.motors.setM2Pwm(38);

  //shield.servo.write(0);

  vSetErrorLed(STAT_LED, 1);

  xTaskCreate(motorThread, "Current and Fault Monitoring", 2000, NULL, tskIDLE_PRIORITY + 1, &Handle_motorTask);
  xTaskCreate(speedThread, "Speed and Position Control", 2000, NULL, tskIDLE_PRIORITY + 3, &Handle_motorTask);
  xTaskCreate(imuThread, "IMU Processing", 2000, NULL, tskIDLE_PRIORITY + 3, &Handle_motorTask);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 1, &Handle_monitorTask);

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
  
  //Serial.println(m1.read());
  //Serial.println(m2.read());

  
  //Serial.print("5V current: ");
  //Serial.print(shield.get5vCurrent());
  //Serial.println(" amps");
  


  
  digitalWrite(19, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(200);               // wait for a second
  digitalWrite(36, HIGH);    // turn the LED off by making the voltage LOW
  delay(200);               // wait for a second
  digitalWrite(19, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(200);               // wait for a second
  digitalWrite(36, LOW);    // turn the LED off by making the voltage LOW
  delay(200);
}
