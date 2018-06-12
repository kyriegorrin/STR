#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <portmacro.h>
#include <FreeRTOSVariant.h>
#include <croutine.h>
#include <task.h>

#include <string.h>
#include <Servo.h>

/////////////CONSTANTS BÀRBARES/////////////////
#define trigPin 11
#define echoPin 12
#define TASKS 1 //Numero de tareas diferentes el Tracer que hay en el RT
#define BUFFER_SIZE 8192

#define TRACER_PRIO   10 //Max. en el RT
#define SENSOR_PRIO   3 
#define CALC_PRIO     2
#define ACTUATOR_PRIO 1 

/////////////INSANE GLOBAL VARIABLES HERE///////

double distance, SetPoint;

double errSum;

double kp, ki, kd;

double PID_output;

int tracerfile_fd;

Servo myservo;

static const char *RD_TEXT 		= "Task \"taskRdSensor\" is running\r\n";
static const char *CALCULATE_ERROR_TEXT = "Task \"taskCalculate\" is running\r\n";
static const char *ACTUATOR_TEXT	= "Task \"taskActuator\" is running\r\n"; 
static const char *TRACER_TEXT 		= "Task \"taskTracer\" is running\r\n";

/////////////AUX. FUNCTIONS/////////////////////

void SetTunings(double Kp, double Ki, double Kd){
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

/////////////RTOS STUFF NOW/////////////////////

void taskRdSensor(void * pvParameters){
  //Trickery per a fer marranades de blocks
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    //unsigned long t1 = millis();

    Serial.println("SENSOR");
    double duration;
    //Per assegurar-se de una bona lectura
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    //Lectura pulsein
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);

    //Conversions de distancia
    distance = (duration/2) / 2.91;
    //unsigned long t2 = millis();
    //Serial.print("timing: "); Serial.println((unsigned long)(t2 - t1));
    //Bloquejem fins X
    vTaskDelay( pdMS_TO_TICKS( 200 ) );
  }
}

void taskCalculate(void * pvParameters){
  
   //Trickery per a fer marranades de blocks
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
	unsigned long lastTime = xTaskGetTickCount();
	double lastErr = 0.0;
	for (;;) {
    //unsigned long t1 = millis();
    Serial.println("CALCULATE");
		static unsigned long now = xTaskGetTickCount();
		static double timeChange = (double)(now - lastTime);
		
		static double error = SetPoint - distance; //'distance' es una variable compartida		
		errSum += (error * timeChange);
		static double dErr = (error - lastErr) / timeChange;

		PID_output = kp * error + ki * errSum + kd * dErr;

		lastErr = error;
		lastTime = now;

    //Bloquejem fins X
    //unsigned long t2 = millis();
    //Serial.print("timing: "); Serial.println((unsigned long)(t2 - t1));
    vTaskDelay( pdMS_TO_TICKS( 200 ));
	}  
}

void taskActuator(void * pvParameters){
  //Trickery per a fer marranades de blocks
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  
	for (;;) {
    //unsigned long t1 = millis();

    Serial.println("ACTUATOR");
		static int degree = map(PID_output, -200, 200, 54, 114);
		myservo.write(degree);

    //unsigned long t2 = millis();
    //Serial.print("timing: "); Serial.println((unsigned long)(t2 - t1));
    //Bloquejem fins X
    vTaskDelay( pdMS_TO_TICKS( 200 ) );
	} 
}

void taskTracer(void * pvParameters) {

	/* This task is in charge of making a trace of the system        */
	/* every 2 ms, we can assume that is always hitting the deadline */
	/* because it's current priority is the maximum priority in the  */
	/* system, this is only for debugging, surely for a real system  */
	/* it would be deactivated.					 */

    //Trickery per a fer marranades de blocks
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    int array_size;
    TaskStatus_t* status_array;

	  int count = 0;
      
	  char buff [BUFFER_SIZE];

	  for (;;) {
  		array_size = uxTaskGetNumberOfTasks();
  		status_array = (TaskStatus_t *)pvPortMalloc(sizeof(TaskStatus_t)*array_size); //Dinamicamente pedimos memoria;
  
      array_size = uxTaskGetSystemState(status_array, array_size, NULL); //Puntero al vector de estructura, size que nos gustaria, no nos hace falta el tiempo de boot;    
       
  		int i;
  		sprintf(buff, "/***time: %d ms\n***\\", count*25); //tiempo en el que se tomó esta parte de la traza

  		for (i=0; i<array_size; i++) {        
  			sprintf(buff, "%s\t%d\n", status_array[i].pcTaskName, (int)status_array[i].uxCurrentPriority);
  		}
     
  		sprintf(buff, "\\******************/"); //seguramente quede un poco feo xdddddddd
  		//Hay que gestionar con un timer cuando pasan los 2 s para hacer un write del buffer en un archivo ...
  
      Serial.println(buff);
  		++count;
  		free(status_array);

      //Bloquejem fins X
      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 25 ) );
    }
      
}


/////////////CLASSIC ARDUINO STUFF NOW/////////////////////
void setup() {
  Serial.begin(9600);
  myservo.attach(9, 700, 2400);
  //Pinmodes pulsein
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //Set the initial tunings
  SetTunings(0.2, 0, 8); //As is never called from a thread, no mutexes inside this function are needed
  SetPoint = 255.0; //a cara de perro

  //Task to read the sensor
  xTaskCreate(taskRdSensor, "RD_SENSOR_TASK", 256, (void *)RD_TEXT, SENSOR_PRIO, NULL );

  //Task to compute the error
  xTaskCreate(taskCalculate, "COMPUTE_ERROR_TASK", 256, (void *)CALCULATE_ERROR_TEXT, CALC_PRIO, NULL);
  
  //Task to move the actuator
  xTaskCreate(taskActuator, "ACTUATOR_TASK", 256, (void *)ACTUATOR_TEXT, ACTUATOR_PRIO, NULL);
  
  //The tracer, which runs periodically
  xTaskCreate(taskTracer, "TRACER_TASK", 256, (void *)TRACER_TEXT, TRACER_PRIO, NULL );

  vTaskStartScheduler(); 
  
}

void loop() {
  // put your main code here, to run repeatedly:
}
