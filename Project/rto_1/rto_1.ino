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
#define TASKS 4
#define BUFFER_SIZE 2000

#define TRACER_PRIO   4 //Max. en el RT
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

static const char *RD_TEXT 		          = "Sensor";
static const char *CALCULATE_ERROR_TEXT = "Calculate";
static const char *ACTUATOR_TEXT	      = "Actuator"; 
static const char *TRACER_TEXT 		      = "Tracer";

char buff [BUFFER_SIZE];

typedef enum {
  IDLE_S,   //0
  RUN_S,    //1
  BLOCKED_S //2
} state_t;

typedef struct {

  char* taskName;

  state_t state;

  UBaseType_t uxCurrentPriority;
  
} taskstruct_t; //Linux like, happy Juan José Costa!

taskstruct_t status_array [sizeof(taskstruct_t)*TASKS];

/////////////AUX. FUNCTIONS/////////////////////

void SetTunings(double Kp, double Ki, double Kd){
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

void InitializePCBs() {
  
  status_array[0].taskName = RD_TEXT;
  status_array[1].taskName = CALCULATE_ERROR_TEXT;
  status_array[2].taskName = ACTUATOR_TEXT;
  status_array[3].taskName = TRACER_TEXT;

  status_array[0].uxCurrentPriority = SENSOR_PRIO;
  status_array[1].uxCurrentPriority = CALC_PRIO;
  status_array[2].uxCurrentPriority = ACTUATOR_PRIO;
  status_array[3].uxCurrentPriority = TRACER_PRIO;
  
  for (int i = 0; i < TASKS; ++i) {
    status_array[i].state = IDLE_S;  
  }  
  
} //Todas las tareas en IDLE

void SetState(taskstruct_t * pcb, const char* taskName, state_t state, UBaseType_t uxCurrentPriority) {
  
  pcb->taskName = taskName;
  pcb->state    = state;
  pcb->uxCurrentPriority = uxCurrentPriority;  
  
}

/////////////RTOS STUFF NOW/////////////////////

void taskRdSensor(void * pvParameters){
  
  for (;;) {

    SetState(&status_array[0], RD_TEXT, state_t::RUN_S, SENSOR_PRIO);

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

    //Bloquejem fins X
    SetState(&status_array[0], RD_TEXT, state_t::BLOCKED_S, SENSOR_PRIO);
    vTaskDelay( pdMS_TO_TICKS( 200 ) );
    
  }
  
}

void taskCalculate(void * pvParameters){
  
	unsigned long lastTime = xTaskGetTickCount();
	double lastErr = 0.0;
 
	for (;;) {

    SetState(&status_array[1], CALCULATE_ERROR_TEXT, state_t::RUN_S, CALC_PRIO);

		static unsigned long now = xTaskGetTickCount();
		static double timeChange = (double)(now - lastTime);
		
		static double error = SetPoint - distance; //'distance' es una variable compartida		
		errSum += (error * timeChange);
		static double dErr = (error - lastErr) / timeChange;

		PID_output = kp * error + ki * errSum + kd * dErr;

		lastErr = error;
		lastTime = now;

    SetState(&status_array[1], CALCULATE_ERROR_TEXT, state_t::BLOCKED_S, CALC_PRIO);

    //Bloquejem fins X
    vTaskDelay( pdMS_TO_TICKS( 200 ));
    
	}  
}

void taskActuator(void * pvParameters){
  /*//Trickery per a fer marranades de blocks
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();*/
  
	for (;;) {

    SetState(&status_array[2], ACTUATOR_TEXT, state_t::RUN_S, ACTUATOR_PRIO);

		static int degree = map(PID_output, -200, 200, 54, 114);
		myservo.write(degree);

    SetState(&status_array[2], ACTUATOR_TEXT, state_t::BLOCKED_S, ACTUATOR_PRIO);

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

  int count = 0;

  for (;;) {

    SetState(&status_array[3], TRACER_TEXT, state_t::RUN_S, TRACER_PRIO);

		int i; 
    char tmp[75];
		sprintf(buff, "/*** time: %d ms ***\\\n", count*25); //tiempo en el que se tomó esta parte de la traza
    sprintf(tmp, "NAME\tSTATE\tCPRIORITY\n");
    strcat(buff, tmp);
    
		for (i=0; i<TASKS; i++) {        
			sprintf(tmp, "%s\t%u\t%d\n", status_array[i].taskName, status_array[i].state, (int)status_array[i].uxCurrentPriority);
		  strcat(buff, tmp);
		}
   
		sprintf(tmp, "\\******************/\n"); //seguramente quede un poco feo xdddddddd
		strcat(buff, tmp);
		//Hay que gestionar con un timer cuando pasan los 2 s para hacer un write del buffer en un archivo ...

    Serial.println(buff);
		
		++count;

    SetState(&status_array[3], TRACER_TEXT, state_t::BLOCKED_S, TRACER_PRIO); //Esto no tiene sentido, pero es para seguir el patron

    //Bloquejem fins X
    vTaskDelay( pdMS_TO_TICKS( 25 ) ); //como es la maxima prioridad siempre entrara cada 25 ms!
  }
      
}


/////////////KLASSIC ARDUINO STUFF NOW/////////////////////
void setup() {
  Serial.begin(112500);
  myservo.attach(9, 700, 2400);
  //Pinmodes pulsein
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  InitializePCBs();

  //Set the initial tunings
  SetTunings(0.2, 0, 8); //As is never called from a thread, no mutexes inside this function are needed
  SetPoint = 255.0; //a cara de perro

  //Task to read the sensor
  xTaskCreate(taskRdSensor, "RD_SENSOR_TASK", 256, (void *)RD_TEXT, SENSOR_PRIO, NULL );

  //Task to compute the error
  xTaskCreate(taskCalculate, "COMPUTE_ERROR_TASK", 1024, (void *)CALCULATE_ERROR_TEXT, CALC_PRIO, NULL);
  
  //Task to move the actuator
  xTaskCreate(taskActuator, "ACTUATOR_TASK", 256, (void *)ACTUATOR_TEXT, ACTUATOR_PRIO, NULL);
  
  //The tracer, which runs periodically
  xTaskCreate(taskTracer, "TRACER_TASK", 1024, (void *)TRACER_TEXT, TRACER_PRIO, NULL );

  vTaskStartScheduler(); 
  
}

void loop() {
  //El loop es por defecto la tarea Idle
  //Serial.println("Idle task");  
}
