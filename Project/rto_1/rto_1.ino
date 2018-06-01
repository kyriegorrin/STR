#include <Arduino_FreeRTOS.h>

/////////////CONSTANTS BÀRBARES/////////////////
#define trigPin 11
#define echoPin 12
#define TASKS 1 //Numero de tareas diferentes el Tracer que hay en el RT

/////////////INSANE GLOBAL VARIABLES HERE///////
double distance;

/*char state [TASKS]; //Aqui almacenan su estado

typedef enum state_t {
  READY,
  RUNNING,
  BLOCKED
};*/

static const char *RD_TEXT = "Task \"taskRdSensor\" is running\r\n";
static const char *TRACER_TEXT = "Task \"taskTracer\" is running\r\n";


/////////////RTOS STUFF NOW/////////////////////
void taskRdSensor(void * pvParameters){
  
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
  
}

void taskCalculate(void * pvParameters){
  
}

void taskActuator(void * pvParameters){
  
}

void taskTracer(void * pvParameters) {
  
      Serial.begin(9600);

      UBaseType_t arraySize = uxCurrentNumberOfTasks();
      TaskStatus_t * systemState = pvPortMalloc(arraySize * sizeof(TaskStatus_t));

      for (;;) {
      
        if (systemState != NULL) {
          
          n = uxTaskGetSystemState(systemState, n, NULL);
  
          unsigned int i;
          for (i = 0; i < n; ++i) {
               Serial.println("Task name: " + systemState [i].pcTaskName);
               Serial.println("Task cpriority: " + systemState [i].uxCurrentPriority);
          }
          
        }
        else
          Serial.println("No tasks are available to schedule");

      }

      vPortFree(systemState);

      Serial.end();

}


/////////////CLASSIC ARDUINO STUFF NOW/////////////////////
void setup() {
  
  //Pinmodes pulsein
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //Task to read the sensor
  xTaskCreate(taskRdSensor, "RD_SENSOR_TASK", 256, (void *)RD_TEXT, 1, NULL );

  //The tracer, which runs periodically
  xTaskCreate(taskTracerSensor, "TRACER_TASK", 256, (void *)TRACER_TEXT, 1, NULL );

  vTaskStartScheduler(); 
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
