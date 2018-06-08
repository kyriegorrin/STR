#include <Arduino_FreeRTOS.h>

#include <FreeRTOSConfig.h>
#include <portmacro.h>
#include <FreeRTOSVariant.h>
#include <croutine.h>
#include <task.h>

/////////////CONSTANTS BÀRBARES/////////////////
#define trigPin 11
#define echoPin 12
#define TASKS 1 //Numero de tareas diferentes el Tracer que hay en el RT
#define BUFFER_SIZE 1024

/////////////INSANE GLOBAL VARIABLES HERE///////
double distance;

/*char state [TASKS]; //Aqui almacenan su estado

typedef enum state_t {
  READY,
  RUNNING,
  BLOCKED
};*/

int tracerfile_fd;

static const char *RD_TEXT = "Task \"taskRdSensor\" is running\r\n";
static const char *TRACER_TEXT = "Task \"taskTracer\" is running\r\n";


/////////////RTOS STUFF NOW/////////////////////
void taskRdSensor(void * pvParameters){
  for (;;) {
    //Serial.println("Midiendo vergas");
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
    delay(3);
  }
}

void taskCalculate(void * pvParameters){
  
}

void taskActuator(void * pvParameters){
  
}

void taskTracer(void * pvParameters) {
  
      for (;;) {
        Serial.println("eeeeee");
        static char buff [BUFFER_SIZE];
        
        vTaskList(buff); 

        Serial.println(buff);
      }
      
}


/////////////CLASSIC ARDUINO STUFF NOW/////////////////////
void setup() {
  Serial.begin(9600);
  //Pinmodes pulsein
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //Task to read the sensor
  xTaskCreate(taskRdSensor, "RD_SENSOR_TASK", 256, (void *)RD_TEXT, 1, NULL );

  //The tracer, which runs periodically
  xTaskCreate(taskTracer, "TRACER_TASK", 256, (void *)TRACER_TEXT, 1, NULL );

  vTaskStartScheduler(); 
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
