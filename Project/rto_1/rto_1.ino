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
#define BUFFER_SIZE 8192

/////////////INSANE GLOBAL VARIABLES HERE///////
double distance;

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
  
	  /* Esta tarea es encargada de hacer la traza del sistema   	*/
	  /* durante los 2s de ejecucion, podemos asumir que siempre 	*/
	  /* cumplira el deadline dado que tendra la mayor prioridad 	*/
      /* posible, seguramente en un sistema 100% real, se quitaria.	*/

      int array_size;
      TaskStatus_t* status_array;

	  int count = 0;
      
	  char buff [BUFFER_SIZE];

	  for (;;) {
		array_size = uxCurrentNumberOfTask();
		status_array = pvPortMalloc(sizeof(TaskStatus)*array_size); //Dinamicamente pedimos memoria;

    	array_size = uxTaskGetSystemState(status_array, array_size, NULL); //Puntero al vector de estructura, size que nos gustaria, no nos hace falta el tiempo de boot;    
     
		int i;
		sprintf(&buff, "/***time: %d ms\n***\\", count*25); //tiempo en el que se tomó esta parte de la traza
		for (i=0; i<array_size; i++)
			sprintf(&buff, "%s\t%d\n", array_size[i].pcTaskName, array_size[i].uxCurrentPriority);
		
		sprintf(&buff, "\\******************/"); //seguramente quede un poco feo xdddddddd
		//Hay que gestionar con un timer cuando pasan los 2 s para hacer un write del buffer en un archivo ...

        Serial.println(buff);
		++count;
		free(status_array);
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
