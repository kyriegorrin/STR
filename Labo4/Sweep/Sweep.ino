#include <Servo.h>

#define FASTASFUCK 160
#define NOTSOFAST 60
#define SLOWASFUCK 20

//dps = (60/(0.16+d))
//0.16*dps + d*dps = 60
//d = (60 - 0.16*dps)/dps
//pero no ens oblidem que el temps minim de comunicacio
//per al servomotor es de 20 ms

//int d = (int) 1000*((60-0.16*dps)/(dps*180)); 

//----------------------------RICARD VERSION QUE SE PASA LA VELOCIDAD POR LA PATILLA------------------------//
/*
 * El código es simple. Utilizamos los bucles del Sweep original y, para determinar la velocidad de rotación,
 * insertamos los DPS deseados. Con estos DPS debemos calcular cuál será el delay a utilizar dentro de cada
 * bucle. Para obtener un delay que sea adecuado para los DPS solicitados, debemos tener lo siguiente en cuenta:
 *    
 *    Los DPS indican cuántos grados debes recorrer en un segundo. Sabiendo que el método de funcionamiento se basa
 *    en un bucle que se desplaza grado a grado, solo necesitamos saber el tiempo que invertimos para desplazarnos un 
 *    grado. Entonces:
 *                    delay = (1/dps)*1000
 *    Obtenemos el tiempo para movernos un grado usando la inversa de los DPS y los multiplicamos por 1000 ya que los
 *    delays funcionan en milisegundos.
 *    
 *    Nótese que esta versión supone que el tiempo entre que se manda la instrucción write al servo y se empieza el delay
 *    es despreciable. También se asume que el servo funciona asíncronamente y el delay se ejecuta paralelamente. Llegados
 *    al caso donde los dps fuesen lo suficientemente altos para que el servo tardase más en llegar que el delay dado, el 
 *    funcionamiento no seria el mismo. Aún así, son casos muy extremos.
 */
//------------------------------------------------SEE YOU SPACE COWBOY---------------------------------------//


//----------------------------MARC VERSION-------------------------------------------------------------------//
/*
 *  Mi version no implementada pretender tomar como velocidad base la maxima que puede alcanzar el servomotor
 *  que viene a ser 375 dps (60/0.16). Entonces, si fueramos capaces de añadir un retraso a esta velocidad 
 *  estaríamos conisguiendo reducir la velocidad (ojo que aumentarla no nos será posible) y conseguir un espectro
 *  de velocidades entre 375 hasta 0 dps. Los cálculos correspondientes serían:
 * 
 *    dps = (60/(0.16+d)); Esta d representa el delay que añadimos, a d's más grandes valores más pequeños de dps conseguimos
 *    
 *    Aislar los terminos de esta ecuacion nos dará lugar al delay correspondiente para la velocidad deseada,
 *    por ejemplo para conseguir 30 dps haríamos lo siguiente:
 *    
 *    30 = (60/0.16+d) ---> 0.16*30 + d*30 = 60 ---> d = (60-0.16*30)/30 = 1.84 segundos
 *    
 *    Pero también sabemos que esto no es posible, deberíamos añadir el retraso a cada iteración del supuesto
 *    bucle que traza el recorrido del servomotor, por lo que deberíamos dividir este delay entre el número de iteraciones.
 */
//-----------------------------------------------------------------------------------------------------------//

Servo myservo;  // create servo object to control a servo

//Funció que fa rotació a tants Degree Per Second com indiquem 
void spin2Win(int dps){
 
  int retraso = (int) 1000.0/dps; 
  int pos = 0;    // variable to store the servo position

  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(retraso);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(retraso);                       // waits 15ms for the servo to reach the position
  }
}

void setup() {
  Serial.begin(9600);
  myservo.attach(9, 700, 2400);  // Custom min and max
}

void loop() {
  spin2Win(FASTASFUCK);
  spin2Win(NOTSOFAST);
  spin2Win(SLOWASFUCK);
}

