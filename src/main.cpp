
/* version 4.x del tablero de LEDS 8x8 que incorpora:
* 8 circuitos de desplazamiento 74HC595 para escritura
* 8 registros de desplazamiento 74LS165 para lectura
* Utilizacion del protocolo SPI 
* @autor: Jose Antonio Gonzales jzgonzales@gmail.com
* @date: julio 2020
*/

/*
************************************************** Librerias ************************************************
*/
#include <Arduino.h>
#include <SPI.h>

/*
******************************************** Conexiones hardware ********************************************
*/
//Asignacion de pines del 74LS165 que controla la funcion general de lectura - DATA INPUT
const byte _74HC165_01 = 7;  // LATCH-- Entrada carga en paralelo. Conecta el pin 1 del 74LS165 al pin 7 de Arduino

/* Con la utilizacion del protocolo SPI las conexiones de a los pines 50 y 52 del Ardunio Mega 2560 no se definen ya que estan predefinidas por el hardware. 
* SCK SPI Bus. Conecta el pin 2 del 74LS165 al pin 52 de Arduino Mega 2560 Protocolo SPI
* MISO SPI Bus. Conecta el pin 7 del 74LS165 al pin 50 de Arduino Mega 2560 Protocolo SPI
* La conexion en serie de multiples 74LS165 se hace del pin 10 del primer Chip al pin 9 del siguiente Chip sucesivamente.
*/


/*
******************************************** Variables globales ********************************************
*/

const int pixels = 10; // Cantidad de pixels por fila
const int size = 16;

#define t 1000 

// Almacenamiento de los estados de los chips SPI de sensores-entradas

int inputState [pixels] = { 0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000 };

int output [pixels] =    {  0b0000000000000000,
                            0b0000000000000000,
                            0b0000000000000000,
                            0b0000000000000000,
                            0b0000000000000000,
                            0b0000000000000000,
                            0b0000000000000000,
                            0b0000000000000000,
                            0b0000000000000000,
                            0b0000000000000000 };


int dataArr[size] = { 0b0000001111111111,
                      0b0000001111111110, 
                      0b0000001111111100,
                      0b0000001111111000,
                      0b0000001111110000,
                      0b0000001111100000,
                      0b0000001111000000,
                      0b0000001110000000,
                      0b0000001100000000,
                      0b0000001000000000,
                      0b0000000000000000 };

const int sizeA = 11;

byte input_A;
String bitValue;

int slaveSelectA = 7;
int slaveSelectB = 6;
int slaveSelectC = 5;
int slaveSelectD = 4;

int delayTime = 50;

/*
******************************************** Funciones Escritura ********************************************
*/

void dataA()
{
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    digitalWrite(slaveSelectA, LOW);            //Write our Slave select low to enable the SHift register to begin listening for data
    SPI.transfer16(dataArr[0]);
    digitalWrite(slaveSelectA, HIGH);          //Once the transfer is complete, set the latch back to high to stop the shift register listening for data
    delay(delayTime);
    SPI.endTransaction();
}

void dataB()
{
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    digitalWrite(slaveSelectB, LOW);            //Write our Slave select low to enable the SHift register to begin listening for data
    SPI.transfer16(dataArr[1]);
    digitalWrite(slaveSelectB, HIGH);          //Once the transfer is complete, set the latch back to high to stop the shift register listening for data
    delay(delayTime);
    SPI.endTransaction();
}

void dataC()
{
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    digitalWrite(slaveSelectC, LOW);            //Write our Slave select low to enable the SHift register to begin listening for data
    SPI.transfer16(dataArr[1]);
    digitalWrite(slaveSelectC, HIGH);          //Once the transfer is complete, set the latch back to high to stop the shift register listening for data
    delay(delayTime);
    SPI.endTransaction();
}

void dataD()
{
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    digitalWrite(slaveSelectD, LOW);            //Write our Slave select low to enable the SHift register to begin listening for data
    SPI.transfer16(dataArr[4]);
    digitalWrite(slaveSelectD, HIGH);          //Once the transfer is complete, set the latch back to high to stop the shift register listening for data
    delay(delayTime);
    SPI.endTransaction();
}



void dataScan(int slave)
{
  for(int i = 0; i<11; i++)
  {
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    digitalWrite(slave, LOW);            //Write our Slave select low to enable the SHift register to begin listening for data
    SPI.transfer16(dataArr[i]);
    digitalWrite(slave, HIGH);          //Once the transfer is complete, set the latch back to high to stop the shift register listening for data
    delay(delayTime);
    SPI.endTransaction();
  }

}

void dataScanInv(int slave)
{
  for(int i = 0; i<11; i++)
  {
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    digitalWrite(slave, LOW);            //Write our Slave select low to enable the SHift register to begin listening for data
    SPI.transfer16(~dataArr[i]);
    digitalWrite(slave, HIGH);          //Once the transfer is complete, set the latch back to high to stop the shift register listening for data
    delay(delayTime);
    SPI.endTransaction();
  }

}


/*
******************************************** Funciones Lectura ********************************************
*/

// Lee y almacena el estado de cada valor entregado por los chips SPI a cada una las filas en el arreglo inputState
void readStateSPI(){

  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  // Inicializacion del estado de acuerdo con los pulsos requeridos por el chip
  digitalWrite (_74HC165_01, LOW); 
  digitalWrite (_74HC165_01, HIGH);

  SPI.setBitOrder(MSBFIRST);
  for(int i = 0; i < pixels; i++){
    
    // Se lee el estado del bus SPI y se almacena en cada posicion del arreglo
    inputState[i] = SPI.transfer16(16);

  }   
  Serial.println(" ");
  SPI.endTransaction();
}

void printState()
{
      // Imprime el valor decimal de la fila - Se puede usar para la etapa analitica de patrones
    Serial.println((String)"SPI valor :" + inputState[0]);  // Se monitorea el estado de asignacion de cada fila
    //Serial.println((String)"Bus SPI buffer " + i + " valor hexadecimal :" + (inputState[i], HEX) );  // Se monitorea el estado de asignacion de cada fila
}

// Lee bit por bit el estado de cada pixel por filas y lo almacena en el arreglo state
void readBitState(){

bool state [16] = { 0,0,0,0,0,0,0,0,0,0 };

    for(int i = 0; i < pixels; i++){

      Serial.print((String)"Estado bit x bit fila " + i + " : ");
      
        for(int j = 0; j < pixels; j++){

            state[i] = bitRead(inputState[i],j);
            Serial.print((String)state[i]);
        }
      Serial.print(" ");
      Serial.print(inputState[i]);
      Serial.println(" ");

    }
    
}


/*
******************************************** Funciones ejecucion de Arduino ********************************************
*/

//funcion de inicializacion general Arduino - Hardware y software

void setup ()
{

  //inicializacion Variables lectura
  SPI.begin ();

  Serial.begin (115200);
  Serial.println ("Begin switch test.");

  //inicializacion pines chip lectura 
  pinMode (_74HC165_01, OUTPUT);
  digitalWrite (_74HC165_01, HIGH);

  //Inicializacion Variables escritura
  pinMode(slaveSelectA, OUTPUT);
  pinMode(slaveSelectB, OUTPUT);
  pinMode(slaveSelectC, OUTPUT);    
  pinMode(slaveSelectD, OUTPUT);

  SPI.setBitOrder(MSBFIRST); 
}
 
//funcion ciclica Arduino 
void loop ()
{
    /*
  * Lectura LEDS
  */
  // Llamado a la funcion de lecto-escritura
  //readAndWrite();
  readStateSPI();
  readBitState();
  //printState();

  /*
  * Escritura LEDS
  */
  dataScanInv(slaveSelectA);
  dataScan(slaveSelectB);
  dataScanInv(slaveSelectB);  
  dataScan(slaveSelectC);
  dataScanInv(slaveSelectC);
  dataScan(slaveSelectD);
  dataScanInv(slaveSelectD); 


} 