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

//Asignacion de pines del 74HC595 que controla la funcion general de escritura
int ss0 = 2; //Slave select fila 0
int ss1 = 3; //Slave select fila 1
int ss2 = 4; //Slave select fila 2
int ss3 = 5; //Slave select fila 3

/*
******************************************** Variables globales ********************************************
*/

const int pixels = 10; // Cantidad de pixels por fila
const int bits = 16;

#define t 1000 

// Almacenamiento de los estados de los chips SPI de sensores-entradas

int dataMock [pixels] = {   0b1100000111,
                            0b0010111000,
                            0b0111000010,
                            0b0111100110,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000 };

int dataInput [pixels] = {  0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000 };

int dataOutput [pixels] = { 0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000,
                            0b0000000000 };


byte input_A;
int delayTime = 500;
String bitValue;

/*
******************************************** Funciones de escritura ********************************************
*/

// Se escribe el valor a la n-fila de LEDs, recibe como parametro el valor del slave select -SPI de la fila y la posicion en el arreglo dataMock a imprimir
void writeMock(int ss, int val)
{
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    digitalWrite(ss, LOW);            //Write our Slave select low to enable the SHift register to begin listening for data
    SPI.transfer16(dataMock[val]);
    digitalWrite(ss, HIGH);          //Once the transfer is complete, set the latch back to high to stop the shift register listening for data
    SPI.endTransaction();
}

// Se escribe el valor a la n-fila de LEDs, recibe como parametro el valor del slave select -SPI de la fila y la posicion en el arreglo dataInput a imprimir
void writeData(int ss, int val)
{
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    digitalWrite(ss, LOW);            //Write our Slave select low to enable the SHift register to begin listening for data
    SPI.transfer16(dataInput[val]);
    digitalWrite(ss, HIGH);          //Once the transfer is complete, set the latch back to high to stop the shift register listening for data
    SPI.endTransaction();
}

// Se escribe el valor Negado a la n-fila de LEDs, recibe como parametro el valor del slave select -SPI de la fila y la posicion en el arreglo dataInput a imprimir
void writeData_Comp(int ss, int val)
{
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    digitalWrite(ss, LOW);            //Write our Slave select low to enable the SHift register to begin listening for data
    SPI.transfer16(~dataInput[val]);
    digitalWrite(ss, HIGH);          //Once the transfer is complete, set the latch back to high to stop the shift register listening for data
    SPI.endTransaction();
}
/*
******************************************** Funciones de lectura ********************************************
*/

// Lee y almacena el estado de cada valor entregado por los chips SPI a cada una las filas en el arreglo inputState
void readStateSPI()
{

  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  SPI.setBitOrder(MSBFIRST);
  // Inicializacion del estado de acuerdo con los pulsos requeridos por el chip
  digitalWrite (_74HC165_01, LOW); 
  digitalWrite (_74HC165_01, HIGH);
  
  
  for(int i = 0; i < pixels; i++)
  {  
    // Se lee el estado del bus SPI y se almacena en cada posicion del arreglo
    dataInput[i] = SPI.transfer16(16);

  }

  Serial.println(" ");
  SPI.endTransaction();

}

void printState()
{
  // Imprime el valor decimal de la fila - Se puede usar para la etapa analitica de patrones
  Serial.println((String)"SPI valor :" + dataInput[0]);  // Se monitorea el estado de asignacion de cada fila
  //Serial.println((String)"Bus SPI buffer " + i + " valor hexadecimal :" + (inputState[i], HEX) );  // Se monitorea el estado de asignacion de cada fila
}

// Lee bit por bit el estado de cada pixel por filas y lo almacena en el arreglo state
void readBitState()
{

  bool state [pixels] = { 0,0,0,0,0,0,0,0,0,0 };

    for(int i = 0; i < pixels; i++)
    {

      Serial.print((String)"Estado bit x bit fila " + i + " : ");
      
        for(int j = 0; j < pixels; j++)
        {
            state[i] = bitRead(dataInput[i],j);
            Serial.print((String)state[i]);
        }
      Serial.print(" ");
      Serial.print(dataInput[i]);
      Serial.println(" ");

    }
    
}



/*
******************************************** Funciones ejecucion de Arduino ********************************************
*/

//funcion de inicializacion general Arduino - Hardware y software

void setup ()
{ 
  Serial.begin (115200);  // Inicializacion del monitor serial
  Serial.println ("Begin switch test.");
  
  //Inicializacion parametros modo lectura
  SPI.begin (); // Inicializacion Protocolo SPI
  SPI.setBitOrder(MSBFIRST); 

  //inicializacion pines chip lectura 
  pinMode (_74HC165_01, OUTPUT);
  digitalWrite (_74HC165_01, HIGH);
  
  //Inicializacion parametros hardware escritura
  pinMode(ss0, OUTPUT);
  pinMode(ss1, OUTPUT);
  pinMode(ss2, OUTPUT);    
  pinMode(ss3, OUTPUT);

}
 
//funcion ciclica Arduino 
void loop ()
{

  readStateSPI();
  readBitState();

  writeData_Comp(ss0, 0);
  writeMock(ss1, 1);
  writeMock(ss2, 2);
  writeMock(ss3, 3);

} 