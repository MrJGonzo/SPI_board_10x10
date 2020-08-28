/*
************************************************** Librerias ************************************************
*/
#include <Arduino.h>
#include <SPI.h>

/*
******************************************** Conexiones hardware ********************************************
*/
//Asignacion de pines del 74LS165 que controla la funcion general de lectura - DATA INPUT
const byte R0_74HC165_01 = 7;  // LATCH-- Entrada carga en paralelo. Conecta el pin 1 del 74LS165 al pin 7 de Arduino

/* Con la utilizacion del protocolo SPI las conexiones de a los pines 50 y 52 del Ardunio Mega 2560 no se definen ya que estan predefinidas por el hardware. 
* SCK SPI Bus. Conecta el pin 2 del 74LS165 al pin 52 de Arduino Mega 2560 Protocolo SPI
* MISO SPI Bus. Conecta el pin 7 del 74LS165 al pin 50 de Arduino Mega 2560 Protocolo SPI
* La conexion en serie de multiples 74LS165 se hace del pin 10 del primer Chip al pin 9 del siguiente Chip sucesivamente.
*/

//Asignacion de pines para control del pulso SS-Slave select del protocolo SPI para los chips 74HC595 que controlan la funcion general de escritura
int ss0 = 2; //Slave select fila 0
int ss1 = 3; //Slave select fila 1
int ss2 = 4; //Slave select fila 2
int ss3 = 5; //Slave select fila 3

/*
******************************************** Variables globales ********************************************
*/

const int rowPixels = 10; // Cantidad de pixels por fila
const int bits = 16;

#define t 1000 

// Almacenamiento de los estados de los chips SPI de sensores-entradas

int dataMock [rowPixels]  = {   0b1100000111,
                                0b0010111000,
                                0b0111000010,
                                0b0111100110,
                                0b1110011111,
                                0b1001111100,
                                0b0111010100,
                                0b1111111000,
                                0b0001111111,
                                0b1111000111 };

int dataInput [rowPixels] = {   0b0000000000,
                                0b0000000000,
                                0b0000000000,
                                0b0000000000,
                                0b0000000000,
                                0b0000000000,
                                0b0000000000,
                                0b0000000000,
                                0b0000000000,
                                0b0000000000 };

int dataOutput [rowPixels] = {  0b0000000000,
                                0b0000000000,
                                0b0000000000,
                                0b0000000000,
                                0b0000000000,
                                0b0000000000,
                                0b0000000000,
                                0b0000000000,
                                0b0000000000,
                                0b0000000000 };



/*
******************************************** Funciones de escritura ********************************************
*/

// Se escribe el valor a la n-fila de LEDs, recibe como parametros: ss = valor asociado al puerto slave select -SPI de la fila; val= posicion en el arreglo dataMock a escribir.
void writeLedRow_mock(int ss, int val)
{
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    digitalWrite(ss, LOW);            //Write our Slave select low to enable the SHift register to begin listening for data
    SPI.transfer16(dataMock[val]);
    digitalWrite(ss, HIGH);          //Once the transfer is complete, set the latch back to high to stop the shift register listening for data
    SPI.endTransaction();
}

// Se escribe el valor a la n-fila de LEDs con datos de las entradas/sensores. Recibe como parametros: ss = valor asociado al puerto slave select -SPI de la fila; val= posicion en el arreglo dataMock a escribir.
void writeLedRow_data(int ss, int val)
{
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    digitalWrite(ss, LOW);            //Write our Slave select low to enable the SHift register to begin listening for data
    SPI.transfer16(dataInput[val]);
    digitalWrite(ss, HIGH);          //Once the transfer is complete, set the latch back to high to stop the shift register listening for data
    SPI.endTransaction();
}

// Se escribe el valor Negado a la n-fila de LEDs, recibe como parametros: ss = valor asociado al puerto slave select -SPI de la fila; val= posicion en el arreglo dataMock a escribir.
void writeLedRow_n(int ss, int val)
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
void readStateSPI(int data[], byte pin, int s  )
{

  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  SPI.setBitOrder(MSBFIRST);
  // Inicializacion del estado de acuerdo con los pulsos requeridos por el chip
  digitalWrite (pin, LOW); 
  digitalWrite (pin, HIGH);
  
  for(int i = 0; i < s; i++)
  {  
    data[i] = SPI.transfer16(16);  // Se lee el estado del bus SPI y se almacena en cada posicion del arreglo
  }

  Serial.println(" ");
  SPI.endTransaction();

}

// Imprime el valor decimal de la fila - Se puede usar para la etapa analitica de patrones
void printState()
{
  for(int i = 0; i < rowPixels; i++)
  {
    Serial.println((String)"SPI valor :" + dataInput[0]);  // Se monitorea el estado de asignacion de cada fila
    Serial.println((String)"Bus SPI buffer " + i + " valor hexadecimal :" + (dataInput[i], HEX) );  // Se monitorea el estado de asignacion de cada fila
  }

}

// Lee bit por bit el estado de cada pixel por filas y lo almacena en el arreglo state. Recibe parametros: in:= arreglo a leer. s:= tamano de la fila.
void readBitState(int in[], int s)
{

  bool tmpState [rowPixels] = { 0,0,0,0,0,0,0,0,0,0 };

    for(int i = 0; i < s; i++)
    {
      Serial.print((String)"Estado bit x bit fila " + i + " : ");
      
        for(int j = 0; j < s; j++)
        {
            tmpState[i] = bitRead(in[i],j);
            Serial.print((String)tmpState[i]);
        }
      Serial.print(" ");
      Serial.print(in[i]);
      Serial.println(" ");
    }
    
}



/*
******************************************** Funciones ejecucion de Arduino ********************************************
*/

//funcion de inicializacion general Arduino - Hardware y software

void setup ()
{ 
  //inicializacion del puerto serial
  Serial.begin (115200);  // Inicializacion del monitor serial
  Serial.println ("Begin switch test.");
  
  //Inicializacion BUS SPI modo lectura
  SPI.begin (); 
  SPI.setBitOrder(MSBFIRST); 

  //Inicializacion pines chip lectura 
  pinMode (R0_74HC165_01, OUTPUT);
  digitalWrite (R0_74HC165_01, HIGH);
  
  //Asignacion puertos SS-Slave select, BUS SPI hardware escritura
  pinMode(ss0, OUTPUT);
  pinMode(ss1, OUTPUT);
  pinMode(ss2, OUTPUT);    
  pinMode(ss3, OUTPUT);

}
 
//funcion ciclica Arduino 
void loop ()
{

  readStateSPI(dataInput, R0_74HC165_01, rowPixels);
  readBitState(dataInput, rowPixels);

  writeLedRow_data(ss0, 0);
  writeLedRow_mock(ss1, 1);
  writeLedRow_mock(ss2, 2);
  writeLedRow_mock(ss3, 3);
  
}