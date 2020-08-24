#include <SPI.h>

/*
******************************************** Conexiones hardware ********************************************
*/

int data0 = 0b0000001111111111;
int data1 = 0b0000001111111110; 
int data2 = 0b0000001111111100;
int data3 = 0b0000001111111000;
int data4 = 0b0000001111110000;
int data5 = 0b0000001111100000;
int data6 = 0b0000001111000000;
int data7 = 0b0000001110000000;
int data8 = 0b0000001100000000;
int data9 = 0b0000001000000000;

const int size = 11;
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


int inputState[size] = {  0b0000000000000000,
                          0b0000000000000000, 
                          0b0000000000000000,
                          0b0000000000000000,
                          0b0000000000000000,
                          0b0000000000000000,
                          0b0000000000000000,
                          0b0000000000000000,
                          0b0000000000000000,
                          0b0000000000000000,
                          0b0000000000000000 };



int slaveSelectA = 7;
int slaveSelectB = 6;
int slaveSelectC = 5;
int slaveSelectD = 4;

int delayTime = 50;

/*
******************************************** Funciones revisadas ********************************************
*/


void dataA()
{
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    digitalWrite(slaveSelectA, LOW);            //Write our Slave select low to enable the SHift register to begin listening for data
    SPI.transfer16(inputState[0]);
    digitalWrite(slaveSelectA, HIGH);          //Once the transfer is complete, set the latch back to high to stop the shift register listening for data
    delay(delayTime);
    SPI.endTransaction();
}

void dataB()
{
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
    digitalWrite(slaveSelectB, LOW);            //Write our Slave select low to enable the SHift register to begin listening for data
    SPI.transfer16(inputState[1]);
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
void setup() {
  Serial.begin(115200);
  pinMode(slaveSelectA, OUTPUT);
  pinMode(slaveSelectB, OUTPUT);
  pinMode(slaveSelectC, OUTPUT);    
  pinMode(slaveSelectD, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);   
}

void loop() {

  dataScanInv(slaveSelectA);
  dataScan(slaveSelectB);
  dataScanInv(slaveSelectB);  
  dataScan(slaveSelectC);
  dataScanInv(slaveSelectC);
  dataScan(slaveSelectD);
  dataScanInv(slaveSelectD); 

}
