//Read data off external memory
#include <Wire.h>

#define BMP085_ADDRESS 0x77  // I2C address of BMP085
#define disk1 0x50    //I2C Address of 24LC256 eeprom chip

const int groundpin = 3;             // analog input pin 4 -- ground
const int powerpin = 2;              // analog input pin 5 -- voltage
const int xpin = A1;                  // x-axis of the accelerometer
const int ypin = A2;                  // y-axis
const int zpin = A3;                  // z-axis (only on 3-axis models)
const unsigned char OSS = 0;  // Oversampling Setting

// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

int currentAddress;
short temperature;
long pressure; //Unsigned may work better, change back otherwise.
int xAccel;
int yAccel;
int zAccel;

// Use these for altitude conversions
const float p0 = 101325;     // Pressure at sea level (Pa)
long altitude;
long altOut;

void setup()
{
  currentAddress = 0;
  Serial.begin(9600);
  Wire.begin();
  delay(5000);//Wait long enough to open serial viewer
}

void loop(){
  readData(); 
  delay(10); 
}

void readData(){
 byte byteArray[16];
 if (currentAddress < 32000){
   for (int i=0; i<16; i++){
    byteArray[i] = readEEPROM(disk1, currentAddress);
    currentAddress++;
   }
   pressure = 0; //Clear old pressure value.
   pressure+= byteArray[3];
   pressure = pressure << 8;
   pressure+= byteArray[2];
   pressure = pressure << 8;
   pressure+= byteArray[1];
   pressure = pressure << 8;
   pressure+= byteArray[0];
   
   xAccel = 0; //Clear old x-value;
   xAccel+= byteArray[5];
   xAccel = xAccel << 8;
   xAccel+= byteArray[4];
  
   yAccel = 0; //Clear old y-value;
   yAccel+= byteArray[7];
   yAccel = yAccel << 8;
   yAccel+= byteArray[6];
  
   zAccel = 0; //Clear old z-value;
   zAccel+= byteArray[9];
   zAccel = zAccel << 8;
   zAccel+= byteArray[8];

   temperature = 0; //Clear old value.
   temperature+= byteArray[11];
   temperature = temperature << 8;
   temperature+= byteArray[10];

   altitude = 0; //Clear old pressure value.
   altitude+= byteArray[15];
   altitude = altitude << 8;
   altitude+= byteArray[14];
   altitude = altitude << 8;
   altitude+= byteArray[13];
   altitude = altitude << 8;
   altitude+= byteArray[12];
   
   //Print out data read from memory.
   Serial.print(temperature,DEC);
   Serial.print(",");
   Serial.print(pressure,DEC);
   Serial.print(",");
   Serial.print(altitude,DEC);
   Serial.print(",");
   Serial.print(xAccel,DEC);
   Serial.print(",");
   Serial.print(yAccel,DEC);
   Serial.print(",");
   Serial.print(zAccel,DEC);
   Serial.println();
 }
}

void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data ) 
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
 
  delay(5);
}
 
byte readEEPROM(int deviceaddress, unsigned int eeaddress ) 
{
  byte rdata = 0xFF;
 
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
 
  Wire.requestFrom(deviceaddress,1);
 
  if (Wire.available()) rdata = Wire.read();
 
  return rdata;
}
