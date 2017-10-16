// test ST accelerometer with OpenTag
// I2C interface

#include <SPI.h>
#include <SdFat.h>
#include <Wire.h>

//#define SDA_PORT PORTC
//#define SDA_PIN 4
//#define SCL_PORT PORTC
//#define SCL_PIN 5
//#define I2C_TIMEOUT 100
//#define I2C_FASTMODE 1
//
//#include <SoftI2CMaster.h>

// pin assignments
#define chipSelect 10
#define LED_GRN 4
#define LED_RED A3
#define BURN 8
#define VHFPOW 9
#define BUTTON1 A2
#define SD_POW 5

int accelX, accelY, accelZ;

void setup() {
  Serial.begin(115200);
  Serial.println("LIS2DS12");

  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED,LOW);
  digitalWrite(LED_GRN,HIGH);
  pinMode(SD_POW, OUTPUT);      
  digitalWrite(SD_POW, HIGH); 

  Wire.begin();

//  byte I2C_check = i2c_init();
//  if(I2C_check == false){
//    Serial.println("I2C Init Failed--SDA or SCL may not be pulled up!");
//     while(1){
//       flashRed();
//     }
//  }

  int testResponse = lis2TestResponse();
  if(testResponse != 67){
    Serial.println("Accel init failed");
    while(1){
      flashRed();
    }
  }
  Serial.println(testResponse);
  lis2Init();
}


void loop() {
  lis2FifoRead();
  Serial.print(accelX); Serial.print("\t");
  Serial.print(accelY); Serial.print("\t");
  Serial.println(accelZ);
  delay(100);

  Serial.println(lis2FifoPts());
}

void flashRed(){
   digitalWrite(LED_RED, HIGH);
   delay(500);
   digitalWrite(LED_RED, LOW);
   delay(500);
}



