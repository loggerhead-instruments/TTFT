// test ST accelerometer with OpenTag
// I2C interface

// - setup 2 MHz SPI read of LIS2 -- 5x faster than I2C
// - setup watermark
// - write data to sd card and benchmark
// - sleep mode and watermark to wake
// - change to use magnitude and benchmark


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
#define LED_GRN A3
#define LED_RED 4
#define BURN 8
#define VHFPOW 9
#define BUTTON1 A2

int accel;

void setup() {
  Serial.begin(115200);
  Serial.println("LIS2DS12");

  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED,LOW);
  digitalWrite(LED_GRN,HIGH);

  Wire.begin();
  Wire.setClock(400000);

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
 if(lis2FifoStatus()>0){
   lis2FifoRead(36);
 }
}

void flashRed(){
   digitalWrite(LED_RED, HIGH);
   delay(500);
   digitalWrite(LED_RED, LOW);
   delay(500);
}



