// test ST accelerometer with OpenTag
// I2C interface

// - setup 2 MHz SPI read of LIS2 -- 5x faster than I2C
// - write data to sd card and benchmark
// - sleep mode and watermark to wake

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

boolean saveData = 0;  // set to 1 to save data to microSD; for debugging

// pin assignments
#define chipSelect 10
#define LED_GRN A3
#define LED_RED 4
#define BURN 8
#define VHFPOW 9
#define BUTTON1 A2

#define DATAOUT 11      //MOSI
#define DATAIN 12       //MISO
#define SPICLOCK 13      //sck
#define chipSelectPin 3 //Hall pin

volatile int accel[32];

// SD file system
SdFat sd;
File dataFile;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("LIS2DS12");

  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GRN, HIGH);

  //  Wire.begin();
  //  Wire.setClock(400000);

  //  byte I2C_check = i2c_init();
  //  if(I2C_check == false){
  //    Serial.println("I2C Init Failed--SDA or SCL may not be pulled up!");
  //     while(1){
  //       flashRed();
  //     }
  //  }
  
  // initalize the  data ready and chip select pins:
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
  pinMode(SPICLOCK, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // with breadboard, speeds higher than 1MHz fail
  // might get better when chip on board with short traces
  
  int testResponse = lis2SpiTestResponse();
  if (testResponse != 67) {
    Serial.println("Accel init failed");
    while (1) {
      flashRed();
    }
  }

  if (saveData) {
    Serial.println("Init microSD");
    // see if the card is present and can be initialized:
    while (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
      Serial.println("Card failed");
      flashRed();
    }
    fileInit();
  }


  Serial.println(testResponse);
  lis2SpiInit();
}

int count = 0;
void loop() {
  while (count < 5000) {
    if (lis2SpiFifoStatus() > 0) {
      count++;
      lis2SpiFifoRead(24);  //bytes to read (Wire library has max of 32)
      Serial.println((int) accel[1]<<8 | accel[0]);
      if (saveData) {
        for (int i = 0; i < 24; i += 2) {
          dataFile.println((int) accel[i + 1] << 8 | accel[i]);
        }
      }
    }
  }
  if (saveData) dataFile.close();
  Serial.println("Done.");
  while (1);
}

void flashRed() {
  digitalWrite(LED_RED, HIGH);
  delay(500);
  digitalWrite(LED_RED, LOW);
  delay(500);
}

void fileInit() {
  char filename[12];
  sprintf(filename, "test.csv"); //filename is DDHHMM
  dataFile = sd.open(filename, O_WRITE | O_CREAT | O_APPEND);
}


