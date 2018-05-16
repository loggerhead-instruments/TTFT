#include <SPI.h>
#include <SdFat.h>

#define LED 4
#define chipSelect 10   // microSD
#define DATAOUT 11      //MOSI
#define DATAIN 12       //MISO
#define SPICLOCK 13      //sck
#define chipSelectPinAccel 9  
#define shifterEnable 14

#define bufLength 128

volatile int accel[bufLength];

// SD file system
SdFat sd;
File dataFile;

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(shifterEnable, OUTPUT);
  digitalWrite(shifterEnable, HIGH);

  delay(500);

  // initialize microSD
  while (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    flashLed(50);
  }
  fileInit();
  
  // initalize the  data ready and chip select pins:
  pinMode(chipSelectPinAccel, OUTPUT);
  digitalWrite(chipSelectPinAccel, HIGH);
  pinMode(SPICLOCK, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // with breadboard, speeds higher than 1MHz fail

  int testResponse = lis2SpiTestResponse();
  if (testResponse != 67) {
    while (1) {
      flashLed(500);
    }
  }


  lis2SpiInit();
  
  digitalWrite(LED, LOW);
  
  
}

int count = 0;
void loop() {
while (count < 5000) {
    if (lis2SpiFifoStatus() > 0) {
      digitalWrite(LED, HIGH);
      count++;
      lis2SpiFifoRead(bufLength);  //bytes to read
      Serial.println((int) accel[1]<<8 | accel[0]);
      for (int i = 0; i < bufLength; i += 2) {
        dataFile.println((int) accel[i + 1] << 8 | accel[i]);
      } 
      digitalWrite(LED, LOW);
    }
  }
  dataFile.close();
  Serial.println("Done.");
  digitalWrite(LED, LOW);
  while (1);
  
}

void flashLed(int interval) {
  digitalWrite(LED, HIGH);
  delay(interval);
  digitalWrite(LED, LOW);
  delay(interval);
}

void fileInit() {
  char filename[12];
  sprintf(filename, "test.csv"); //filename is DDHHMM
  dataFile = sd.open(filename, O_WRITE | O_CREAT | O_APPEND);
}
