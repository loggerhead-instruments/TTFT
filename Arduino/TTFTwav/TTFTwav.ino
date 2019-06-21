
// AVR Board: Arduino Pro/Pro Mini
// Microprocessor: ATmega328P, 3.3V, 8MHz

// - should we flag overflow? Could store in upper bit of data--or not worry about it, because you don't know how much you miss
// Issue with magnitude mode is that data is 800 Hz, not 1600 Hz; 3200 Hz results in a triangle wave

// Include necessary libraries
#include <SPI.h>
#include <SdFat.h>
#include <avr/sleep.h>
#include <avr/power.h>

// If 3 channel recording defined will store raw 3 axes data
// Otherwise will store magnitude of 3 channels
#define CHAN3

// Define other properties
#define LED 4
#define chipSelect 10   // microSD
#define DATAOUT 11      //MOSI
#define DATAIN 12       //MISO
#define SPICLOCK 13      //sck
#define chipSelectPinAccel 9  
#define INT0 2
#define INT1 3

// when storing magnitude of acceleraton watermark threshold are represented by 1Lsb = 3 samples
#define FIFO_WATERMARK (0x80) // samples 0x0C=12 0x24=36; 0x2A=42; 0x80 = 128
#define bufLength 384 // samples: 3x watermark
int16_t accel[bufLength];
uint32_t bufsPerFile = 750; // each buffer is 0.08 seconds; 750 buffers = 1 minute
uint32_t wavBufLength = bufLength;

// SD file system
SdFat sd;
File dataFile;

uint32_t srate = 1600;
unsigned int fileCount = 0; 
volatile int bufsRec = 0;

typedef struct hdrstruct {
    char     rId[4];
    uint32_t rLen;
    char     wId[4];
    char     fId[4];
    uint32_t fLen;
    uint16_t nFormatTag;
    uint16_t nChannels;
    uint32_t nSamplesPerSec;
    uint32_t nAvgBytesPerSec;
    uint16_t nBlockAlign;
    uint16_t nBitsPerSample;
    char     dId[4];
    uint32_t dLen;
} HdrStruct;

HdrStruct wav_hdr;

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

void setup() {
  delay(4000);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(INT0, INPUT_PULLUP);
  pinMode(INT1, INPUT_PULLUP);
  delay(1000);
  digitalWrite(LED, LOW);

  // deactivate board components not in use (timer0 used in delay, usart0, spi, twi in use)
  cbi(ADCSRA,ADEN);  // switch Analog to Digital converter OFF
  ADCSRA = 0;         // disable ADC
  power_adc_disable(); // ADC converter
  power_timer1_disable();// Timer 1
  power_timer2_disable();// Timer 2
  power_twi_disable(); // TWI (I2C)
  power_usart0_disable();// Serial (USART) 

  //intialize .wav file header
  sprintf(wav_hdr.rId,"RIFF");
  sprintf(wav_hdr.wId,"WAVE");
  sprintf(wav_hdr.fId,"fmt ");
  wav_hdr.fLen = 0x10;
  wav_hdr.nFormatTag = 1;
  #ifdef CHAN3
    wav_hdr.nChannels = 3;
    wav_hdr.nAvgBytesPerSec = srate * 6;
    wav_hdr.nBlockAlign = 6;
  #else
    wav_hdr.nChannels = 1;
    wav_hdr.nAvgBytesPerSec = srate * 2;
    wav_hdr.nBlockAlign = 2;
  #endif
  wav_hdr.nSamplesPerSec = srate;
  wav_hdr.nBitsPerSample = 16;
  sprintf(wav_hdr.dId,"data");
  wav_hdr.dLen = bufsPerFile * wavBufLength * 2; // number of bytes in data
  wav_hdr.rLen = 36 + wav_hdr.dLen;  // total length of file in bytes - 8 bytes
  
  // initalize the  data ready and chip select pins:
  pinMode(chipSelectPinAccel, OUTPUT);
  digitalWrite(chipSelectPinAccel, HIGH);
  pinMode(SPICLOCK, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);

  // initialize and test microSD - start flashing LED slowly if SD card fails
  while (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    flashLed(1000);
  }

  // Start SPI communication with accelerometer
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0)); // with breadboard, speeds higher than 1MHz fail

  // Test accelerometer - start flashing LED quickly if accelerometer fails
  int testResponse = lis2SpiTestResponse();
  while (testResponse != 67) {
      flashLed(500);
      int testResponse = lis2SpiTestResponse();
  }
  // IS THERE A SPI.endTransaction() missing here?
  
  // initialize microSD
  while (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    flashLed(200);
    delay(1000);
  }

  // Check which file number to start at
  checkExistingFiles();

  // initialize accelerometer
  lis2SpiInit();
  
}

// Main recording loop
void loop() {
  // initialize file
  fileInit();
  while (bufsRec < bufsPerFile) {
     processBuf(); // process buffer first to empty FIFO so don't miss watermark
     //if(lis2SpiFifoStatus()==0) system_sleep();
     //if(lis2SpiFifoPts() < 128) system_sleep();
     
     if (bufsRec < bufsPerFile) system_sleep();
     // ... ASLEEP HERE...
  }
  bufsRec = 0;
  dataFile.close();
}

void processBuf(){
  while((lis2SpiFifoPts() * 3 > bufLength)){
    bufsRec++;
    lis2SpiFifoRead(bufLength);  //samples to read
    dataFile.write(&accel, bufLength*2);
  }
}

void flashLed(int interval) {
  for(int n=0; n<3; n++){
    digitalWrite(LED, HIGH);
    delay(interval);
    digitalWrite(LED, LOW);
    delay(interval);
  }
}

void checkExistingFiles() {
  char filename[12];

  // Increment by 1 as when creating new file
  fileCount++;
  sprintf(filename,"F%07d.wav",fileCount);

  // check if file exists
  while (sd.exists(filename)) {
    fileCount++;
    sprintf(filename,"F%07d.wav",fileCount);    
  }

  // Decrease by 1 again (fileInit increases by 1)
  fileCount--;
}

void fileInit() {
  char filename[12];

  // Turn LED on while making new file
  digitalWrite(LED, HIGH); 
  
  fileCount++;
  sprintf(filename,"F%07d.wav",fileCount);
  dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
  while (!dataFile){
    fileCount++;
    sprintf(filename,"F%07d.wav",fileCount); //if can't open just use count
    dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
  }
  dataFile.write((uint8_t *)&wav_hdr, 44);

  // Turn LED off
  digitalWrite(LED, LOW);
}

void watermark(){
  // wake up
}



//****************************************************************  
// set system into the sleep state 
// system wakes up when interrupt detected
void system_sleep() {
  // make all pin inputs and enable pullups to reduce power
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  power_all_disable();
  attachInterrupt(digitalPinToInterrupt(INT1), watermark, LOW);
  sleep_mode();  // go to sleep
  // ...sleeping here....  
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(INT1));
  power_all_enable();
}
