#include <SPI.h>
#include <SdFat.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define LED 4
#define chipSelect 10   // microSD
#define DATAOUT 11      //MOSI
#define DATAIN 12       //MISO
#define SPICLOCK 13      //sck
#define chipSelectPinAccel 9  
#define INT0 2
#define INT1 3

// when storing magnitude of acceleraton watermark threshold are represented by 1Lsb = 3 samples
#define FIFO_WATERMARK (0x80) // samples 0x0C=12 0x24=36; 0x80 = 128
#define bufLength 768 // bytes: 3x watermark x2 bytes/sample
uint8_t accel[bufLength];

// SD file system
SdFat sd;
File dataFile;

uint32_t srate = 1600;
unsigned int fileCount = 0;
uint32_t bufsPerFile = 100;
uint32_t wavBufLength = bufLength;

typedef struct hdrstruct {
    char    rId[4];
    uint32_t rLen;
    char    wId[4];
    char    fId[4];
    uint32_t    fLen;
    uint16_t nFormatTag;
    uint16_t nChannels;
    uint32_t nSamplesPerSec;
    uint32_t nAvgBytesPerSec;
    uint16_t nBlockAlign;
    uint16_t  nBitsPerSamples;
    char    dId[4];
    uint32_t  dLen;
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
  //Serial.begin(115200);

  cbi(ADCSRA,ADEN);  // switch Analog to Digitalconverter OFF
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(INT0, INPUT);

  delay(500);

  // initialize microSD
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
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
      flashLed(500);
  }

  attachInterrupt(digitalPinToInterrupt(INT0), watermark, RISING);
  lis2SpiInit();
  
  digitalWrite(LED, HIGH);
}

volatile int bufsRec = 0;

void loop() {
  while (bufsRec < bufsPerFile) {
     system_sleep();
     processBuf();
  }
  bufsRec = 0;
  detachInterrupt(digitalPinToInterrupt(INT0));
  dataFile.close();
  //Serial.println("Done.");
  digitalWrite(LED, HIGH);
  delay(1000);
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
  fileCount += 1;
  sprintf(filename,"F%06d.wav",fileCount); //if can't open just use count
  dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
  
  while (!dataFile){
    fileCount += 1;
    sprintf(filename,"F%06d.wav",fileCount); //if can't open just use count
    dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
  }

  //intialize .wav file header
  sprintf(wav_hdr.rId,"RIFF");
  sprintf(wav_hdr.wId,"WAVE");
  sprintf(wav_hdr.fId,"fmt ");
  wav_hdr.fLen=0x10;
  wav_hdr.nFormatTag=1;
  wav_hdr.nChannels=1;
  wav_hdr.nSamplesPerSec=srate;
  wav_hdr.nAvgBytesPerSec=srate*2;
  wav_hdr.nBlockAlign=2;
  wav_hdr.nBitsPerSamples=16;
  sprintf(wav_hdr.dId,"data");
  wav_hdr.dLen = bufsPerFile * wavBufLength; // number of bytes in data
  wav_hdr.rLen = 36 + wav_hdr.dLen;  // total length of file in bytes - 8 bytes
  dataFile.write((uint8_t *)&wav_hdr, 44);
}

void watermark(){
  // do nothing just wake up
}

void processBuf(){
  bufsRec++;
  digitalWrite(LED, HIGH);
  
  lis2SpiFifoRead(bufLength);  //bytes to read
  dataFile.write(&accel, bufLength);

  digitalWrite(LED, LOW);
}

//****************************************************************  
// set system into the sleep state 
// system wakes up when interrupt detected
void system_sleep() {
  // make all pin inputs and enable pullups to reduce power

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  power_all_disable();
  sleep_mode();  // go to sleep
  // ...sleeping here....  
  sleep_disable();
  power_all_enable();
  
}
