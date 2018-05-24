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
#define bufLength 384 // samples: 3x watermark
int16_t accel[bufLength];

// SD file system
SdFat sd;
File dataFile;

uint32_t srate = 1600;
unsigned int fileCount = 0;
uint32_t bufsPerFile = 750; // each buffer is 0.08 seconds; 750 buffers = 1 minute
uint32_t wavBufLength = bufLength;

volatile boolean introPeriod = 1;

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
    uint16_t  nBitsPerSample;
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

  cbi(ADCSRA,ADEN);  // switch Analog to Digital converter OFF
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(INT0, INPUT);
  pinMode(INT1, INPUT);
  delay(500);

  // initialize and test microSD
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    flashLed(50);
  }
  
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

  // double-tap to start
  digitalWrite(LED, LOW);
  lis2SpiDt(); // setup for double tap
  attachInterrupt(digitalPinToInterrupt(INT0), doubleTap, RISING);
  system_sleep();

  // 
  // ASLEEP HERE
  //
  
  detachInterrupt(digitalPinToInterrupt(INT0));

  // initialize microSD
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    flashLed(50);
  }
  fileInit();
  
  attachInterrupt(digitalPinToInterrupt(INT1), watermark, RISING);
  lis2SpiInit();
}

volatile int bufsRec = 0;

void loop() {
  while (bufsRec < bufsPerFile) {
     system_sleep();
     processBuf();
  }
  introPeriod = 0;
  bufsRec = 0;
  dataFile.close();
  fileInit();
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
  wav_hdr.fLen = 0x10;
  wav_hdr.nFormatTag = 1;
  wav_hdr.nChannels = 3;
  wav_hdr.nSamplesPerSec = srate;
  wav_hdr.nAvgBytesPerSec = srate * 6;
  wav_hdr.nBlockAlign = 6;
  wav_hdr.nBitsPerSample = 16;
  sprintf(wav_hdr.dId,"data");
  wav_hdr.dLen = bufsPerFile * wavBufLength * 2; // number of bytes in data
  wav_hdr.rLen = 36 + wav_hdr.dLen;  // total length of file in bytes - 8 bytes
  dataFile.write((uint8_t *)&wav_hdr, 44);
}

void doubleTap(){
  // do nothing just wake up
}

void watermark(){
  // do nothing just wake up
}

void processBuf(){
  if(introPeriod) digitalWrite(LED, HIGH);
  bufsRec++;
  lis2SpiFifoRead(bufLength);  //samples to read
  dataFile.write(&accel, bufLength*2);
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
