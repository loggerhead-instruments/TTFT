
// - should we flag overflow? Could store in upper bit of data--or not worry about it, because you don't know how much you miss
// Issue with magnitude mode is that data is 800 Hz, not 1600 Hz; 3200 Hz results in a triangle wave

#include <SPI.h>
#include <SdFat.h>
#include <avr/sleep.h>
#include <avr/power.h>

// If 3 channel recording defined will store raw 3 axes data
// Otherwise will store magnitude of 3 channels
//#define CHAN3

#define LED 4
#define chipSelect 10   // microSD
#define DATAOUT 11      //MOSI
#define DATAIN 12       //MISO
#define SPICLOCK 13      //sck
#define chipSelectPinAccel 9  
#define INT0 2
#define INT1 3

// when storing magnitude of acceleraton watermark threshold are represented by 1Lsb = 3 samples
#define FIFO_WATERMARK (0x128) // samples 0x0C=12 0x24=36; 0x2A=42; 0x80 = 128
#define bufLength 384 // samples: 3x watermark
int16_t accel[bufLength];
uint32_t bufsPerFile = 75; // each buffer is 0.08 seconds; 750 buffers = 1 minute
uint32_t wavBufLength = bufLength;

// SD file system
SdFat sd;
File dataFile;

uint32_t srate = 800;
unsigned int fileCount = 0; 
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
  delay(4000);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(INT0, INPUT_PULLUP);
  pinMode(INT1, INPUT_PULLUP);
  delay(1000);
  cbi(ADCSRA,ADEN);  // switch Analog to Digital converter OFF

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

  // initialize and test microSD
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    flashLed(1000);
  }
  
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0)); // with breadboard, speeds higher than 1MHz fail

  int testResponse = lis2SpiTestResponse();
  if (testResponse != 67) {
      flashLed(2000);
  }

  // double-tap to start
  digitalWrite(LED, LOW);
  lis2SpiDt(); // setup for double tap
  attachInterrupt(digitalPinToInterrupt(INT0), doubleTap, FALLING);
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
 
  lis2SpiInit();
}

volatile int bufsRec = 0;

void loop() {
  while (bufsRec < bufsPerFile) {
     processBuf(); // process buffer first to empty FIFO so don't miss watermark
     //if(lis2SpiFifoStatus()==0) system_sleep();
     //if(lis2SpiFifoPts() < 128) system_sleep();
     system_sleep();
     // ... ASLEEP HERE...
  }
  //digitalWrite(LED, HIGH);
  introPeriod = 0;
  bufsRec = 0;
  dataFile.close();
  fileInit();
}

void processBuf(){
  while((lis2SpiFifoPts() * 3 > bufLength)){
    if(introPeriod) digitalWrite(LED, HIGH);
    bufsRec++;
    lis2SpiFifoRead(bufLength);  //samples to read
    dataFile.write(&accel, bufLength*2);
  }
  digitalWrite(LED, LOW);
}

void flashLed(int interval) {
  while(1){
    digitalWrite(LED, HIGH);
    delay(interval);
    digitalWrite(LED, LOW);
    delay(interval);
  }
}

void fileInit() {
  char filename[12]; 
  fileCount += 1;
  sprintf(filename,"F%06d.wav",fileCount);
  dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
  while (!dataFile){
    fileCount += 1;
    sprintf(filename,"F%06d.wav",fileCount); //if can't open just use count
    dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
  }
  dataFile.write((uint8_t *)&wav_hdr, 44);
}

void doubleTap(){
  // do nothing just wake up
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
