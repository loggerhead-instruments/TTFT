// Only save on detections
// 1600 Hz 0.6 mA when not writing to card; 1.9 mA when writing to card

// duty cycle to save power?

#include <SPI.h>
#include <SdFat.h>
#include <avr/sleep.h>
#include <avr/power.h>

// If 3 channel recording defined will store only Z axis but let's it run at 1600 and 3200 Hz
// Otherwise will store magnitude of 3 channels
 #define CHAN3
 uint32_t srate = 1600;

#define LED 4
#define chipSelect 10   // microSD
#define DATAOUT 11      //MOSI
#define DATAIN 12       //MISO
#define SPICLOCK 13      //sck
#define chipSelectPinAccel 9  
#define INT0 2
#define INT1 3

// when storing magnitude of acceleraton watermark threshold are represented by 1Lsb = 3 samples
// max buffer is 256 sets of 3-axis data
#define FIFO_WATERMARK (0x80) // samples 0x0C=12 0x24=36; 0x2A=42; 0x80 = 128
#define bufLength 128 // samples: 3x watermark
int16_t accel[bufLength];
uint32_t bufsPerFile = 750;  //should be 1 minute long files at 1600 Hz sample rate
uint32_t wavBufLength = bufLength;

int16_t threshold = 200; // threshold for storing raw buffer

// SD file system
SdFat sd;
File dataFile;


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
  
  //intialize .wav file header
  sprintf(wav_hdr.rId,"RIFF");
  sprintf(wav_hdr.wId,"WAVE");
  sprintf(wav_hdr.fId,"fmt ");
  wav_hdr.fLen = 0x10;
  wav_hdr.nFormatTag = 1;
  wav_hdr.nChannels = 1;
  wav_hdr.nAvgBytesPerSec = srate * 2;
  wav_hdr.nSamplesPerSec = srate;
  
  wav_hdr.nBlockAlign = 6;
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
  while((lis2SpiFifoPts() > bufLength)){
    lis2SpiFifoRead(bufLength);  //samples to read
 //   if(detectSound()){
      if(introPeriod)  digitalWrite(LED, HIGH);
      bufsRec++;
      dataFile.write(&accel, bufLength*2);
      digitalWrite(LED, LOW);
 //   }
  }
  // digitalWrite(LED, LOW);
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


// simple algorithm to detect whether buffer contains sound
int16_t maxFiltered = 0;
int diffData;

boolean detectSound(){
  // High-pass filter options:
  // diff()
  // IIR
  // FIR

  // Threshold options:
  // -fixed
  // -dynamic (e.g. 4 * SD)

  maxFiltered = 0;
  for (int i=1; i<bufLength; i++){
    diffData = accel[i] - accel[i-1];
    if (diffData > maxFiltered) maxFiltered = diffData;
  }
  if(maxFiltered > threshold) 
    return 1;
  else
    return 0;
}


//****************************************************************  
// set system into the sleep state 
// system wakes up when interrupt detected
void system_sleep() {
  // make all pin inputs and enable pullups to reduce power
  cbi(ADCSRA,ADEN);  // switch Analog to Digital converter OFF
  power_timer1_disable();  
  power_usart0_disable();
  power_twi_disable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  power_all_disable();
  attachInterrupt(digitalPinToInterrupt(INT1), watermark, LOW);
  sleep_mode();  // go to sleep
  // ...sleeping here....  
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(INT1));
  power_all_enable();
  cbi(ADCSRA,ADEN);  // switch Analog to Digital converter OFF
  power_timer1_disable();  
  power_usart0_disable();
  power_twi_disable();  
}
