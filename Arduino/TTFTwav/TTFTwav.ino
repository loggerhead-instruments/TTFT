#include <SPI.h>
#include <SdFat.h>

#define LED 4
#define chipSelect 10   // microSD
#define DATAOUT 11      //MOSI
#define DATAIN 12       //MISO
#define SPICLOCK 13      //sck
#define chipSelectPinAccel 9  
#define INT0 2
#define INT1 3

#define bufLength 108 // should be 3x watermark
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


void setup() {
  Serial.begin(115200);
  
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

int bufsRec = 0;

void loop() {
  while (bufsRec < bufsPerFile) {
//    int nsamples = lis2SpiFifoPts() * 3;
//    while(nsamples > bufLength){
//      bufsRec++;
//      Serial.print("n:"); Serial.println(nsamples);
//      Serial.print("status:"); Serial.println(lis2SpiFifoStatus());
//     // digitalWrite(LED, HIGH);
//      lis2SpiFifoRead(bufLength);  //bytes to read
//     // Serial.print("v:");Serial.println((int) accel[1]<<8 | accel[0]);
//      dataFile.write(&accel, bufLength);
//      digitalWrite(LED, LOW);
//      nsamples = lis2SpiFifoPts() * 3;
//    }
//    delay(1);
  }
  bufsRec = 0;
  dataFile.close();
  Serial.println("Done.");
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
//  bufsRec++;
  digitalWrite(LED, HIGH);
  
  lis2SpiFifoRead(bufLength);  //bytes to read
  dataFile.write(&accel, bufLength);

  digitalWrite(LED, LOW);
}

