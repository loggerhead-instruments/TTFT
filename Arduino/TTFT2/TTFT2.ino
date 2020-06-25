// TTFT2
// Board bootloader loaded using AtmelICE from Arduino IDE
// Board programmed using USB via Arduino Genuino Zero Native USB port using SAMD v 1.8.3

// Note: SAMD variants.h needs to have the following lines changed, otherwise SD_POW will get switched off when read USB
// C:\Users\dmann\AppData\Local\Arduino15\packages\arduino\hardware\samd\1.8.3\variants\arduino_zero\variant.h
// LEDs
//#define PIN_LED_13           PIN_A5
//#define PIN_LED_RXL          PIN_A5
//#define PIN_LED_TXL          PIN_A5
//#define PIN_LED              PIN_A5


// To Do:

// Optimizations (not critical):
// - set file duration from menu
// - to save power:
//       + change unused pins to INPUTS
//       + make a larger internal buffer so uSD writes are at least 512 bytes 
// - have backup wake from RTC (in case accelerometer hiccup)?
// - reset on WDT and check if write fail (would need bypass of startup menu on restart; maybe check if USB connected to enter menu)
// - delayed start

#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <RTCZero.h>
#include "LowPower.h"

SdFat sd;
File dataFile;

// *** USER SETTINGS *********************//
int printDiags = 0;
uint32_t srate = 1600;
unsigned int fileCount = 0; 
volatile boolean introPeriod = 1;
uint32_t bufsPerFile = 750; // each buffer is 0.08 seconds; 750 buffers = 1 minute
// If 3 channel recording defined will store raw 3 axes data
// Otherwise will store magnitude of 3 channels
#define CHAN3

// desired slope threshold (A(t)-A(t-1)) 19 mg for 5 blocks of 16 samples
#define DET_THRESHOLD 311 // Detection threshold - 19[mg]/0.061 [mg/sample]
#define DET_CRIT 6        // Critical number of detected blocks
#define DET_BLOCK 16      // Number of accelerometer samples per block
boolean call = 0;
#define LED_HOLD 10;       // Keep LED activated for this many ACC buffers
int post_call_blocks = 0; // variable for counting ACC buffers
boolean turn_off;
uint32_t eT;
//****************************************//

#define CPU_HZ 48000000

// Pin Assignments
#define ledGreen A5
#define vSense A4
#define chipSelectPinAccel 5
#define chipSelectMemory 12 
#define chipSelect 10  // microSD
#define INT1 13
#define INT2 11
#define SDPOW 25 // previously PIN_LED_RXL
#define DATAOUT 23      //MOSI PB10
#define DATAIN 22       //MISO PA12
#define SPICLOCK 24     //SCK PB11
#define PWM 8

volatile int bufsRec = 0;

#define ledGreen_ON LOW
#define ledGreen_OFF HIGH

byte toggleledGreen = 1;

// when storing magnitude of acceleration watermark threshold are represented by 1Lsb = 3 samples
#define FIFO_WATERMARK (0x80) // samples 0x0C=12 0x24=36; 0x2A=42; 0x80 = 128
#define bufLength 384 // samples: 3x watermark
int16_t accel[bufLength];

uint32_t wavBufLength = bufLength;

// Pressure/Temp
#define pressAddress 0x76
float MS58xx_constant = 8192.0; // for 30 bar sensor
// float MS58xx_constant = 327680.0; // for 2 bar sensor; will switch to this if 30 bar fails to give good depth

byte Tbuff[3];
byte Pbuff[3];
volatile float depth, temperature, pressure_mbar;
boolean togglePress = 0; //flag to toggle conversion of pressure and temperature

//Pressure and temp calibration coefficients
uint16_t PSENS; //pressure sensitivity
uint16_t POFF;  //Pressure offset
uint16_t TCSENS; //Temp coefficient of pressure sensitivity
uint16_t TCOFF; //Temp coefficient of pressure offset
uint16_t TREF;  //Ref temperature
uint16_t TEMPSENS; //Temperature sensitivity coefficient

/* Create an rtc object */
RTCZero rtc;

/* Change these values to set the current initial time and date */
volatile byte second = 0;
volatile byte minute = 0;
volatile byte hour = 15;
volatile byte day = 1;
volatile byte month = 1;
volatile byte year = 17;

#define SECONDS_IN_MINUTE 60
#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400
#define SECONDS_IN_YEAR 31536000
#define SECONDS_IN_LEAP 31622400

// Delayed start in seconds
#define startDelay 60
//#define startHour 13 // start next 9 am east coast time
#define startHour 15 // start next 11 am east coast time
#define startMin 00
#define startSec 00

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

void setup() {
  SerialUSB.begin(9600);
  digitalWrite(ledGreen, ledGreen_ON);
  delay(1000);
  digitalWrite(ledGreen, ledGreen_OFF);
    
  delay(10000);  // long delay here to make easier to reprogram
  SerialUSB.println("TTFT2");

  // Create header info for wav files
  makeWavHeader();

  // Communicate with iTag terminal
  Wire.begin();
  Wire.setClock(400);  // set I2C clock to 400 kHz
  rtc.begin();
  sensorInit();
  setupMenu();  

  // Send final serial message, then turn off USB
  
  SerialUSB.println("Running"); 
  SerialUSB.println("USB disabled");
  SerialUSB.println("Ignore error");
  delay(500); // time to display
                  
  // Turn off USB (so pins don't corrode in seawater; and it doesn't trigger interrupts)
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;

// STAY POWERED DOWN UNTIL WAKE UP TIME

// Flash LED a few times after deploying
  for(int i=0; i<3; i++) {
    delay(400);
    digitalWrite(ledGreen, ledGreen_ON);
    delay(600);
    digitalWrite(ledGreen, ledGreen_OFF);
  }
 
  // initialize RTC alarm
  rtc.setAlarmTime(startHour, startMin, startSec);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  rtc.attachInterrupt(alarmMatch);
  
  // Store current time and use as reference
  getTime();
  long startTime = RTCToUNIXTime(year, month, day, hour, minute, second);

  // Delayed start sleep loop - wakes up on RTC alarm provided elapsed time exceeds minimum startDelay
  // Can be simplified to just operate with LowPower.standby and include yy-mm-dd in alarm match
  uint32_t eTime = 0;
  while(eTime<startDelay) {
    // Sleep until next interrupt
    //system_sleep(); // this did not work!
    LowPower.standby(); // lowpower standby works - wake up next time RTC alarm triggers

    // Single led when RTC triggers
    digitalWrite(ledGreen, ledGreen_ON);
    delay(200);
    digitalWrite(ledGreen, ledGreen_OFF);
    
    // Update time and check if we are good
    getTime();
    eTime = RTCToUNIXTime(year, month, day, hour, minute, second) - startTime;
  }
  rtc.detachInterrupt();


// Flash LED for first minute when recording is started
  for(int i=0; i<59; i++) {
    delay(400);
    digitalWrite(ledGreen, ledGreen_ON);
    delay(600);
    digitalWrite(ledGreen, ledGreen_OFF);
  }


// NOW ACTIVATE EVERYTHING
  
  updateTemp();  // get first temperature reading ready
  fileInit();
  lis2SpiInit();

//  Configure sleep mode and interrupt. Both attachinterrupt and subsequent parts are needed for interrupt to work
  attachInterrupt(digitalPinToInterrupt(INT2), watermark, FALLING); 
  
  // looking at this forum because wake from interrupt not working
  // https://forum.arduino.cc/index.php?topic=410699.0
  // Set the XOSC32K to run in standby
   SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;
   
   // Configure EIC to use GCLK1 which uses XOSC32K
   // This has to be done after the first call to attachInterrupt()
   GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |
                       GCLK_CLKCTRL_GEN_GCLK1 |
                       GCLK_CLKCTRL_CLKEN;

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}



void loop() {
  while (bufsRec < bufsPerFile) {
     getTime();
     //if(second==0 | second==1) digitalWrite(ledGreen, ledGreen_ON); // flash LED on every minute
     processBuf(); // process buffer first to empty FIFO so don't miss watermark

     // Sleep with interrupt seems to work with this. Will it hang eventually? Need to test long
     LowPower.standby();
    
     // SLEEP MODE DOES NOT WORK RIGHT NOW - INTERRUPT DOES NOT WAKE (line 152)
     //if(digitalRead(INT2)==1 & lis2SpiFifoPts()<40) system_sleep(); // look at the interrupt flag and mostly empty FIFO to decide whether to sleep; INT2 needs to be high before sleeping
      
     //if(second==0 | second==1) digitalWrite(ledGreen, ledGreen_OFF); 
     // ... NOT ASLEEP ...
  }

  introPeriod = 0;
  bufsRec = 0;
  dataFile.close();
  fileInit();

  // could write pressure/temperature here to log file
  
}

void processBuf(){
  while((lis2SpiFifoPts() * 3 > bufLength)){
   //if(introPeriod) digitalWrite(ledGreen, ledGreen_ON);
    bufsRec++;
    uint32_t eTime = millis();
    lis2SpiFifoRead(bufLength);  //samples to read
    dataFile.write(&accel, bufLength*2);

    //SerialUSB.println(accel[0]);  // for debugging, look at one accelerometer value per buffer
    
    //uint32_t eTime = millis();
    checkvocs();
    //eTime = millis()-eTime;
    //Event detector takes ~1ms << 80ms acc buffer
    //SerialUSB.print(" Time to run event detector: ");
    //SerialUSB.println(eTime);
  }
  //if(introPeriod) digitalWrite(ledGreen, ledGreen_OFF);
}

void checkvocs() {
  // Check if LED is still on from previous call
  if (call) {
    // If call was recently detected, track when to turn LED off 
    digitalWrite(ledGreen, ledGreen_ON); 
    
    // Count an extra block
    post_call_blocks++;
    turn_off = post_call_blocks>LED_HOLD;
    // If we have spent enough acc buffers with LED on, reset LED
    if (turn_off) {
      call = 0;
      post_call_blocks = 0;
      digitalWrite(ledGreen, ledGreen_OFF);
      // debugging
      //SerialUSB.println(" Turning off LED ");
      //eT = millis()-eT;
      //SerialUSB.println(" Time with LED on: ");
      //SerialUSB.println(eT);
    }
  }
  else {
    // Check for calls and activate LED if call is found
    detectvocs();
    if (call) {
      digitalWrite(ledGreen, ledGreen_ON); 
      //SerialUSB.println(" Turning LED on");
      eT = millis();
    }
  }
}

void detectvocs() {
  // default is no calls
  call = 0;
  
  // Create variables for detector
  int dt = 0; 
  int sample = 1; // 
  int max_sample = bufLength/3-1; // Can be defined
  boolean blockdone = 0;
  int blockrem = 0;

  // Create variables for differencing (can be done as part of acc settings)
  int16_t cur = accel[0];
  int16_t next = accel[0];

  // Go through sequence
  while (sample<max_sample) {
    // Update value
    cur = accel[(sample-1)*3];     // grab value
    next = accel[(sample)*3];
    blockrem = sample%DET_BLOCK; // calculate how far from new block
    


    if ( blockrem == 0) {
      blockdone=0; // initialize new block
      //SerialUSB.print("Sample ");
      //SerialUSB.print(sample); 
      //SerialUSB.println("New Block ");
    }
    
    // check if detected
    if (blockdone==0) {
      if (abs(next-cur)>DET_THRESHOLD) {
//        SerialUSB.print("Sample ");
//        SerialUSB.print(sample); 
//        SerialUSB.print(" value ");
//        SerialUSB.print(next); 
//        SerialUSB.print(" diff ");
//        SerialUSB.println(abs(next-cur)); 
        dt++;
        blockdone=1;
      }
    }
    // then move to next sample
    sample++;
  }
 
  if (dt>DET_CRIT) {
    // Found a call!
    call = 1;
    SerialUSB.println("Call detected!"); // debugging
  }
  
}

void sensorInit(){
 // initialize and test sensors
  SerialUSB.println("Sensor Init");

  pinMode(ledGreen, OUTPUT);
  pinMode(vSense, INPUT);
  pinMode(INT1, INPUT_PULLUP);
  pinMode(INT2, INPUT_PULLUP);

  pinMode(chipSelectPinAccel, OUTPUT);
  digitalWrite(chipSelectPinAccel, HIGH);
  pinMode(chipSelectMemory, OUTPUT);  // flash memory chip
  digitalWrite(chipSelectMemory, HIGH);
  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH);

  pinMode(SPICLOCK, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);

  pinMode(SDPOW, OUTPUT);
  digitalWrite(SDPOW, HIGH); // turn on power to SD
  // see if the card is present and can be initialized:
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    SerialUSB.println("SD failed");
    flashLed(100);
  }
  SerialUSB.println("SD init");

  // Digital IO
  digitalWrite(ledGreen, ledGreen_ON);

  // battery voltage measurement
  SerialUSB.print("Battery: ");
  SerialUSB.println(readVoltage());

  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0)); // with breadboard speeds higher than 1MHz fail
  SerialUSB.println("SPI Started");

  int testResponse = lis2SpiTestResponse();
  SerialUSB.print("Accelerometer:");
  if (testResponse != 67) {
      SerialUSB.println(testResponse);
      SerialUSB.println(" Not connected");
      flashLed(500);
  }
  else{
    SerialUSB.println(" connected");
  }

//  // Pressure sensor
//  if(pressInit()){
//    SerialUSB.print("MS5837 Pressure Detected: ");
//    updatePress();
//    delay(10);
//    readPress();
//    updateTemp();
//    delay(10);
//    readTemp();
//    calcPressTemp();
//    SerialUSB.print("Press (mBar): "); SerialUSB.print(pressure_mbar);
//    SerialUSB.print("  Depth: "); SerialUSB.print(depth);
//    SerialUSB.print("  Temp: "); SerialUSB.println(temperature);
//  }

  digitalWrite(ledGreen, ledGreen_OFF);

  long curTime = RTCToUNIXTime(year, month, day, hour, minute, second);
  SerialUSB.print("Current UNIX time UTC:"); 
  SerialUSB.println(curTime);
  printTime();
}

float readVoltage(){
  float vDivider = 0.5;
  float vReg = 3.3;
  float voltage = (float) analogRead(vSense) * vReg / (vDivider * 1024.0);
  return voltage;
}

void resetFunc(){

}

void powerDown(){
  digitalWrite(ledGreen, ledGreen_OFF);
}

void makeWavHeader(){
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
}

void flashLed(int interval) {
  while(1){
    digitalWrite(ledGreen, ledGreen_ON);
    delay(interval);
    digitalWrite(ledGreen, ledGreen_OFF);
    delay(interval);
    digitalWrite(ledGreen, ledGreen_ON);
    delay(interval);
    digitalWrite(ledGreen, ledGreen_OFF);
    delay(interval);
    digitalWrite(ledGreen, ledGreen_ON);
    delay(interval);
    digitalWrite(ledGreen, ledGreen_OFF);
    delay(10*interval);
  }
}

void fileInit() {
  char filename[40]; 
  fileCount += 1;
  getTime(); // update time
  SdFile::dateTimeCallback(file_date_time);
  uint32_t ms = millis() % 1000; // store also (potentially inaccurate) ms info
  sprintf(filename,"F%04d_%02d-%02d-%02d_T%02d-%02d-%02d_%03d.wav",fileCount, year, month, day, hour, minute, second, ms);
  dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
  while (!dataFile){
    fileCount += 1;
    sprintf(filename,"F%06d.wav",fileCount); //if can't open just use count
    dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
  }
  dataFile.write((uint8_t *)&wav_hdr, 44);
}


void watermark(){
  // wake up
  // digitalWrite(ledGreen, ledGreen_ON);
}

//****************************************************************  
// set system into the sleep state 
// system wakes up when interrupt detected
void system_sleep() {

  __WFI(); //Wait for interrupt
  //detachInterrupt(digitalPinToInterrupt(INT2));
}

//This function returns the date and time for SD card file access and modify time. One needs to call in setup() to register this callback function: SdFile::dateTimeCallback(file_date_time);
void file_date_time(uint16_t* date, uint16_t* time) 
{
    *date=FAT_DATE(year,month,day);
    *time=FAT_TIME(hour,minute,second);
}

void alarmMatch(){
  // just wake up tag
}
