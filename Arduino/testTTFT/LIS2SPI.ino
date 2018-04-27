// when storing magnitude of acceleraton watermark threshold are represented by 1Lsb = 3 samples
#define FIFO_WATERMARK (0x80) // 0x0C=12 0x24=36; 0x80 = 128

const byte SPI_READ = 0x80;
const byte SPI_WRITE = 0x00;

#define LIS_WHO_AM_I (0x0F)
#define LIS_CTRL1 (0x20)
#define LIS_CTRL2 (0x21)
#define LIS_CTRL3 (0x22)
#define LIS_CTRL4 (0x23)
#define LIS_CTRL5 (0x24)
#define LIS_FIFO_CTRL (0x25)
#define LIS_OUT_T (0x26)
#define LIS_STATUS (0x27)
#define LIS_OUT_X (0x28)
#define LIS_OUT_Y (0x2A)
#define LIS_OUT_Z (0x2C)
#define LIS_FIFO_THS (0x2E)
#define LIS_FIFO_SRC (0x2F)
#define LIS_FIFO_SAMPLES (0x30)
#define LIS_TAP_6D_THS (0x31)
#define LIS_INT_DUR (0x32)
#define LIS_WAKE_UP_THS (0x33)
#define LIS_WAKE_UP_DUR (0x34)
#define LIS_FREE_FALL (0x35)
#define LIS_STATUS_DUP (0x36)
#define LIS_WAKE_UP_SRC (0x37)
#define LIS_TAP_SRC (0x38)
#define LIS_6D_SRC (0x39)
#define LIS_STEP_COUNTER_MINTHS (0x3A)
#define LIS_STEP_COUNTER_L (0x3B)
#define LIS_STEP_COUNTER_H (0x3C)
#define LIS_FUNC_CK_GATE (0x3D)
#define LIS_FUNC_SRC (0x3E)
#define LIS_FUNC_CTRL (0x3F)


void lis2SpiInit(){
  // reset
  writeRegister(LIS_CTRL2, 0x40); // soft reset
  delay(100);

  // CTRL 1
  // ODR[3:0]
  // FS[1:0]  full-scale selection. 00 +/-2g; 01 +/-16g; 10 +/-4g; 11 +/- 8g
  // HF_ODR: High-frequency ODR mode enable; default = 0
  // BDU: Block data update. default=0; 0: continuous; 1: output registers not updated until MSB and LSB read)

  //         ODR  FS HF_ODR BDU
  // 1600Hz: 0101 10 1      0 (0x5A)
  // 800Hz:  0111 10 0      0 (0x78) 800 Hz, +/-4g, high frequency ODR disabled, block data update off
  // 800Hz:  0111 00 0      0 (0x70) 800 Hz, +/-2g, ODR disabled, block data off
  // 200Hz:  0101 10 0      0 (0x58)
  // 50 Hz:  0011 10 0      0 (0x38) 50 Hz, +/- 4g, ODR disabled, block data off
  // FS: 00 (=/-2g); 10 (+/- 4g); 01 (+/-16g); 11 (+/-8g)
  // HF_ODR: 1
  // BDU: 0

  writeRegister(LIS_CTRL1, 0x5A);
  delay(10);

  // Set FIFO watermark
  writeRegister(LIS_FIFO_THS, FIFO_WATERMARK);
  delay(10);

  // FIFO threshold interrupt is routed to INT1
  writeRegister(LIS_CTRL4, 0x02);
  delay(10);
  
  // Turn Module ON to calculate acceleration magnitude
  writeRegister(LIS_FUNC_CTRL, 0x20);//0x20 Module On
  delay(10);
  
  // Bypass mode 00001000 0x08 // bypass mode with magnitude module on
  // FIFO in bypass mode, module on
  writeRegister(LIS_FIFO_CTRL, 0x08);
  delay(10);
  
  // Continuous mode FIFO 11001000 0xC8 (module result to FIFO)
  // Continuous mode FIFO 11000000 0xC0 (X,Y,Z to FIFO) 

  // Continuous mode: 110
  // Int2_Step_count_Ov: 0
  // Module_to_FIFO: 1 (module routine result is sent to FIFO instead of X,Y,Z)
  // RESVD: 00
  // IF_CS_PU_DIS: 0
  //  writeI2C(lis2Address, LIS_FIFO_CTRL, 0xC0);
  writeRegister(LIS_FIFO_CTRL, 0xC8);
}

int lis2SpiTestResponse(){
  byte response;
  response = readRegister(LIS_WHO_AM_I, 1);
  return response;
}

void lis2SpiFifoRead(int bytesToRead){
/* It is recommended to read all FIFO slots in a multiple byte reading of 1536 bytes (6 output
registers by 256 slots). In order to minimize communication between the master and slave
the reading address may be automatically incremented by the device by setting the
IF_ADD_INC bit of CTRL2 register to ‘1’; the device rolls back to 0x28 when register 0x2D
is reached
*/
  digitalWrite(LED, HIGH);
  readRegister(LIS_OUT_X, bytesToRead);
  digitalWrite(LED, LOW); 
}

// note that when storing module (magnitude) nsamples is 3x value returned
int lis2SpiFifoPts(){
  byte val;
  val = readRegister(LIS_FIFO_SAMPLES, 1); // note ignoring that upper part stored in FIFO_SRC
  return (val);
}

int lis2SpiFifoStatus(){
  byte val1;
  val1 = readRegister(LIS_STATUS, 1);
  return(val1 & 0x80); // return 0 if less than threshold
}

//Read from or write to register from the SCP1000:
unsigned int readRegister(byte thisRegister, int bytesToRead) {
  byte inByte = 0;           // incoming byte from the SPI
  unsigned int result = 0;   // result to return
  byte dataToSend = thisRegister | SPI_READ;
  // take the chip select low to select the device:
  digitalWrite(chipSelectPinAccel, LOW);
  // send the device the register you want to read:
  SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);
  accel[0] = result;
  // decrement the number of bytes left to read:
  bytesToRead--;
  // if you still have another byte to read:
  int j = 0;
  if (bytesToRead > 0) {
    j++;
    accel[j] = SPI.transfer(0x00);
    // decrement the number of bytes left to read:
    bytesToRead--;
  }
  // take the chip select high to de-select:
  digitalWrite(chipSelectPinAccel, HIGH);
  // return the result:
  return (result);
}


//Sends a write command to SCP1000

void writeRegister(byte thisRegister, byte thisValue) {
  byte dataToSend = thisRegister | SPI_WRITE;

  // take the chip select low to select the device:
  digitalWrite(chipSelectPinAccel, LOW);

  SPI.transfer(thisRegister); //Send register location
  SPI.transfer(thisValue);  //Send value to record into register

  // take the chip select high to de-select:
  digitalWrite(chipSelectPinAccel, HIGH);
}
