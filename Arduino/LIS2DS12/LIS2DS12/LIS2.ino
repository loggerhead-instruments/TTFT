int lis2Address = 0x1D; // (SA0 to +) 7-bit address 0x1D 0011101b ; 8-bit address 0x3A
                        // (SA0 to -) 7-bit address 0x1E 0011110b; 8-bit address 0x3C

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


void lis2Init(){

  // CTRL 1
  // ODR[3:0]
  // FS[1:0]  full-scale selection. 00 +/-2g; 01 +/-16g; 10 +/-4g; 11 +/- 8g
  // HF_ODR: High-frequency ODR mode enable; default = 0
  // BDU: Block data update. default=0; 0: continuous; 1: output registers not updated until MSB and LSB read)





  // 1600Hz: 01011010 (0x5A)
  // 200Hz:  01011000 (0x58)
  // ODR: 0101
  // FS: 10 (+/- 4g)
  // HF_ODR: 1
  // BDU: 0
  //writeI2C(lis2Address, LIS_CTRL1, 0x58);

  //writeI2C(lis2Address, LIS_CTRL4, 0x01); 

  Wire.beginTransmission(lis2Address);
  Wire.write(LIS_CTRL1);
  Wire.write(0x5A);
  Wire.endTransmission();

  // Continuous FIFO 11001000 0xC8 (module result to FIFO)
  // Continuous FIFO 11000000 0xC0 (X,Y,Z to FIFO) 
  // FIFO mode 00100000 0x20 (X,Y,Z to FIFO) 
  // Continuous mode: 110
  // Int2_Step_count_Ov: 0
  // Module_to_FIFO: 1 (module routine result is sent to FIFO instead of X,Y,Z)
  // RESVD: 00
  // IF_CS_PU_DIS: 0
  //writeI2C(lis2Address, LIS_FIFO_CTRL, 0xC0);
  
  Wire.beginTransmission(lis2Address);
  Wire.write(LIS_FIFO_CTRL);
  Wire.write(0x20);
  Wire.endTransmission();

 Wire.beginTransmission(lis2Address);
  Wire.write(LIS_FIFO_THS);  //FIFO threshold
  Wire.write(0x80); // 0x80 = 128
  Wire.endTransmission();
  
//  Wire.beginTransmission(lis2Address);
//  Wire.write(LIS_CTRL4);
//  Wire.write(0x01);
//  Wire.endTransmission();
  
}

int lis2TestResponse(){
//  // WHO_AM_I should return 0x43 (67 decimal)
//  byte response;
//  i2c_start(lis2Address);
//  i2c_write(LIS_WHO_AM_I);
//
//  i2c_rep_start(lis2Address | 1); //or'd with 1 for read bit
//  response = i2c_read(true);
//  i2c_stop();
//  return response;

  byte response;
  Wire.beginTransmission(lis2Address);
  Wire.write(LIS_WHO_AM_I);
  Wire.endTransmission();
  Wire.requestFrom(lis2Address, 1); //request 1 bytes
  response = Wire.read();
  return response;
}

void lis2FifoRead(){
//  byte val[6];
//  int i;
//  i2c_start(lis2Address);
//  i2c_write(LIS_OUT_X);
//  i2c_rep_start(lis2Address | 1); //or'd with 1 for read bit
//  for(i=0; i<5; i++){
//    val[i] = i2c_read(false);
//  }
//  val[i] = i2c_read(true);
//  i2c_stop();
//  for(i=0; i<6; i++){
//    accelX = val[i+1]<<8 | val[i];
//    accelY = val[i+3]<<8 | val[i+2];
//    accelZ = val[i+5]<<8 | val[i+4];
//  }

  byte val[6];
  int i;
  Wire.beginTransmission(lis2Address);
  Wire.write(LIS_OUT_X);
  Wire.endTransmission();
  Wire.requestFrom(lis2Address, 6);
  while(Wire.available()){
    val[i] = Wire.read();
    i++;
  }

  if(i!=6) digitalWrite(LED_RED, HIGH);
  if(i==6) digitalWrite(LED_RED, LOW);

  accelX = ((int) val[1]<<8) | val[0];
  accelY = ((int) val[3]<<8) | val[2];
  accelZ = ((int) val[5]<<8) | val[4];
}

int lis2FifoPts(){
//  byte val1, val2;
//  i2c_start(lis2Address);
//  i2c_write(LIS_FIFO_SRC);
//  i2c_rep_start(lis2Address | 1); //or'd with 1 for read bit
//  val1 = i2c_read(false);
//  val2 = i2c_read(false);
//  i2c_stop();
//
//  return (val1<<8 | val2);
    byte val1, val2;
  Wire.beginTransmission(lis2Address);
  Wire.write(LIS_FIFO_SRC);
  Wire.endTransmission();
  Wire.requestFrom(lis2Address, 2); 
  val1 = Wire.read();
  val2 = Wire.read();

  return (val2);
}

//void writeI2C(int devAddress, int registerAddress, int value){
//  i2c_start(devAddress); //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
//  i2c_write(registerAddress);
//  i2c_write(value);
//  i2c_stop();
//  __asm__ __volatile__ ("nop\n\t"); // very short delay
//}
