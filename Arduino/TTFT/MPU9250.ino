int CompassAddress = 0x0C;  //0x0C internal compass on 9150
int GyroAddress = 0x69;

#define AKM_REG_WHOAMI      (0x00)

#define AKM_REG_ST1         (0x02)
#define AKM_REG_HXL         (0x03)
#define AKM_REG_ST2         (0x09)

#define AKM_REG_CNTL        (0x0A)
#define AKM_REG_ASTC        (0x0C)
#define AKM_REG_ASAX        (0x10)
#define AKM_REG_ASAY        (0x11)
#define AKM_REG_ASAZ        (0x12)

#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)
#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

#define AKM_BIT_SELF_TEST   (0x40)

#define SUPPORTS_AK89xx_HIGH_SENS   (0x10)
#define AKM_POWER_DOWN          (0x00 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_SINGLE_MEASUREMENT  (0x01 | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_FUSE_ROM_ACCESS     (0x0F | SUPPORTS_AK89xx_HIGH_SENS)
#define AKM_MODE_SELF_TEST      (0x08 | SUPPORTS_AK89xx_HIGH_SENS)

#define AKM_WHOAMI      (0x48)

#define BIT_I2C_READ        (0x80)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_MST_VDDIO   (0x80)

#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38

int mpuInit(boolean mode)
{
  int ecode;
   if (printDiags) Serial.print("MPU Init\n");
   if(mode==0)
  {
     ecode = I2Cwrite(GyroAddress, 0x6B, 0x40);  //Sleep mode, internal 8 MHz oscillator  //another mode is cycle where it wakes up periodically to take a value
     return ecode;
  }

    //set clock source
    ecode = I2Cwrite(GyroAddress, 0x6B, 0x01);  //everything awake; autosource clock

    I2Cwrite(GyroAddress, 0x6C, 0x07); // disable gyroscopes
  
    // configure frame sync and LPF
    I2Cwrite(GyroAddress, 0x1A, 0x00);  //FIFO roll when full
    
    // set gyro range
    I2Cwrite(GyroAddress, 0x1B, 0x00);  // 0x10 +/- 1000 deg/s ; 0x18 +/-2000 deg/s Fchoice_b = 00 (use DLPF)
    
    // set sample rate divider
    I2Cwrite(GyroAddress, 0x19, 0x00);  //  0x31=49=>20Hz; 1kHz/(1+4)=200; divide 1 kHz/(1+9)=100 Hz sample rate for all sensors

    // set accel range
    I2Cwrite(GyroAddress, 0x1C, 0x00); // 0x00 =  +/- 2 g  DEFAULT
    if (accel_scale == 4) I2Cwrite(GyroAddress, 0x1C, 0x08); // 4g
    if (accel_scale == 8) I2Cwrite(GyroAddress, 0x1C, 0x10); // 8g
    if (accel_scale == 16) I2Cwrite(GyroAddress, 0x1C, 0x18); // 16g
  
    // Accelerometer Configuration 2
    I2Cwrite(GyroAddress, 0x1D, 0x0F); // 0x08 low pass filter at 1046; 4 kHz sample rate

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
    // until interrupt cleared, clear on read of INT_STATUS, and enable
    // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
    // controlled by the Arduino as master.
     I2Cwrite(GyroAddress, INT_PIN_CFG, 0x22);
    // Enable data ready (bit 0) interrupt
     I2Cwrite(GyroAddress, INT_ENABLE, 0x01);

    I2Cwrite(GyroAddress, 0x23, 0x08); // write accelerometer only to FIFO
    I2Cwrite(GyroAddress, 0x6A, 0x07); // reset FIFO
    I2Cwrite(GyroAddress, 0x6A, 0x60); // FIFO enabled, Master Mode enabled

   return ecode;
}

void resetGyroFIFO(){
    I2Cwrite(GyroAddress, 0x6A, 0x07); // reset FIFO
    I2Cwrite(GyroAddress, 0x6A, 0x60); // FIFO enabled, Master Mode enabled
}

byte I2Cwrite(byte addr, byte reg, byte val)
{
  Wire.beginTransmission(addr);  
  Wire.write(reg);  // gyro scale, sample rate and LPF
  Wire.write(val);  
  byte ecode=Wire.endTransmission(); //end transmission
  if (printDiags) Serial.print(ecode);
  delay(5);
  return ecode;
}

void readImu()
{
  int i = 0;
  Wire.beginTransmission(GyroAddress); 
  Wire.write(0x3B);        //sends address to read from  0x3B is direct read; 0x74 is FIFO
  Wire.endTransmission(false); //send restart to keep connection alive
  Wire.requestFrom(GyroAddress, 20); //send restart to keep alive
  while(Wire.available()){
    imuTempBuffer[i] = Wire.read(); 
    i++;
  }
}

void readImuFifo(int nBytes){
  int chunkSize = 32;
  int i;
  for(int x = 0; x<nBytes/chunkSize; x++){
    i = 0;
    Wire.beginTransmission(GyroAddress); 
    Wire.write(0x74);        //sends address to read from  0x3B is direct read; 0x74 is FIFO
    Wire.endTransmission(false); //send restart to keep connection alive
    Wire.requestFrom(GyroAddress, chunkSize); //send restart to keep alive
    while(Wire.available()){
      imuTempBuffer[i] = Wire.read(); 
      i++;
    }
  }
}

int getImuFifoPts()
{
 int fifopts; 
  // read FIFO size
//  Wire.beginTransmission(GyroAddress); 
//  Wire.write(0x3B);        //sends address to read from
//  Wire.endTransmission(); //end transmission
    
  Wire.beginTransmission(GyroAddress); 
  Wire.write(0x72);        //sends address to read from
  Wire.endTransmission(false); //end transmission
  Wire.requestFrom(GyroAddress, 2);    // request 6 bytes from device
  
  byte FIFO_CNT[2];
  int i=0;
 
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    FIFO_CNT[i]= Wire.read();  // receive one byte
    i++;
  }
  fifopts=FIFO_CNT[0]<<8|FIFO_CNT[1];
   
 return fifopts; 
}

int intStatus(){
  byte intStatus; 
  Wire.beginTransmission(GyroAddress); 
  Wire.write(0x3A);        //sends address to read from
  Wire.endTransmission(false); //end transmission
  Wire.requestFrom(GyroAddress, 1);    // request 6 bytes from device
  
  byte FIFO_CNT[2];
  int i=0;
 
  if(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    intStatus = Wire.read();  // receive one byte
  }
 return intStatus; 
}

