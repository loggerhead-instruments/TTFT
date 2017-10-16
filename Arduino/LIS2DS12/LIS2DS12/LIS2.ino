int lis2Address = 0x3A; // (SA0 to +) 7-bit address 0x1D 0011101b ; 8-bit address 0x3A
                        // (SA0 to -) 7-bit address 0x1E 0011110b; 8-bit address 0x3C

#define LIS_WHO_AM_I (0x0F)


void writeI2C(int devAddress, int registerAddress, int value){
  i2c_start(devAddress); //This needs the 8 bit address (7bit Device Address + RW bit... Read = 1, Write = 0)
  i2c_write(registerAddress);
  i2c_write(value);
  i2c_stop();
  __asm__ __volatile__ ("nop\n\t"); // very short delay
}

int lis2TestResponse(){
//  // WHO_AM_I should return 0x43 (67 decimal)
  byte response;
  i2c_start(lis2Address);
  i2c_write(LIS_WHO_AM_I);

  i2c_rep_start(lis2Address | 1); //or'd with 1 for read bit
  response = i2c_read(true);
  i2c_stop();
  return response;
}
