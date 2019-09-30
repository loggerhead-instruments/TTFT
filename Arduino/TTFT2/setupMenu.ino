// Menu for when TTFT2 connected via Serial Interface
#define PACKET 1024

void setupMenu(){
  displayMenu();

  while(1){
    digitalWrite(ledGreen,ledGreen_ON);
    if(SerialUSB.available()){
      int inByte = SerialUSB.read();
   //   SerialUSB.print("Selection:"); SerialUSB.println(inByte);
      switch (inByte){
        case 49:  //list files
          listFiles();
          clearSerial();
          break;
        case 50: // download files
          downloadFiles();
          clearSerial();
          break;
        case 51: // set time
          serialSetTime();
          clearSerial();
          break;
        case 52: // sensor Test
          sensorInit();
          clearSerial();
          break;
        case 53: // delete Files
          deleteFiles();
          clearSerial();
          break;
        case 54: // start
          SerialUSB.println("Starting");
          getTime();
          clearSerial();
          return;  
      }
      inByte = 0;
   //   displayMenu();
    }
  }
}

void displayMenu(){
  SerialUSB.println();
  SerialUSB.print("TTFT2: ");
  printChipId();
  getTime();
  printTime();
  SerialUSB.println("(1) List files");
  SerialUSB.println("(2) Download File");
  SerialUSB.println("(3) Set time");
  SerialUSB.println("(4) Sensor test");
  SerialUSB.println("(5) Delete all files");
  SerialUSB.println("(6) Start");
}

void printChipId() {
  volatile uint32_t val1, val2, val3, val4;
  volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
  val1 = *ptr1;
  volatile uint32_t *ptr = (volatile uint32_t *)0x0080A040;
  val2 = *ptr;
  ptr++;
  val3 = *ptr;
  ptr++;
  val4 = *ptr;
  char buf[42];
  sprintf(buf, "TTFT2 %8x%8x%8x%8x", val1, val2, val3, val4);
  SerialUSB.println(buf);
}

void listFiles(){
  sd.chdir(); // change to root
  printChipId();
  printDirectory(false);
}

void deleteFiles(){
  SerialUSB.println("Press Delete All again to delete. Any other button to cancel.");
  while(1){
    if(SerialUSB.available()){
      char inChar = SerialUSB.read();
      if(inChar != '5') {
        SerialUSB.println("Canceling delete");
        return;
      }
      else break;
    }
  }
  SerialUSB.println("Deleting Files");
  SdFile file;
  char filename[40];
  printDirectory(true);
  SerialUSB.println("Done deleting");
}
void serialSetTime(){
  // YYMMDDHHMMSS
  char inputStr[2];
//  SerialUSB.println("Enter Datetime (YYMMDDHHMMSS)");
//  SerialUSB.flush();
//  clearSerial();
  char datetime[12];
  int i = 0;
  long startTime = millis();
   while(1){
    if(SerialUSB.available()){
      char inChar = SerialUSB.read();
      datetime[i] = inChar;
      i++;
      if(i>=12) break;
      if(millis()-startTime > 10000) {
        SerialUSB.println("Set time fail");
        SerialUSB.flush(); // clear buffer
        return; // timeout
      }
    }
  }
  inputStr[0] = datetime[0]; inputStr[1] = datetime[1]; year = atoi(inputStr);
  inputStr[0] = datetime[2]; inputStr[1] = datetime[3]; month = atoi(inputStr);
  inputStr[0] = datetime[4]; inputStr[1] = datetime[5]; day = atoi(inputStr);
  inputStr[0] = datetime[6]; inputStr[1] = datetime[7]; hour = atoi(inputStr);
  inputStr[0] = datetime[8]; inputStr[1] = datetime[9]; minute = atoi(inputStr);
  inputStr[0] = datetime[10]; inputStr[1] = datetime[11]; second = atoi(inputStr);
  rtc.setTime(hour, minute, second);
  rtc.setDate(day, month, year);
  SerialUSB.println("New Time: "); printTime();
}
//170202142700


void printDirectory(int deleteFile) {
  SdFile file;
  char myFileName[40];
  sd.vwd()->rewind(); 
  while (file.openNext(sd.vwd(), O_READ)) {
    if(!file.isDir()){
      file.printName(&SerialUSB);
      SerialUSB.print(',');
      file.printFileSize(&SerialUSB);
      SerialUSB.println();
      file.getName(myFileName, 40);
      file.close();
      if(deleteFile){
        if(!sd.remove(myFileName)) SerialUSB.print(" fail");
      }
    }
    else{
      file.close();
    }
  }
}

void printTime(){
  getTime();
  SerialUSB.print(year); SerialUSB.print("-");
  SerialUSB.print(month); SerialUSB.print("-");
  SerialUSB.print(day); SerialUSB.print(" ");
  SerialUSB.print(hour); SerialUSB.print(":");
  SerialUSB.print(minute); SerialUSB.print(":");
  SerialUSB.println(second); 
}

void clearSerial(){
  while(SerialUSB.available()){
      int inByte = SerialUSB.read();
  }
}


void downloadFiles(){
//  SerialUSB.println("Enter Filename ");
//  SerialUSB.flush();
  int inByte = 100;
  char filename[15];
  memset(filename, 0, 15);
  int index = 0;
  long startTime = millis();

  while(1){ 
    if(SerialUSB.available()){
      inByte =  SerialUSB.read();
      SerialUSB.write(inByte);
      filename[index] = inByte;
      index++;
      if(inByte == 'V' | inByte == 'X') break;
    }
    if(millis() - startTime > 5000) break;
  }
  
  clearSerial();

  File file;
  if(file = sd.open(filename)){
    sendFile(&file);
    file.close();
  }
  else{
    SerialUSB.print("File open fail: ");
    SerialUSB.println(filename);
  }
}

void sendFile(File *file){
  // send data 1024 bytes at a time
  byte inByte;
  byte data[PACKET];
  int bytes2write;
  byte header;
  long bytesAvail = file->available();
  int getNextPacket = 1;
  long startTime;
  
  while(1){
    digitalWrite(ledGreen,ledGreen_OFF);
    bytesAvail = file->available();
    startTime = millis();
    // wait for a 'C'
    while(inByte!='C'){
      if(SerialUSB.available()){
          inByte =  SerialUSB.read();
      }
      if(millis()-startTime > 10000) return; // timeout
    }

    header = 1;
    // read 1 packet from file if first packet or received ACK
    if(getNextPacket){
      file->read(data, PACKET);
      bytes2write = PACKET;
      if(bytesAvail < PACKET) {
        bytes2write = bytesAvail;
        header = 4;
      }
    }

    if(!file->available()) header = 4; // no more data in file; last packet
    
    // send header byte
    SerialUSB.write(header);
    // send packet
    SerialUSB.write(data, bytes2write);
    SerialUSB.flush();
    
    // wait for ACK (6) or NACK (21)
    startTime = millis();
    while(1){
      if(millis()-startTime > 10000) return; // timeout
      digitalWrite(ledGreen, ledGreen_ON);
      if(SerialUSB.available()){
          inByte = SerialUSB.read();
          digitalWrite(ledGreen, ledGreen_OFF);
          if(inByte == 6) {
            getNextPacket = 1;
            break;
          }
          if(inByte == 21) {
            getNextPacket = 0;
            break;
          }
      }
    }
    if((inByte==6) & (header==4)) break; // got ACK on last bit
  }

  digitalWrite(ledGreen,ledGreen_OFF);
}
