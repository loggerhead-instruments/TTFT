/*
  sd card read/write

 This example shows how to read and write data to and from an sd card file
 The circuit:
 * sd card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */
#define sd_CS_PIN 10
#include <SPI.h>
//#include <sd.h>
#include "SdFat.h"
SdFat sd;

#define sdPOW PIN_LED_RXL

File myFile;

void setup()
{
  // Open serial communications and wait for port to open:
  SerialUSB.begin(9600);

  pinMode(sdPOW, OUTPUT);
  digitalWrite(sdPOW, HIGH); // turn on power to sd

    // Wait for USB SerialUSB 
  while (!SerialUSB) {
    SysCall::yield();
  }
  delay(1000);

  SerialUSB.print("Initializing sd card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the sd library functions will not work.
  pinMode(10, OUTPUT);

  if (!sd.begin(10, SPI_FULL_SPEED)) {
    SerialUSB.println("initialization failed!");
    return;
  }
  SerialUSB.println("initialization done.");

    // Discard any input.
  do {
    delay(10);
  } while (SerialUSB.available() && SerialUSB.read() >= 0);
  
  SerialUSB.println("Type any character to begin");
  while (!SerialUSB.available()) {
    SysCall::yield();
  }

  byte inputVal = SerialUSB.read();

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = sd.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    SerialUSB.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    SerialUSB.println("done.");
  } else {
    // if the file didn't open, print an error:
    SerialUSB.println("error opening test.txt");
  }

  // re-open the file for reading:
  myFile = sd.open("test.txt");
  if (myFile) {
    SerialUSB.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      SerialUSB.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    SerialUSB.println("error opening test.txt");
  }
}

void loop()
{
  // nothing happens after setup
}
