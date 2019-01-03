/**
 * This example synchronizes the actions of two motors by using
 * SPI daisy chaining.
 *
 * The hardware setup is:
 *   MOSI from controller tied to SDI on the first device
 *   SDO of the first device is tied to SDI of the next device
 *   SDO of the last device is tied to MISO of the controller
 *   all devices share the same SCK, SS_PIN and RESET_PIN
 *
 * Each L6470 passes the data it saw on its SDI to its neighbor
 * on the NEXT SPI cycle (8 bit delay).
 *
 * Each L6470 acts on the last SPI data it saw when the SS_PIN goes high.
 */

/**
 * Two different SPI routines are used.  One routine is used to send
 * commands to individual devices.  A different one is used for motion
 * commands so that all devices act at the same time.
 */

/**
 * The array "L6470_chain[]" is used to tell the software how the hardware is hooked up
 *   [0] - number of drivers in chain
 *   [1] - axis index for first device in the chain (closest to MOSI)
 *   [2] - axis index for next device in the chain
 *
 *  Axis index is an arbitrary identifier assigned by the user
 */


#include <L6470.h>

#define SS_PIN     10
#define SCK_PIN    11
#define MOSI_PIN   12
#define MISO_PIN   13
#define RESET_PIN  14




 byte L6470_SpiTransfer_Mode_3(byte b) { // using Mode 3
    int bits = 8;
    do {

      digitalWrite(SCK_PIN, LOW);
      digitalWrite(MOSI_PIN, b & 0x80);


      //DELAY_NS(125);

      digitalWrite(SCK_PIN, HIGH);
      b <<= 1;        // little setup time

      b |= (digitalRead(MISO_PIN) != 0);

    } while (--bits);

    //DELAY_NS(125);
    return b;
  }


/**
 * This is the routine involved in all non-motion commands/transfers
 *
 * This routine sends/receives one byte to the target device.  All other
 * Devices are sent the NOOP command.
 *
 * Note that the data for the last device in the chain is sent out first.
 *
 * The library will automatically link to "byte L6470_Transfer(byte,int,byte)"
 */

  byte L6470_Transfer(byte data, int _SSPin, byte chain_position) {
    #define CMD_NOP 0
    byte data_out = 0;
    data--;
    // first device in chain has data sent last
    digitalWrite(_SSPin, LOW);

    for (byte i = L6470_chain[0]; i >= 1; i--) {
      byte temp = L6470_SpiTransfer_Mode_3(byte (i == chain_position ? data : CMD_NOP));
      if (L6470_chain[i] == chain_position) data_out = temp;
    }

    digitalWrite(_SSPin, HIGH);
    return data_out;

  }

/**
 * This is the routine that sends the motion commands.
 *
 * This routine sends a buffer of data that is filled by the application.  The
 * library is not involved with it.
 */

//byte buffer[number of steppers + 1];
  // [0] - not used
  // [1] - command for first device
  // [2] - command for second device

void Buffer_Transfer(byte buffer[] , byte length) {
    // first device in chain has data sent last
    digitalWrite(SS_PIN, LOW);
    for (byte i = length; i >= 1; i--)
      buffer[i] = L6470_SpiTransfer_Mode_3(byte (buffer[i]));
    digitalWrite(SS_PIN, HIGH);
}


/**
 * Initialize pins for non-library SPI software
 *
 * The library will automatically link to "void L6470_SPI_init()"
*/

void L6470_SPI_init() {

  pinMode(SS_PIN,   OUTPUT);
  pinMode(SCK_PIN,  OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  digitalWrite(SS_PIN,   HIGH);
  digitalWrite(SCK_PIN,  HIGH);
  digitalWrite(MOSI_PIN, HIGH);
  pinMode(MISO_PIN, INPUT);

}

void goTo(long location_1, long location_2) {
  // the command string to move a stepper to an absolute position is
  // four byte long so four arraya are used for convenience

  byte buffer_command[3] = {dSPIN_GOTO, dSPIN_GOTO};  // create and fill buffer with command bytes

  if (location_1 > 0x3FFFFF) location_1 = 0x3FFFFF;  // limit to 22 bits
  if (location_1 > 0x3FFFFF) location_1 = 0x3FFFFF;

  byte addr21_16[3] = {0, (byte)(location_1 >> 16) , (byte)(location_2 >> 16)};
  byte addr15_8[3]  = {0, (byte)(location_1 >>  8) , (byte)(location_2 >>  8)};
  byte addr7_0[3]   = {0, (byte)(location_1)       , (byte)location_2};

  Buffer_Transfer( buffer_command, L6470_chain[0]);  // send the commands
  Buffer_Transfer( addr21_16     , L6470_chain[0]);  // send the MSB of the position
  Buffer_Transfer( addr15_8      , L6470_chain[0]);
  Buffer_Transfer( addr7_0       , L6470_chain[0]);  //  this one results in the motors moving
}


//////////////////////////////////////////////////////////////////////




L6470 stepperA(SS_PIN);           // create first stepper object
L6470 stepperB(SS_PIN);           // create second stepper object

void setup(){

  pinMode(RESET_PIN,OUTPUT);        // reset all drivers
  digitalWrite(RESET_PIN, LOW);     // do this before any setup commands are sent to the drivers
  delay(10);
  digitalWrite(RESET_PIN, HIGH);

  stepperA.set_chain_info(56, 1);     // completely setup L6470_chain[] before
  stepperB.set_chain_info(56, 2);     // any SPI traffic is sent

  stepperA.init();
  stepperA.setAcc(100);              // Set acceleration
  stepperA.setMaxSpeed(800);
  stepperA.setMinSpeed(1);
  stepperA.setMicroSteps(2);         // 1,2,4,8,16,32,64 or 128
  stepperA.setThresholdSpeed(1000);
  stepperA.setOverCurrent(6000);     // Set overcurrent protection
  stepperA.setStallCurrent(3000);

  stepperB.init();
  stepperB.setAcc(100);              // Set acceleration
  stepperB.setMaxSpeed(800);
  stepperB.setMinSpeed(1);
  stepperB.setMicroSteps(2);         // 1,2,4,8,16,32,64 or 128
  stepperB.setThresholdSpeed(1000);
  stepperB.setOverCurrent(6000);     // Set overcurrent protection
  stepperB.setStallCurrent(3000);


  goTo(200,200);                     //  spin the motors at the same time

}

void loop(){
  while (stepperB.isBusy()) delay(10);
  goTo(-200,-200);
  while (stepperB.isBusy()) delay(10);
  goTo(2000,2000);
}
