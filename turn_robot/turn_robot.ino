/****************************************************************
*                    Arduino MD25 example code                  *
*                   MD25 running in serial mode                 *
*                                                               *
*                     by James Henderson 2012                   *
*****************************************************************/

#include <SoftwareSerial.h>

// Values of 0 being sent using serial.write() have to be cast as a byte to stop them being misinterpereted as NULL
// This is a bug with arduino 1
#define CMD                 (byte)0x00              //  MD25 command byte of 0

#define WRITESP1            0x31                    // Byte writes speed to motor 1
#define WRITEACCEL          0x33                    // Byte writes a value of acceleration
#define RESETREG            0x35                    // Byte resets encoders
#define SETMODE             0x34                    // Byte sets mode
#define READIVS             0x2C                    // Byte reads motor currents and battery voltage        
#define READENCS            0x25                    // Byte reads both encoders
#define GET_VER             0x29
#define TURN                0x32                    // Byte writes speed to motor 2 <<<------------------------- This was added.

#define LCD_RX              0x02                    // RX and TX pins used for LCD0303 serial port
#define LCD_TX              0x03
#define LCD03_HIDE_CUR      0x04
#define LCD03_CLEAR         0x0C
#define LCD03_SET_CUR       0x02
#define s 1000

SoftwareSerial lcd03 = SoftwareSerial(LCD_RX, LCD_TX);          // Define the serial port for the LCD03

long encValue = 0;
byte softwareRev = 0;

void setup(){
  Serial.begin(38400);
    
  Serial.write(CMD);                                            // Set MD25 accelleration value
  Serial.write(WRITEACCEL);
  Serial.write(10);
  delayMicroseconds(10);                                        // Wait for this to be processed
  Serial.write(CMD);                                            // Reset the encoder registers to 0
  Serial.write(RESETREG);         
  Serial.write(CMD);                                            // Set mode to 2, Both motors controlled by writing to speed 1
  Serial.write(SETMODE);
  Serial.write(2);    
  
  Serial.write(CMD);                                            // Get software version of MD25
  Serial.write(GET_VER);
  while(Serial.available() < 1){}                               // Wait for byte to become available         
  softwareRev = Serial.read();  

}

void loop(){ 
  while(encValue < 180){               // While encoder 1 value less than 3000 move forward
    
    Serial.write(CMD);                  // Set motors to drive forward at full speed
    Serial.write(WRITESP1);
    Serial.write(200);
    encValue = readEncoder();
    
  }
  
  delay(1 * s);
  
  while(encValue > 0){
    
    Serial.write(CMD);                  // Drive motors reverse at full speed
    Serial.write(WRITESP1);
    Serial.write(10);
    encValue = readEncoder();

  }

  delay(1 * s);
    
//  Turn robot

  Serial.write(CMD); // Set motors to drive forward at full speed
  Serial.write(WRITESP1); 
  Serial.write(1);
  Serial.write(CMD);  //<---- This was missing
  Serial.write(TURN); 
  Serial.write(40);
    
 delay(3 * s);
 
 }


 long readEncoder(){                        // Function to read and display the value of both encoders, returns value of first encoder
  long result1 = 0; 
  long result2 = 0;
  Serial.write(CMD);
  Serial.write(READENCS);
  while(Serial.available() < 8){}          // Wait for 8 bytes, first 4 encoder 1 values second 4 encoder 2 values 
  result1 = Serial.read();                 // First byte for encoder 1, HH.
  result1 <<= 8;
  result1 += Serial.read();                // Second byte for encoder 1, HL
  result1 <<= 8;
  result1 += Serial.read();                // Third byte for encoder 1, LH
  result1 <<= 8;
  result1  += Serial.read();               // Fourth byte for encoder 1, LL
  result2 = Serial.read();
  result2 <<= 8;
  result2 += Serial.read();
  result2 <<= 8;
  result2 += Serial.read();
  result2 <<= 8;
  result2 += Serial.read();

  return result1;          
                           
}

