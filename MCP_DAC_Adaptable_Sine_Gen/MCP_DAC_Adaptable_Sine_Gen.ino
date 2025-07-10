/*____________SineGenNoPWM - DAC Version w/Trigger__________________________________
 * Modified for MCP4921 DAC control with trigger output
 *
 *  Arduino software for Spinning laser for TIRF systems or holotomography. Original Arduino program controls galvos
 * using TimerONE library for a faster PWN than the native Arduino PWM. Speed of one cycle
 * with delayMicroseconds set to zero is approx 1.68ms.
 *
 * Changes since original version:
 * - Replaced TimerOne PWM with MCP4921 DAC control
 * - Added digital trigger (pin 8) synchronized with DAC updates
 * - Scaled 8-bit values (0-255) to 12-bit DAC range (0-4095)
 * - Updated center position from 512 (10-bit) to 2048 (12-bit)
 *
 * SERIAL MONITOR COMMANDS
 * ** TYPE ONLY CONTENTS OF <> FOLLOWED BY RETURN KEY
 * ** #int# #float# refers to number of specified type
 * <p>  Normal Tirf mode, <q> to quit
 * <c>  Centred mode, <q> to quit
 * <s#int#q> Change the delay between loops, increasing will slow rotation. 0 by default
 * <x#float#q> Changes scaling in x direction to #float#, default 1
 * <y#float#q> Changes scaling in y direction to #float#, default 1
 * <>#int#q> Increases shift in x direction by #int#
 * <^#int#q> Increases shift in y direction by #int#
 */

#include <MCP_DAC.h>
MCP4921 dac1;         // DAC for Y-axis (formerly pinA)
MCP4921 dac2;         // DAC for X-axis (formerly pinB)
#define TRIGGER_PIN 8  // Digital trigger output pin

#define maxSamplesNum 100

//Variables for code
const byte dac1_CS = 10;  // Chip select for DAC1 (Y-axis)
const byte dac2_CS = 9;   // Chip select for DAC2 (X-axis)
byte index = 0;

// Original sine array (8-bit values)
static byte SineArray [maxSamplesNum] = {
      128,136,144,152,160,168,176,183,190,197,
      204,210,216,222,227,232,237,241,244,248,
      250,252,254,255,255,255,255,255,253,251,
      249,246,243,239,235,230,225,219,213,207,
      201,194,187,179,172,164,156,148,140,132,
      124,116,108,100,92,84,77,69,62,55,
      49,43,37,31,26,21,17,13,10,7,
      5,3,1,0,0,0,1,2,4,6,
      8,12,15,19,24,29,34,40,46,52,
      59,66,73,80,88,96,104,112,120,128};

// 12-bit value arrays for DACs
uint16_t XArray12[maxSamplesNum];
uint16_t YArray12[maxSamplesNum];

//Variable Pointers
float scaleX = 3;
float scaleY = 3;
int centreX = 2048;  // Scaled for 12-bit (512 * 4)
int centreY = 2048;  // Scaled for 12-bit (512 * 4)
int msDelay = 0;
int elipsis = 28; //default 25 This will change the elipticity of your circle
int counter = 0;

void setup() {
  SPI.begin();
  // Initialize DACs
  dac1.begin(dac1_CS);
  dac2.begin(dac2_CS);
  
  // Initialize trigger pin for TTL output
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  
  Serial.begin(9600);
  updateloop();  // Initialize DAC value arrays

  // Uncomment following section to check the values of the X and Y arrays
/*
  for (int i=0; i<maxSamplesNum; i++){
  Serial.println(XArray12[i]);
  }
  for (int i=0; i<maxSamplesNum; i++){
  Serial.println(YArray12[i]);
  } 
*/
}
  
void loop() {
  for(int i=0; i<maxSamplesNum; i++) {

    // Update both DACs
    dac1.write(YArray12[i]);
    dac2.write(XArray12[i]);

    // Generate TTL trigger pulse
    digitalWrite(TRIGGER_PIN, HIGH);  
    delayMicroseconds(12);             // Maintain pulse width
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(6); // Short delay to ensure trigger pulse is registered

    delayMicroseconds(msDelay);
    
    // Check for serial commands after completing a full cycle
    if (i == maxSamplesNum - 1) {
      while(Serial.available()) {
        readSerial(&scaleX, &scaleY, &msDelay, &centreX, &centreY);
      }
    }
  }

// Uncomment following section to print system time per 1000 cycles in us
// Uncommenting this WILL pause your code slightly while printing
/*
   counter = counter + 1;
    if (counter>999){
      Serial.println(micros());
      counter = 0;
    }
*/
}

void updateloop() {
  // Update arrays with current scaling and center values
  for (int i=0; i<maxSamplesNum; i++) {
    // Calculate Y value (12-bit scaled)
    float tempY = (SineArray[i] - 128) * scaleY * 4 + centreY;
    tempY = constrain(tempY, 0, 4095);  // Limit to DAC range
    YArray12[i] = static_cast<uint16_t>(tempY);
    
    // Calculate X value (12-bit scaled)
    index = i + elipsis;  
    if (index >= maxSamplesNum) index -= maxSamplesNum;
    float tempX = (SineArray[index] - 128) * scaleX * 4 + centreX;
    tempX = constrain(tempX, 0, 4095);  // Limit to DAC range
    XArray12[i] = static_cast<uint16_t>(tempX);
  } 
}

void readSerial(float *scalex, float *scaley, int *msdelay, int *centreX, int *centreY) {
  char store = 'n';
  String str = "";
  float num = 0;
  
  while (Serial.available()) {
    char nextChar = Serial.read();
    if (nextChar == 'c')  {
      // Set center position (12-bit values)
      dac1.write(*centreY);
      dac2.write(*centreX);

      digitalWrite(TRIGGER_PIN, HIGH);             // Rising edge
      delayMicroseconds(12);             // Maintain pulse width
      digitalWrite(TRIGGER_PIN, LOW);              // Falling edge
      delayMicroseconds(6); // Short delay to ensure trigger pulse is registered

      Serial.println("Laser at Center, press q to quit");

      char quit = ' ';
      while (1==1){
        if (Serial.available()) {
          quit = Serial.read();
          if (quit == 'q')
            return;
        }
      }   
    }
    if (nextChar == 'p')  {
      // Calculate normal TIRF position (12-bit)
      float yVal = *centreY;  // (128-128) term zeroed
      float xVal = (255 - 128) * (*scalex) * 4 + *centreX;
      
      dac1.write(static_cast<uint16_t>(constrain(yVal, 0, 4095)));
      dac2.write(static_cast<uint16_t>(constrain(xVal, 0, 4095)));

      digitalWrite(TRIGGER_PIN, HIGH);             // Rising edge
      delayMicroseconds(12);             // Maintain pulse width
      digitalWrite(TRIGGER_PIN, LOW);              // Falling edge
      delayMicroseconds(6); // Short delay to ensure trigger pulse is registered

      Serial.println("Laser at normal TIRF, press q to quit");
 
      char quit = ' ';
      while (1==1){
        if (Serial.available()) {
          quit = Serial.read();
          if (quit == 'q')
            return;
        }
      }   
    }
    else if (nextChar == 'x'){
      Serial.println("Scaling in X");
      str = "";
      while (1==1){
        if (Serial.available()) {
          store = Serial.read();
          if (isDigit(store) || store == '.') {
             str += store;
          }
          else if (store == 'q')  {
            *scalex = str.toFloat();
            updateloop();
            return;
          }
        }
      }
    }
    else if (nextChar == 'y') {
      Serial.println("Scaling in y");
      str = "";
      while (1==1){
        if (Serial.available()) {
          store = Serial.read();
          if (isDigit(store) || store == '.') {
             str += store;
          }
          else if (store == 'q')  {
            *scaley = str.toFloat();
            updateloop();
            return;
          }
        }
      }
    }
    else if (nextChar == 's') {
      Serial.println("Changing ms delay");
       str = "";
      while (1==1){
        if (Serial.available()) {
          store = Serial.read();
          if (isDigit(store)) {
             str += store;
          }
          else if (store == 'q')  {
            *msdelay = str.toInt();
            return;
          }
        }
      }
    }
    else if (nextChar == '^') {
      Serial.println("Shifting centre Y");
       str = "";
      while (1==1){
        if (Serial.available()) {
          store = Serial.read();
          if (isDigit(store) || store == '-' || store == '.') {
             str += store;
          }
          else if (store == 'q')  {
            *centreY = *centreY + str.toInt() * 4;  // Scale shift for 12-bit
            updateloop();
            return;
          }
        }
      }
    }
    else if (nextChar == '>') {
      Serial.println("Shifting centre X");
       str = "";
      while (1==1){
        if (Serial.available()) {
          store = Serial.read();
          if (isDigit(store) || store == '-' || store == '.') {
             str += store;
          }
          else if (store == 'q')  {
            *centreX = *centreX + str.toInt() * 4;  // Scale shift for 12-bit
            updateloop();
            return;
          }
        }
      }
    }
    else{
      Serial.print("Unknown command: ");
      Serial.println(nextChar);
      return;
    }
  }
}

/* From original Revision Log [To deprecate on next commit]
 * V1 17-Aug-2015
 *  New, includes code to count time for 1000 cycles
 * V2 18-Aug-2015
 *  Serial Communication to control speed, xy scaling and centrepoint of Arduino
 * V3 6-Oct-2016
 *  Circles scale from a fixed centre point rather than from the top left corner.  
 * V4 20-Mar-2017
 *    Flipped pinouts for pinA and pinB because galvos are flipped upside down
 *    Laser is at centre position rather than top left when stopped
 *    Changed default scale to 1:1
 *    Add function to add shift centreX centreY from Serial Monitor
 * V5 20-Mar-2017
 *  Removed digit in speed input
 *  Included functionality to change to normal TIRF
 * V6 04-Apr-2017
 *  Moved serial checking to once per circle vs. once per point to increase speed of spinning
 * V6 Speed Edit
 *  Updated entire archetecture. ReadSerial now updates arrays XArray and YArray, scan speed of 
 *  code is greatly increased. From 7.61ms in V6 to 1.469ms in V6 Speed Edit
 * 
 * MCP DAC Version [To deprecate on next commit]:
 *  - Replaced PWM with MCP4921 DAC control
 *  - Added digital trigger on pin 8
 *  - Scaled values for 12-bit output (0-4095)
 *  - Shift commands now scaled by 4x for 12-bit resolution
 */