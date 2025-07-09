/*
MCP_DAC 0.52
*/

#include "MCP_DAC.h" //reference the library files
MCP4921 dac1;       //create DAC object
MCP4921 dac2;       //create DAC object

uint32_t del_time = 1000;

uint32_t step1 = 100;
uint32_t start1 = 0;
uint32_t end1 = 4095;
uint32_t step2 = 5;
uint32_t start2 = 0;
uint32_t end2 = 4095;

void setup(){

SPI.begin();

dac1.begin(10);     //initialize
dac2.begin(9);     //initialize
}
 
void loop(){
  
  /*for (int j = start2; j<end2; j+=step2*2) {
    for (int i = start1; i<end1; i+=step1) {
      dac1.analogWrite(i);
    }
    dac2.analogWrite(j);
    for (int i = end1; i>start1; i-=step1) {
      dac1.analogWrite(i);
    }
    dac2.analogWrite(j+step2);
  }*/
  for (int i = start1; i<end1-step1; i+=step1) {
    dac1.write(i);
    delayMicroseconds(del_time);
  }
  for (int i = start1; i<end1-step1; i+=step1) {
    dac2.write(i);
    delayMicroseconds(del_time);
  }
  for (int i=end1; i> start1+step1; i-=step1) {
    dac1.write(i);
    delayMicroseconds(del_time);
  }
  for (int i=end1; i> start1+step1; i-=step1) {
    dac2.write(i);
    delayMicroseconds(del_time);
  }
}