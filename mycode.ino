#include <Wire.h>                                 // Library used for I2C communication with the Accelerometer
#include "MMA7660.h"                              // Library for the Accelerometer module
#include <LedControl.h>

#define ACCEL_THRESHOLD   10

int DIN = 5;
int CS = 7;
int CLK = 4;
LedControl lc=LedControl(DIN,CLK,CS,0);  //inicijalizacija led matrice

int Cat[8] ={B10001000,B11111000,B10101000,B01110001,B00100001,B01111001,B01111101,B10111110};

int arrow_r[8] ={B00011000,B00001100,B00000110,B11111111,B11111111,B00000110,B00001100,B00011000};
int arrow_l[8] ={B00011000,B00110000,B01100000,B11111111,B11111111,B01100000,B00110000,B00011000};




MMA7660 accelerometer; 

bool    moving = false;                           // Variable used to check if the device is moving
float   prevLatitude, prevLongitude;              // Used to save previous location in order to compare to current one
int8_t  prevX,prevY,prevZ;                        // Used to save previous X, Y and Z axis data in order to compare to current one
unsigned long sendNextAt = 0;                     // Variable used as a timer to check when the program is supposed to send location data when triggered
int where_x = 0;  //0 means left 1 means right;

void setup() {
  Serial.begin(9600);
  accelerometer.init();                           // Initialize the accelerometer module
  accelerometer.getXYZ(&prevX, &prevY, &prevZ);


    /*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,15);
  /* and clear the display */
  lc.clearDisplay(0);
  
}

void loop(){
  //moving = isAccelerating(); 
  //Serial.println(moving); 
if (!moving) {                                  // If not moving, check accelerometer
    moving = isAccelerating();                    // "moving" inherits true/false value from isAccelerating function
    
    
    //delay(500);
    
  }else{
      if(where_x){
          for(int i=0;i<8;i++) lc.setRow(0,i,arrow_l[i]);
          delay(500);
          lc.clearDisplay(0);
        }else{
            for(int i=0;i<8;i++) lc.setRow(0,i,arrow_r[i]);
            delay(500);
            lc.clearDisplay(0);
          }
      
      moving = false;
    }
//  accelerometer.getXYZ(&prevX, &prevY, &prevZ);
//  Serial.println(prevX);
//  Serial.println(prevY);
//  Serial.println(prevZ);
//  delay(100);
}









bool isAccelerating() {
  int8_t x,y,z;                                   // Create three variables where the current data will be stored
  accelerometer.getXYZ(&x, &y, &z);               // Read accelerometer data and save it to the variables create above
  // We now need a way to compare the previous reading with the current one. 
  // We'll do this by subtracting current value of X, Y and Z from their old values,
  // then we'll add the results of each one and check if the number is greater than
  // our set threshold (ACCEL_THRESHOLD). Since accelerometer module outputs numbers 
  // that can go negative, we'll use the built-in arduino function "abs" to calculate 
  // absolute values of these numbers.
  
  //bool result = (abs(prevX - x) + abs(prevY - y) + abs(prevZ - z)) > ACCEL_THRESHOLD;
  bool result = (abs(prevX - x)) > ACCEL_THRESHOLD;
  if(result == true) {                            // If motion is above threshold, save the current values as previous values for future comparison
    Serial.println("Accelerometer motion detected!");
//    Serial.println((prevX - x));
    if((prevX > x)){
        Serial.println("L");
        where_x = 0; 
      }else{
        Serial.println("R");
        where_x = 1;
        }
    prevX = x;
    prevY = y;
    prevZ = z;
  }
  return result;                                  // This whole function will return boolean true/false depending on whether motion is detected 
}
