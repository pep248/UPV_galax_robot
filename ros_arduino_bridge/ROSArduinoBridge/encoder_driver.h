/**************************************************************
  Encoder driver function definitions - by James Nugen
  ************************************************************ */
#ifdef ARDUINO_ENC_COUNTER
  #include "RotaryEncoder.h"
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A 2  //pin 2
  #define LEFT_ENC_PIN_B 3  //pin 3

  //below can be changed, but should be PORTD pins
  #define RIGHT_ENC_PIN_A 18  //pin 18
  #define RIGHT_ENC_PIN_B 19  //pin 19

  RotaryEncoder encoderLeft(LEFT_ENC_PIN_A, LEFT_ENC_PIN_B); // Left encoder pins
  RotaryEncoder encoderRight(RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B); // Right encoder pins
#endif
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

