/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  // Serial initialization removed - this belongs in setup()
  // static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
    
  // /* Interrupt routine for LEFT encoder, taking care of actual counting */
  // ISR (PCINT2_vect){
  // 	static uint8_t enc_last=0;
        
	// enc_last <<=2; //shift previous state two places
	// enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
  
  // 	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  // }
  
  // /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  // ISR (PCINT1_vect){
  //       static uint8_t enc_last=0;
          	
	// enc_last <<=2; //shift previous state two places
	// enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
  
  // 	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  // }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT)
    {
      // encoderLeft.getRPM(); // Left encoder pins
      // Serial.print("Left encoder: ");
      // Serial.println(encoderLeft.getPosition());
      return encoderLeft.getPosition();
    }
    else
    {
      // encoderRightight.getPosition(); // Right encoder pins
      // Serial.print("Right encoder: ");
      // Serial.println(encoderRight.getPosition());
      return encoderRight.getPosition();
    }
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      Serial.println("Left encoder reset");
      encoderLeft.setPosition(0L);
      return;
    }
    else
    { 
      Serial.println("Right encoder reset");
      encoderRight.setPosition(0L);
    }
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif

