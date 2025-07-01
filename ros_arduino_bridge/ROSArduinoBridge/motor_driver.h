/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define LEFT_MOTOR_FORWARD 24     // IN1
  #define LEFT_MOTOR_BACKWARD 26    // IN2  
  #define LEFT_MOTOR_ENABLE 7       // ENA
  #define RIGHT_MOTOR_FORWARD 13    // IN3
  #define RIGHT_MOTOR_BACKWARD 12   // IN4
  #define RIGHT_MOTOR_ENABLE 11     // ENB
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
