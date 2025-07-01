float Diametroa = 0;  // Diameter in cm
float Ppr = 0;        // Pulses Per Revolution
float Cpr = 0;        // Counts Per Revolution
float StartDuty = 0;  // Initial Duty Cycle PWM (0.0 - 1.0)

void setMotorParameters(int motorType) {
  switch (motorType) {
    case 1: case 2: case 3: case 4: case 5:  // Motor: JGA25-370
      Diametroa = 6.7;          // Diameter in cm
      Ppr = 11 * 226;           // Pulses Per Revolution
      Cpr = Ppr * 4;            // Counts Per Revolution
      StartDuty = 0.4;          // Initial Duty Cycle PWM
      break;

    case 6:  // Motor: am-3103 NeveRest 60
      Diametroa = 6.7;          // Diameter in cm
      Ppr = 7 * 60;             // Pulses Per Revolution
      Cpr = Ppr * 4;            // Counts Per Revolution
      StartDuty = 0.2;          // Initial Duty Cycle PWM
      break;

    case 7:  // Motor: GM9236E349-R1 / Pioneer 3
      Diametroa = 18.9;         // Diameter in cm
      Ppr = 500.0 / 50 * 38.3;  // Pulses Per Revolution
      Cpr = Ppr * 4;            // Counts Per Revolution
      StartDuty = 0.2;          // Initial Duty Cycle PWM
      break;

    case 8:  // Motor: GM9236E376 / Pioneer 2
      Diametroa = 18.9;         // Diameter in cm
      Ppr = 500.0 / 50 * 19.7;  // Pulses Per Revolution
      Cpr = Ppr * 4;            // Counts Per Revolution
      StartDuty = 0.25;         // Initial Duty Cycle PWM
      break;

    case 9:  // Motor: GM8212D142 / AmigoBot
      Diametroa = 10.2;         // Diameter in cm
      Ppr = 500.0 / 50 * 19.5;  // Pulses Per Revolution
      Cpr = Ppr * 4;            // Counts Per Revolution
      StartDuty = 0.25;         // Initial Duty Cycle PWM
      break;

    default:
      return;
  }
}