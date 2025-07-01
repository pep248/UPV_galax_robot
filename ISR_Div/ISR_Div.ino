#define E2PROM_JL // Si hay EEPROM -> factor divisor programable / else -> factor divisor fijo
#define EVAL_SIMPLE // Evaluacion solamente por flanco de subida, usarla si CPU no da (si es posible, mejor no usarla)
#define FACTOR_DIV 50 // Factor divisor para: #if !defined(E2PROM_JL)
//#define TOGGLE_INT // Definirla si habilitar/deshabilitar INT (en teoria no seria necesario, al ser volatile y muy rapido)
#define UNO_R4_JL   // Arduino UNO R4 (compila con Nano Every. Pinout ???)
//#define PORT_DIRECT // Mucho mas eficiente escritura en las salidas (no portable) /Compila con Nano y Nano Every pero  no son compatibles los puertos
//#define DEBUG_JL  // Para ver enc1_chA_outPin o enc1_chB_outPin en el LED (para testeo, anular en real)
// !!! DEBUG_JL2 con Serial.xx falla algunas veces la cuenta (por generador de baudios, que pueda interferir ???)
//#define DEBUG_JL2 // Para buscar errores en sentido de rotacion (para testeo, anular en real)


#if defined (E2PROM_JL)
#include <EEPROM.h>
#define PIN_CONFIG 12 // LOW = serial config (EEPROM)
const byte EepromAddr = 0; // Posicion donde se almacena el divisor
#else // Factor divisor fijo
#if defined (EVAL_SIMPLE)
const byte countMax = FACTOR_DIV; // Factor divisor / EVAL_SIMPLE -> divide por countMax*2 / else -> divide por countMax
#else
const byte countMax = FACTOR_DIV * 2; // Factor divisor / EVAL_SIMPLE -> divide por countMax*2 / else -> divide por countMax
#endif // #if defined (EVAL_SIMPLE)
const byte countMaxR = countMax + countMax/2;  // Canal con retardo
#endif // #if defined (E2PROM_JL)

#if defined(UNO_R4_JL) && defined(PORT_DIRECT)
#error "Directivas incompatibles"
#endif


/*
  Para  attachInterrupt()
  LOW to trigger the interrupt whenever the pin is low,
  CHANGE to trigger the interrupt whenever the pin changes value
  RISING to trigger when the pin goes from low to high,
  FALLING for when the pin goes from high to low.
*/
#if defined EVAL_SIMPLE // Usarla, solamente si la CPU no da
  #define MODE RISING // Para  attachInterrupt(), Divider para RISING, la frecuencia se divide por countMax*2 (evaluacion simple)
#else // Evaluacion doble (subida y bajada), mejor, si da la CPU (pero no da), 
  #define MODE CHANGE // Para  attachInterrupt(), Divider para CHANGE, la frecuencia se divide por countMax (evaluacion doble)
#endif

#if defined (E2PROM_JL)
byte countMax, countMaxR;
#endif // #if defined (E2PROM_JL)

byte enc1_chA_cntMax, enc1_chB_cntMax;
byte enc1_First = 0;

#if defined(UNO_R4_JL) // Para el 2º encoder
byte enc2_chA_cntMax, enc2_chB_cntMax;
byte enc2_First = 0;
#endif // #if defined(UNO_R4_JL)

unsigned long t_Time = 0; // Para controlar (reposo/tramas)


#if defined(DEBUG_JL) // Para señalizar
#pragma message "No usar en produccion"
#endif

#if defined(DEBUG_JL2) // Para buscar errores en sentido de rotacion
#pragma message "No usar en produccion"
#endif

/*
  Cuadrature encoder1 ch A
*/
const byte enc1_chA_outPin = 8; // Fijo por PORT_DIRECT 
const byte enc1_chA_interruptPin = 2;
boolean enc1_chA_outPinState = 0; // Estado inicial, luego actualizado en espera
volatile byte enc1_chA_cnt = 0; 

/*
  Cuadrature encoder1 ch B
*/
const byte enc1_chB_outPin = 9; // Fijo por PORT_DIRECT
const byte enc1_chB_interruptPin = 3;
boolean enc1_chB_outPinState = 0; // Estado inicial, luego actualizado en espera
volatile byte enc1_chB_cnt = 0;

#if defined(UNO_R4_JL)
/*
  Cuadrature encoder2 ch A
*/
const byte enc2_chA_outPin = 10; 
const byte enc2_chA_interruptPin = 0;
boolean enc2_chA_outPinState = 0; // Estado inicial, luego actualizado en espera
volatile byte enc2_chA_cnt = 0;

/*
  Cuadrature encoder2 ch B
*/
const byte enc2_chB_outPin = 11;
const byte enc2_chB_interruptPin = 1;
boolean enc2_chB_outPinState = 0; // Estado inicial, luego actualizado en espera
volatile byte enc2_chB_cnt = 0;
#endif // #if defined(UNO_R4_JL)

void setup() {
#if defined (E2PROM_JL)  
  pinMode(PIN_CONFIG, INPUT_PULLUP);
  delay(20); // Para evitar falsa lectura

  if (digitalRead(PIN_CONFIG) == LOW) // Entra en configuracion
  {
    ReadConfig();
  }
  countMax = EEPROM.read(EepromAddr); // 
  countMaxR = countMax + countMax/2;  // Canal con retardo
#endif // #if defined (E2PROM_JL)

  enc1_chA_cntMax = countMax; // 
  enc1_chB_cntMax = countMax; //

  // Cadrature encoder1 ch A
  pinMode(enc1_chA_outPin, OUTPUT);

  pinMode(enc1_chA_interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1_chA_interruptPin), enc1_chA_ISR, MODE);

  // Cuadrature encoder1 ch B)
  pinMode(enc1_chB_outPin, OUTPUT);

  pinMode(enc1_chB_interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1_chB_interruptPin), enc1_chB_ISR, MODE);

#if defined(UNO_R4_JL)
  enc2_chA_cntMax = countMax; // 
  enc2_chB_cntMax = countMax; //
  
  // Cuadrature encoder2 ch A)
  pinMode(enc2_chA_outPin, OUTPUT);

  pinMode(enc2_chA_interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc2_chA_interruptPin), enc2_chA_ISR, MODE);

  // Cuadrature encoder2 ch B
  pinMode(enc2_chB_outPin, OUTPUT);

  pinMode(enc2_chB_interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc2_chB_interruptPin), enc2_chB_ISR, MODE);
#endif // #if defined(UNO_R4_JL)

  // Led apagado (para evitar consumo)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

#if defined(DEBUG_JL2)
  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
#endif  
}

// Cuadrature encoder1 ch A Interrupt service Routine (ISR)
void enc1_chA_ISR() {
  enc1_chA_cnt++;
}

// Cuadrature encoder1 ch B Interrupt service Routine (ISR)
void enc1_chB_ISR() {
  enc1_chB_cnt++;
}

#if defined(UNO_R4_JL)
// Cuadrature encoder2 ch A Interrupt service Routine (ISR)
void enc2_chA_ISR() {
  enc2_chA_cnt++;
}

// Cuadrature encoder2 ch B Interrupt service Routine (ISR)
void enc2_chB_ISR() {
  enc2_chB_cnt++;
}
#endif // #if defined(UNO_R4_JL)

void loop() {
  // Cuadrature encoder1 ch A)
  if (enc1_chA_cnt >= enc1_chA_cntMax)
  {
    enc1_chA_cntMax = countMax; // Tras 1ª trama, todo normal
#if defined(TOGGLE_INT)    
    noInterrupts ();
#endif    
    enc1_chA_cnt = 0;
#if defined(TOGGLE_INT)    
    interrupts ();
#endif    
    if (enc1_First)
    {
      enc1_chB_cntMax = countMaxR; // 1ª trama A -> trama B con retardo
      enc1_First = 0;
    }    
    enc1_chA_outPinState = !enc1_chA_outPinState;
#if defined(PORT_DIRECT)
    if (enc1_chA_outPinState) // Bit set
            //  76543210
      PORTB |= B00000001; // set digital 8 HIGH
    else // Bit clear
            //  76543210
      PORTB &= B11111110; // set digital 8 LOW
#else   
    digitalWrite(enc1_chA_outPin, enc1_chA_outPinState);
#endif    
#if defined(DEBUG_JL)
    digitalWrite(LED_BUILTIN, enc1_chA_outPinState);
#endif
    t_Time = millis(); // Para controlar si (reposo/tramas)
  }
  
  // Cuadrature encoder1 ch B)
  if (enc1_chB_cnt >= enc1_chB_cntMax)
  {
    enc1_chB_cntMax = countMax; // Tras 1ª trama, todo normal
#if defined(TOGGLE_INT)    
    noInterrupts ();
#endif    
    enc1_chB_cnt = 0;
#if defined(TOGGLE_INT)    
    interrupts ();
#endif     
    if (enc1_First)
    {
      enc1_chA_cntMax = countMaxR; // 1ª trama B -> trama A con retardo
      enc1_First = 0;
    }
    enc1_chB_outPinState = !enc1_chB_outPinState;
#if defined(PORT_DIRECT)
    if (enc1_chB_outPinState) // Bit set
            //  76543210
      PORTB |= B00000010; // set digital 9 HIGH
    else // Bit clear
            //  76543210
      PORTB &= B11111101; // set digital 9 LOW
#else   
    digitalWrite(enc1_chB_outPin, enc1_chB_outPinState);
#endif    
#if defined(DEBUG_JL)
    digitalWrite(LED_BUILTIN, enc1_chB_outPinState);
#endif
    t_Time = millis(); // Para controlar si (reposo/tramas)
  }

#if defined(UNO_R4_JL)
  // Cuadrature encoder2 ch A
  if (enc2_chA_cnt >= enc2_chA_cntMax)
  {
    enc2_chA_cntMax = countMax; // Tras 1ª trama, todo normal
#if defined(TOGGLE_INT)    
    noInterrupts ();
#endif    
    enc2_chA_cnt = 0;
#if defined(TOGGLE_INT)    
    interrupts ();
#endif    
    if (enc2_First)
    {
      enc2_chB_cntMax = countMaxR; // 1ª trama A -> trama B con retardo
      enc2_First = 0;
    }    
    enc2_chA_outPinState = !enc2_chA_outPinState;    
    digitalWrite(enc2_chA_outPin, enc2_chA_outPinState);
    t_Time = millis(); // Para controlar si (reposo/tramas)
  }

  // Cuadrature encoder2 ch B
  if (enc2_chB_cnt >= enc2_chB_cntMax)
  {
    enc2_chB_cntMax = countMax; // Tras 1ª trama, todo normal
#if defined(TOGGLE_INT)    
    noInterrupts ();
#endif    
    enc2_chB_cnt = 0;
#if defined(TOGGLE_INT)    
    interrupts ();
#endif    
    if (enc2_First)
    {
      enc2_chA_cntMax = countMaxR; // 1ª trama B -> trama A con retardo
      enc2_First = 0;
    }    
    enc2_chB_outPinState = !enc2_chB_outPinState;
    digitalWrite(enc2_chB_outPin, enc2_chB_outPinState);
    t_Time = millis(); // Para controlar si (reposo/tramas)
  }
#endif // #if defined(UNO_R4_JL)

  // Si lleva un tiempo parado ... (36Khz -> Ton= Toff = +- 14 us)
//  if ((millis() > t_Time+100) && !enc1_First){ // ???
#if defined(UNO_R4_JL)
  if ((millis() > t_Time+200) && (!enc1_First || !enc2_First)){ // Por si solo mueven una rueda / 200 por tramas MSE lentas a 10 Hz -> T= 0.1 s
#else
  if ((millis() > t_Time+200) && !enc1_First){ // 200 por tramas MSE lentas a 10 Hz -> T= 0.1 s
#endif  
 #if defined(DEBUG_JL)
      digitalWrite(LED_BUILTIN, LOW); // En espera led OFF
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH); // En espera led ON 
      delay(100);           
      digitalWrite(LED_BUILTIN, LOW); // En espera led OFF
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH); // En espera led ON      
 #endif

#if defined(DEBUG_JL2)
      Serial.print("#2: ");
      Serial.print(digitalRead(2));
      Serial.print(", #3: ");
      Serial.println(digitalRead(3));
#endif

      enc1_chA_outPinState = 0; 
      digitalWrite(enc1_chA_outPin, enc1_chA_outPinState);
    
      enc1_chB_outPinState = 0;
      digitalWrite(enc1_chB_outPin, enc1_chB_outPinState);
 
      enc1_chA_cnt = 0;
      enc1_chB_cnt = 0;
      enc1_chA_cntMax = countMax;
      enc1_chB_cntMax = countMax;
      enc1_First = 1;
#if defined(UNO_R4_JL)
      enc2_chA_outPinState = 0;
      digitalWrite(enc2_chA_outPin, enc2_chA_outPinState);
    
      enc2_chB_outPinState = 0;
      digitalWrite(enc2_chB_outPin, enc2_chB_outPinState);

      enc2_chA_cnt = 0;
      enc2_chB_cnt = 0;
      enc2_chA_cntMax = countMax;
      enc2_chB_cntMax = countMax;
      enc2_First = 1;
/*
      digitalWrite(LED_BUILTIN, HIGH); // En espera led ON 
      delay(100);           
      digitalWrite(LED_BUILTIN, LOW); // En espera led OFF
      delay(100);
*/      
#endif // #if defined(UNO_R4_JL)    
  }    
}

#if defined (E2PROM_JL)
// Lee configuracion
void ReadConfig(void)
{
   byte ValueW, ValueR;
    // initialize serial:
    Serial.begin(115200);
    while (!Serial)
      ; // wait for serial port to connect. Needed for native USB port only
    delay(2000); // Si no, repite mensajes

    ValueR = EEPROM.read(EepromAddr);

    Serial.print(F("\nValor almacenado: "));
    Serial.print(ValueR);
    Serial.print(F(",  divide por: "));
#if defined EVAL_SIMPLE // Divider para RISING, la frecuencia se divide por countMax*2 (evaluacion simple)
    Serial.println(ValueR * 2);
    Serial.print(F("\nNuevo valor (no se testea), 1 a 168: ")); // countMaxR = 168+168/2 = 252
#else // Divider para CHANGE, la frecuencia se divide por countMax (evaluacion doble)
    Serial.println(ValueR);
    Serial.print(F("\nNuevo valor (PAR no se testea), 2 a 168: ")); // countMaxR = 168+168/2 = 252
#endif

    // Lee byte (como un entero)
    while (!Serial.available())
      ;
    ValueW = Serial.parseInt(SKIP_ALL); // Solo numeros
    Serial.println(ValueW);
    serialPurge();

    while (1) {
      Serial.print(F("Almacenar valor (s/n): "));
      while (!Serial.available())
        ;     
      char c = Serial.read();
      Serial.println(c);
      serialPurge(); 
      if ((c == 's') || (c == 'S') )
      {
        if (ValueR != ValueW) // Solamente grabar si distintos valores
        {
          EEPROM.write(EepromAddr, ValueW);
          Serial.print(F("Valor almacenado: "));
          ValueR = EEPROM.read(EepromAddr);
          Serial.println(ValueR);
        }
        else {
          Serial.println(F("Ignorado, valor ya almacenado anteriormente"));
        }
        break;
    }
      if ((c == 'n') || (c == 'N')) 
      {
        break;
      }
    }

    serialPurge(); // ???

    Serial.print(F("Divide por: "));
#if defined EVAL_SIMPLE // Divider para RISING, la frecuencia se divide por countMax*2 (evaluacion simple)
    Serial.println(ValueR * 2);
#else // Divider para CHANGE, la frecuencia se divide por countMax (evaluacion doble)
    Serial.println(ValueR);
#endif    
    Serial.println(F("\nQuitar cable PULL-DOWN y pulsar Reset"));
    delay(100); // Para verlo
    Serial.end();
    while(1);
}
// Purge serial incoming chars
void serialPurge(void) {
  delay(100); // wait for CR, LN ...     
  while (Serial.available() > 0)  // Clear
    Serial.read();
}
#endif // #if defined (E2PROM_JL)