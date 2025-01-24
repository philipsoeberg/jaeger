/* Sketch made by Thomas Jensen for Dinizuli 2025 - Franz Jäger v5 - 2025-01-19
  
   Designed HW setup (Arduino Every):
   pin 2+3 : inv A and inv B terminals on a 100pp precicion rotary encoder (clockwise rotation for increasing values 0-99)
   pin 7   : Button (to gnd)
   pin 10  : Red LED
   pin 11  : Green LED
   pin 13  : Relay module for [relayAudioClue] sound and driving magnetic door lock [openDoor]

   Encoder logic based on Oleg Mazurov's code for rotary encoder interrupt service routines for AVR micros
   here https://chome.nerpa.tech/mcu/reading-rotary-encoder-on-arduino/
   and using interrupts https://chome.nerpa.tech/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros/
*/

// Enable Serial Debug outut. Comment out to disable.
#define SERIAL_DEBUG

// Define rotary encoder pins
#define PIN_ENCODER_A   (2)
#define PIN_ENCODER_B   (3)
#define PIN_BUTTON      (7)
#define PIN_LED_RED     (10)
#define PIN_LED_GREEN   (11)
#define PIN_RELAY       (13)

// Delay Constants
#define RELAY_AUDIO_CLUE_DELAY (10)      // in ms
#define RELAY_DOOROPEN_DELAY   (300)     // in ms
#define LED_VISUAL_CLUE_DELAY  (10)      // in ms
#define ALARM_TIMEOUT          (unsigned long)(900000)  // in ms. 15 minutes

// Encoder value
#define ENCODER_RESET_VALUE   (0)
#define ENCODER_OUTSIDE_RANGE (100)
volatile int encoderValue = ENCODER_RESET_VALUE;

// Encoder direction
#define ENCODER_DIRECTION_CW  (true)
#define ENCODER_DIRECTION_CCW (false)
volatile bool encoderDirection = ENCODER_DIRECTION_CW;

// Button pressed flag
volatile bool armButtonPressed = false;
volatile bool buttonPressed = false;

// Pass codes
#define CODE1_VALUE (68)                         // Code 1
#define CODE1_DIRECTION (ENCODER_DIRECTION_CCW)  // Required direction to accept code
#define CODE1_GRACETIMEOUT (unsigned long)(1500) // Code 1 grace periode with no encoder movement before code accept

#define CODE2_VALUE (25)                         // Code 2
#define CODE2_DIRECTION (ENCODER_DIRECTION_CW)   // Required direction to accept code
#define CODE2_GRACETIMEOUT (unsigned long)(3000) // Code 2 grace periode with no encoder movement before code accept

#define CODE3_VALUE (44)                         // Code 3
#define CODE3_DIRECTION (ENCODER_DIRECTION_CCW)  // Required direction to accept code
#define CODE3_GRACETIMEOUT (unsigned long)(4500) // Code 3 grace periode with no encoder movement before code accept

enum States {
  ST_POWERON,
  ST_RESET,
  ST_IDLING_WAITING_TO_BEGIN,
  ST_SEEK_CODE_1,
  ST_WAIT_CODE_1,
  ST_ACCEPT_CODE_1,
  ST_SEEK_CODE_2,
  ST_WAIT_CODE_2,
  ST_ACCEPT_CODE_2,
  ST_SEEK_CODE_3,
  ST_WAIT_CODE_3,
  ST_ACCEPT_CODE_3,
  ST_CODES_ACCEPTED,
  ST_IDLING_WAIT_GAME_RESET,
  ST_ALARM_TRIGGERED,
  ST_ALARM_TRIGGERED_BODY,
};
enum States state = ST_POWERON;

// ****************************************************************
// *** SETUP                                                    ***
// ****************************************************************
void setup() {
  // Setup pins and attach interrupts
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), read_encoder_irqhandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), read_encoder_irqhandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), read_button_irqhandler, CHANGE);

  // Set the LED pins as outputs
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  // Set the relay pin as output
  pinMode(PIN_RELAY, OUTPUT);

  // Start the serial monitor to show output
  #ifdef SERIAL_DEBUG
    Serial.begin(115200);
  #endif
}

/****************************************************************
 *** MAIN LOOP                                                ***
 ****************************************************************/

void loop() {
  static int prevEncoderValue = ENCODER_OUTSIDE_RANGE;
  static unsigned long alarmTimer = 0;
  static unsigned long graceTimer = 0;
  static bool resetAudioVisualCycle = false;
  static unsigned long audioVisualTimer = 0;
  static unsigned long audioVisualTimerIteration = 0;

  #ifdef SERIAL_DEBUG
    static enum State oldState = -1;
    if (oldState != state) {
      Serial.print("State Change: From: ");
      Serial.print(oldState);
      Serial.print(" To: ");
      Serial.println(state);
      oldState = state;
    }
  #endif
  
  // #ifdef SERIAL_DEBUG
  //   if(encoderValue != prevEncoderValue) {
  //     Serial.print(" , encoderValue:");
  //     Serial.print(encoderValue);
  //     Serial.print(" , encoderDirection:");
  //     Serial.print(encoderDirection);
  //     Serial.print(" , state:");
  //     Serial.println(state);
  //   }
  // #endif

  switch(state) {

    case ST_POWERON:
      state = ST_RESET;
      break;
    
    case ST_RESET:
      ledOff();
      armButtonPressed = false;
      buttonPressed = false;
      prevEncoderValue = encoderValue;
      state = ST_IDLING_WAITING_TO_BEGIN;
      resetAudioVisualCycle = true;
      audioVisualTimer = millis();
      break;

    case ST_IDLING_WAITING_TO_BEGIN:
      // Wait for encoderValue change
      if (prevEncoderValue != encoderValue) {
        alarmTimer = millis();
        ledRedOn();
        state = ST_SEEK_CODE_1;
        return;
      }
      //RED led blip 10ms every 1 second
      if ((millis() - audioVisualTimer) > 1000) {
        ledRedOn();
        delay(10);
        ledOff();
        audioVisualTimer = millis();
      }
      break;

    case ST_SEEK_CODE_1:
      // If alarmTimeout, Goto Alarm. Game over.
      if ((millis() - alarmTimer) > ALARM_TIMEOUT) { state = ST_ALARM_TRIGGERED; return; }

      // Wait for code arrival
      if (encoderValue == CODE1_VALUE && encoderDirection == CODE1_DIRECTION) {
        relayAudioClue();
        graceTimer = millis();
        state = ST_WAIT_CODE_1;
      }
      break;

    case ST_WAIT_CODE_1:
      // If alarmTimeout, Goto Alarm. Game over.
      if ((millis() - alarmTimer) > ALARM_TIMEOUT) { state = ST_ALARM_TRIGGERED; return; }

      // If encoderValue has changed away from Code, go back to seeking code1
      if (encoderValue != CODE1_VALUE) { state = ST_SEEK_CODE_1; return; }

      // Wait for encoder grace timeout before accepting code
      if ((millis() - graceTimer) > CODE1_GRACETIMEOUT) { state = ST_ACCEPT_CODE_1; return; }

      break;

    case ST_ACCEPT_CODE_1:
      // If alarmTimeout, Goto Alarm. Game over.
      if ((millis() - alarmTimer) > ALARM_TIMEOUT) { state = ST_ALARM_TRIGGERED; return; }

      // Issue clue and accept code
      ledVisualClue();
      state = ST_SEEK_CODE_2;
      break;

    case ST_SEEK_CODE_2:
      // If alarmTimeout, Goto Alarm. Game over.
      if ((millis() - alarmTimer) > ALARM_TIMEOUT) { state = ST_ALARM_TRIGGERED; return; }

      // If encoderDirection is not correct direction, go back to seek state1, but only after move away from previous code
      if (encoderValue != CODE1_VALUE && encoderDirection != CODE2_DIRECTION) { state = ST_SEEK_CODE_1; return; }

      // Wait for code arrival
      if (encoderValue == CODE2_VALUE && encoderDirection == CODE2_DIRECTION) {
        relayAudioClue();
        graceTimer = millis();
        state = ST_WAIT_CODE_2;
      }
      break;

    case ST_WAIT_CODE_2:
      // If alarmTimeout, Goto Alarm. Game over.
      if ((millis() - alarmTimer) > ALARM_TIMEOUT) { state = ST_ALARM_TRIGGERED; return; }

      // If encoderValue has changed away from Code, go back to seeking code1
      if (encoderValue != CODE2_VALUE) { state = ST_SEEK_CODE_1; return; }

      // Wait for encoder grace timeout before accepting code
      if ((millis() - graceTimer) > CODE2_GRACETIMEOUT) { state = ST_ACCEPT_CODE_2; return; }

      break;

    case ST_ACCEPT_CODE_2:
      // If alarmTimeout, Goto Alarm. Game over.
      if ((millis() - alarmTimer) > ALARM_TIMEOUT) { state = ST_ALARM_TRIGGERED; return; }

      // Issue clue and accept code
      ledVisualClue();
      state = ST_SEEK_CODE_3;
      break;

    case ST_SEEK_CODE_3:
      // If alarmTimeout, Goto Alarm. Game over.
      if ((millis() - alarmTimer) > ALARM_TIMEOUT) { state = ST_ALARM_TRIGGERED; return; }

      // If encoderDirection is not correct direction, go back to seek state1, but only after move away from previous code
      if (encoderValue != CODE2_VALUE && encoderDirection != CODE3_DIRECTION) { state = ST_SEEK_CODE_1; return; }

      // Wait for code arrival
      if (encoderValue == CODE3_VALUE && encoderDirection == CODE3_DIRECTION) {
        relayAudioClue();
        graceTimer = millis();
        state = ST_WAIT_CODE_3;
      }
      break;

    case ST_WAIT_CODE_3:
      // If alarmTimeout, Goto Alarm. Game over.
      if ((millis() - alarmTimer) > ALARM_TIMEOUT) { state = ST_ALARM_TRIGGERED; return; }

      // If encoderValue has changed away from Code, go back to seeking code1
      if (encoderValue != CODE3_VALUE) { state = ST_SEEK_CODE_1; return; }

      // Wait for encoder grace timeout before accepting code
      if ((millis() - graceTimer) > CODE3_GRACETIMEOUT) { state = ST_ACCEPT_CODE_3; return; }

      break;

    case ST_ACCEPT_CODE_3:
      // If alarmTimeout, Goto Alarm. Game over.
      if ((millis() - alarmTimer) > ALARM_TIMEOUT) { state = ST_ALARM_TRIGGERED; return; }

      // Issue clue and accept code
      ledVisualClue();
      state = ST_CODES_ACCEPTED;
      break;

    case ST_CODES_ACCEPTED:
      // LED to green
      ledGreenOn();

      // Open door, game won
      relayOpenDoor();

      // Listen for button changes
      armButtonPressed = true;
      
      // Wait for game reset
      state = ST_IDLING_WAIT_GAME_RESET;
      break;
    
    case ST_IDLING_WAIT_GAME_RESET:      
      // If button is pressed, reset statemachine
      if (buttonPressed == true) { state = ST_RESET; return; }
      break;

    case ST_ALARM_TRIGGERED:
      // Setup alarm
      audioVisualTimer = millis();
      audioVisualTimerIteration = 0;
      armButtonPressed = true;
      state = ST_ALARM_TRIGGERED_BODY;
      return;
    
    case ST_ALARM_TRIGGERED_BODY:
      // If button is pressed, reset statemachine
      if (buttonPressed == true) { state = ST_RESET; return; }

      //LED flashy (250ms ON every 500ms)
      //Relay noisy (10ms ON every 100ms for 5 seconds)
      if ((millis() - audioVisualTimer) > 10) {
        audioVisualTimerIteration++; //this will ++ every 10ms'ich
        audioVisualTimer = millis();
      }
      if ((audioVisualTimerIteration % (unsigned long)25) == 0) {  //every 250ms
        ledRedOn();
      }
      if ((audioVisualTimerIteration % (unsigned long)50) == 0) {  //every 500ms
        ledOff();
      }
      if ((audioVisualTimerIteration % (unsigned long)25) == 0) {  //every 250ms
        if (audioVisualTimerIteration < 20) {    //for up to 5000 ms (5 seconds)
          relayAudioClue();                       //do relayAudioClue
        }
      }

      break;

    default:
      state = ST_RESET;
      break;
  }
}

// ****************************************************************
// *** SUPPORT FUNCTIONS                                        ***
// ****************************************************************
void read_encoder_irqhandler() {
  // Encoder interrupt routine for both pins. 
  // Updates encoderValue and encoderDirection
  // if they are valid and have rotated a full indent
  // #ifdef SERIAL_DEBUG
  //   Serial.print("=>");
  // #endif
  static int maxCount = ENCODER_OUTSIDE_RANGE;
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(PIN_ENCODER_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(PIN_ENCODER_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // #ifdef SERIAL_DEBUG
  //   Serial.print(" encval=");
  //   Serial.print(encval);
  // #endif

  // Update encoderValue if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = 1;
    if(encoderValue == (maxCount-1)) {
      encoderValue = -1;
    } 
    encoderValue = encoderValue + changevalue;   // Update encoderValue
    encval = 0;
    // wheel turned clockwise
    encoderDirection = ENCODER_DIRECTION_CW;                    
  }
  else if( encval < -3 ) {             // Four steps backward
    int changevalue = -1;
    if(encoderValue == 0) {
      encoderValue = maxCount;
    } 
    encoderValue = encoderValue + changevalue;    // Update encoderValue
    encval = 0;
    // wheel turned encoderValue clockwise
    encoderDirection = ENCODER_DIRECTION_CCW;
  }

  // #ifdef SERIAL_DEBUG
  //   Serial.print(" encoderValue=");
  //   Serial.print(encoderValue);
  //   Serial.print(" encoderDirection=");
  //   Serial.print(encoderDirection);
  //   Serial.println();
  // #endif
}

void read_button_irqhandler() {
  if (armButtonPressed == true) { buttonPressed = true; }
}

void ledOff() {
  digitalWrite(PIN_LED_RED, LOW);     //red off
  digitalWrite(PIN_LED_GREEN, LOW);   //green off
}

void ledRedOn() {
  digitalWrite(PIN_LED_RED, HIGH);    //red on
  digitalWrite(PIN_LED_GREEN, LOW);   //green off
}

void ledGreenOn() {
  digitalWrite(PIN_LED_RED, LOW);     //red off
  digitalWrite(PIN_LED_GREEN, HIGH);  //green on
}

// Short-time 'blink' the red LED.
// Note: Assumes RED is on as it turns RED on afterwards.
void ledVisualClue() {
  ledOff();
  delay(LED_VISUAL_CLUE_DELAY);
  ledRedOn();
}

void relayOn() {
  digitalWrite(PIN_RELAY, HIGH);              // Turn the relay on
}

void relayOff() {
  digitalWrite(PIN_RELAY, LOW);              // Turn the relay off
}

// Make relay 'click' for a short time for a audio-sensation
void relayAudioClue() {
  relayOn();
  delay(RELAY_AUDIO_CLUE_DELAY);
  relayOff();
}

// Keep relay on for a longer time to open the door
void relayOpenDoor() {
  relayOn();
  delay(RELAY_DOOROPEN_DELAY);  
  relayOff();
}
