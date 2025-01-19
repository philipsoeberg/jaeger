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

// Enable Serial & Display Debug outut. Comment out to disable.
// #define SERIAL_DEBUG
// #define DISPLAY_DEBUG

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
#define ALARM_TIMEOUT          (900000)  // in ms. 15 minutes

// Encoder value
#define ENCODER_RESET_VALUE   (0)
#define ENCODER_OUTSIDE_RANGE (100)
volatile int encoderValue = ENCODER_RESET_VALUE;

// Encoder direction
#define ENCODER_DIRECTION_CW  (true)
#define ENCODER_DIRECTION_CCW (false)
volatile bool encoderDirection = ENCODER_DIRECTION_CW;

// Button pressed flag
volatile bool buttonPressed = false;

// Pass codes
#define CODE1VALUE (68)                         // Code 1
#define CODE1DIRECTION (ENCODER_DIRECTION_CCW)  // Required direction to accept code
#define CODE1GRACETIMEOUT (1500)                // Code 1 grace periode with no encoder movement before code accept

#define CODE2VALUE (25)                         // Code 2
#define CODE2DIRECTION (ENCODER_DIRECTION_CW)   // Required direction to accept code
#define CODE2GRACETIMEOUT (3000)                // Code 2 grace periode with no encoder movement before code accept

#define CODE3VALUE (44)                         // Code 3
#define CODE3DIRECTION (ENCODER_DIRECTION_CCW)  // Required direction to accept code
#define CODE3GRACETIMEOUT (4500)                // Code 3 grace periode with no encoder movement before code accept

enum States {
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
};
enum States state = ST_RESET;

// ****************************************************************
// *** SETUP                                                    ***
// ****************************************************************
#ifdef SERIAL_DEBUG
#define SERIALPRINT(...) Serial.print(__VA_ARGS__)
#else
#define SERIALPRINT(...) yield()
#endif
void setup() {
  // Setup pins and attach interrupts
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), read_encoder_irqhandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), read_encoder_irqhandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), read_button_irqhandler, CHANGE);

  // Set the LED pins as outputs
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  // Set the relay pin as output
  pinMode(PIN_RELAY, OUTPUT);

  // Start the serial monitor to show output
  #ifdef SERIAL_DEBUG
    Serial.begin(9600);
  #endif
}

/****************************************************************
 *** MAIN LOOP                                                ***
 ****************************************************************/

void loop() {
  static int prevEncoderValue = ENCODER_OUTSIDE_RANGE;
  static int alarmTimer = 0;
  static int graceTimer = 0;
  static bool acceptButtonChange = false;

  if (acceptButtonChange == false && buttonPressed == true) {
    buttonPressed = false;
  }

  switch(state) {

    case ST_RESET:
      ledOff();
      acceptButtonChange = false;
      prevEncoderValue = encoderValue;
      state = ST_IDLING_WAITING_TO_BEGIN;
      break;

    case ST_IDLING_WAITING_TO_BEGIN:
      // Wait for encoderValue change
      if (prevEncoderValue != encoderValue) {
        alarmTimer = millis();
        ledRedOn();
        state = ST_SEEK_CODE_1;
      }
      //TODO: RED led blip 10ms @ 1hz
      break;

    case ST_SEEK_CODE_1:
      // If alarmTimeout, Goto Alarm. Game over.
      if ((millis() - alarmTimer) > ALARM_TIMEOUT) { state = ST_ALARM_TRIGGERED; return; }

      // Wait for code arrival
      if (encoderValue == CODE1VALUE && encoderDirection == CODE1DIRECTION) {
        relayAudioClue();
        graceTimer = millis();
        state = ST_WAIT_CODE_1;
      }
      break;

    case ST_WAIT_CODE_1:
      // If alarmTimeout, Goto Alarm. Game over.
      if ((millis() - alarmTimer) > ALARM_TIMEOUT) { state = ST_ALARM_TRIGGERED; return; }

      // If encoderValue has changed away from Code, go back to seeking code1
      if (encoderValue != CODE1VALUE) { state = ST_SEEK_CODE_1; return; }

      // Wait for encoder grace timeout before accepting code
      if ((millis() - graceTimer) > CODE1GRACETIMEOUT) { state = ST_ACCEPT_CODE_1; return; }

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

      // If encoderDirection is not correct direction, go back to seek state1
      if (encoderDirection != CODE2DIRECTION) { state = ST_SEEK_CODE_1; return; }

      // Wait for code arrival
      if (encoderValue == CODE2VALUE && encoderDirection == CODE2DIRECTION) {
        relayAudioClue();
        graceTimer = millis();
        state = ST_WAIT_CODE_2;
      }
      break;

    case ST_WAIT_CODE_2:
      // If alarmTimeout, Goto Alarm. Game over.
      if ((millis() - alarmTimer) > ALARM_TIMEOUT) { state = ST_ALARM_TRIGGERED; return; }

      // If encoderValue has changed away from Code, go back to seeking code1
      if (encoderValue != CODE2VALUE) { state = ST_SEEK_CODE_1; return; }

      // Wait for encoder grace timeout before accepting code
      if ((millis() - graceTimer) > CODE2GRACETIMEOUT) { state = ST_ACCEPT_CODE_2; return; }

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

      // If encoderDirection is not correct direction, go back to seek state1
      if (encoderDirection != CODE3DIRECTION) { state = ST_SEEK_CODE_1; return; }

      // Wait for code arrival
      if (encoderValue == CODE3VALUE && encoderDirection == CODE3DIRECTION) {
        relayAudioClue();
        graceTimer = millis();
        state = ST_WAIT_CODE_3;
      }
      break;

    case ST_WAIT_CODE_3:
      // If alarmTimeout, Goto Alarm. Game over.
      if ((millis() - alarmTimer) > ALARM_TIMEOUT) { state = ST_ALARM_TRIGGERED; return; }

      // If encoderValue has changed away from Code, go back to seeking code1
      if (encoderValue != CODE3VALUE) { state = ST_SEEK_CODE_1; return; }

      // Wait for encoder grace timeout before accepting code
      if ((millis() - graceTimer) > CODE3GRACETIMEOUT) { state = ST_ACCEPT_CODE_3; return; }

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

      // Wait for game reset
      state = ST_IDLING_WAIT_GAME_RESET;
      break;
    
    case ST_IDLING_WAIT_GAME_RESET:
      // Listen for button changes
      if (acceptButtonChange == false) { acceptButtonChange = true; }
      
      // If button is pressed, reset statemachine
      if (buttonPressed == true) { state = ST_RESET; return; }
      break;

    case ST_ALARM_TRIGGERED:
      // Listen for button changes
      if (acceptButtonChange == false) { acceptButtonChange = true; }

      //TODO: LED flashy
      //TODO: Relay noisy

      // If button is pressed, reset statemachine
      if (buttonPressed == true) { state = ST_RESET; return; }

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
  SERIALPRINT("=");
  static const int _maxCount = ENCODER_OUTSIDE_RANGE;
  static uint8_t _old_AB = 3;  // Lookup table index
  static int8_t _encval = 0;   // Encoder value  
  static const int8_t _enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  _old_AB <<=2;  // Remember previous state

  if (digitalRead(PIN_ENCODER_A)) _old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(PIN_ENCODER_B)) _old_AB |= 0x01; // Add current state of pin B
  
  _encval += _enc_states[( _old_AB & 0x0f )];

  // Update encoderValue if encoder has rotated a full indent, that is at least 4 steps
  if( _encval > 3 ) {        // Four steps forward
    int changevalue = 1;
    if(encoderValue == (_maxCount-1)) {
      encoderValue = -1;
    } 
    encoderValue = encoderValue + changevalue;   // Update encoderValue
    _encval = 0;
    // wheel turned clockwise
    encoderDirection = ENCODER_DIRECTION_CW;                    
  }
  else if( _encval < -3 ) {             // Four steps backward
    int changevalue = -1;
    if(encoderValue == 0) {
      encoderValue = _maxCount;
    } 
    encoderValue = encoderValue + changevalue;    // Update encoderValue
    _encval = 0;
    // wheel turned encoderValue clockwise
    encoderDirection = ENCODER_DIRECTION_CCW;
  }
}

void read_button_irqhandler() {
  buttonPressed = true;
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
  #ifdef DISPLAY_DEBUG
    display.clearDisplay();
    display.setCursor(0, 0); // Start at top-left corner
    display.print("Lock OPEN");
    display.display();  
  #endif
}