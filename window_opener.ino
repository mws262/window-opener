#include <Servo.h>
#include <Adafruit_INA260.h>

#define RFID Serial1

Adafruit_INA260 ina260 = Adafruit_INA260();
const double CURRENT_LIMIT = 320.; 

// Authorized list of RFIDs.
const int NUM_CATS = 2;
const int NUM_DOGS = 2;
const int MAX_ID_LEN = 20;
const char catAuthID[NUM_CATS][MAX_ID_LEN] = {"F98200041199001878", // Tonks.
                                           "985141002571129"};
const char dogAuthID[NUM_DOGS][MAX_ID_LEN] = {"F9000202060000037A", // Test tag.
                                           "todoPIPER"};                                           

// Driving pins to motor hbridge.
const int PIN_MOTOR_CLOSE = 11;
const int PIN_MOTOR_OPEN = 10;

// Endstop pins.
const int PIN_ENDSTOP_CLOSE = 51;
const int PIN_ENDSTOP_OPEN = 9;

// Window lock servo pin.
const int PIN_SERVO_ENABLE = 53; // NOTE: enabled when sent a LOW logic level.
const int PIN_LOCK_SERVO = 28;

// Momentary switch to open the window without a tag read.
const int PIN_SWITCH = 52;

Servo lock_servo;
const int LOCK_CLOSE_VAL = 25; // Servo pwms for min and max range of the servo.
const int LOCK_OPEN_VAL = 130;
const int LOCK_DURATION = 2000; // Milliseconds for the lock to complete opening or closing.

const int LOOP_DELAY = 50; // Too shaort and entire messages might not come in in time between loops. One downside of polling everything.

const bool debug = true;

enum State {
    IDLE_CLOSED,
    UNLOCKING,
    LOCKING,
    OPENING,
    WAITING_OPEN,
    CLOSING,
};

// Start in closing mode, so if power is lost, the window can't get stuck open.
State currentState = CLOSING;

unsigned long lastEventStartTime;
const int stayOpenDuration = 4000; // Milliseconds.
const int catWindowTime = 8000;
const int dogWindowTime = 28000;
const int maxClosingTime = 45000;
int windowMovementTimeout = dogWindowTime;

void setup() {

    // initialize both serial ports:
    if (debug) Serial.begin(9600); // Default serial is for communicating with the computer for debugging. Should eventually be eliminated.

    RFID.begin(9600); // Serial 1 is for communication with the RFID board. Information only flows in from it.
    lock_servo.attach(PIN_LOCK_SERVO);
    lock_servo.write(LOCK_CLOSE_VAL);

    pinMode(PIN_MOTOR_CLOSE, OUTPUT);
    pinMode(PIN_MOTOR_OPEN, OUTPUT);
    pinMode(PIN_SERVO_ENABLE, OUTPUT);
    pinMode(PIN_ENDSTOP_CLOSE, INPUT);
    pinMode(PIN_ENDSTOP_OPEN, INPUT);
    pinMode(PIN_SWITCH, INPUT);

    digitalWrite(PIN_SERVO_ENABLE, HIGH); 

    if (!ina260.begin()) {
      if (debug) Serial.println("Couldn't find INA260 chip");
    } else {
      if (debug) Serial.println("Found INA260 current sensor.");
    }
    
    lastEventStartTime = millis();
    if (debug) Serial.println("Setup complete.");
}

// Read from the serial buffer and check for a valid RFID tag. Returns the index of the tag id or -1 if the not authenticated.
int authenticateRFID() {
  
    if (RFID.available()) {
      char id[64];
      char byteRead;
      
      int availableBytes = RFID.available();
      for (int i = 0; i < availableBytes; i++) {
         id[i] = RFID.read();
         id[i+1] = '\0'; // Append a null
      }

        if (debug) Serial.println(id);
        for (int i = 0; i < NUM_CATS; i++) {
            if (strstr(id, catAuthID[i])) {
                if (debug) {
                    Serial.print(catAuthID[i]);
                    Serial.println(" identified!");
                }
                windowMovementTimeout = catWindowTime;
                return i;
            }
        }

        for (int i = 0; i < NUM_DOGS; i++) {
            if (strstr(id, dogAuthID[i])) {
                if (debug) {
                    Serial.print(dogAuthID[i]);
                    Serial.println(" identified!");
                }
                windowMovementTimeout = dogWindowTime;
                return i;
            }
        }
    }else {
      if (digitalRead(PIN_SWITCH) == LOW) { // Basically treat the switch as another tag.
        if (debug) {
          Serial.print("switch triggered");
        }
        windowMovementTimeout = dogWindowTime;
        return 999;
      }
    }
    return -1;
}

// Use the lock servo to unlock the window.
void openLock() {
    if (debug) Serial.println("Lock open commanded.");
    digitalWrite(PIN_SERVO_ENABLE, LOW); // Turn the servo on and off on each use to prevent humming and power use when not in use.
    lock_servo.write(LOCK_OPEN_VAL);
    delay(LOCK_DURATION);
    digitalWrite(PIN_SERVO_ENABLE, HIGH);
}

// Use the lock servo to lock the window.
void closeLock() {
    if (debug) Serial.println("Lock close commanded.");
    digitalWrite(PIN_SERVO_ENABLE, LOW); // Turn the servo on and off on each use to prevent humming and power use when not in use.
    lock_servo.write(LOCK_CLOSE_VAL);
    delay(LOCK_DURATION);
    digitalWrite(PIN_SERVO_ENABLE, HIGH);
}

void loop() {
    switch(currentState) {
        case IDLE_CLOSED:
            // Be constantly checking the serial buffer for new RFID tag reads.
            // Unlock the window when a correct match occurs.
            if (authenticateRFID() >= 0) {
                currentState = UNLOCKING;
            }
            delay(LOOP_DELAY);
            break;

        case UNLOCKING:
            // Unlock the window, and transition to opening the window.
            openLock();
            lastEventStartTime = millis();
            currentState = OPENING;
            break;

        case LOCKING:
            // Lock the window, and go to idle.
            closeLock();
            currentState = IDLE_CLOSED;
            break;

        case OPENING:
            // Start opening the window. Keep going until a time threshold or the endstop is hit. TODO: different durations for differently sized animals.
            digitalWrite(PIN_MOTOR_CLOSE, LOW);
            digitalWrite(PIN_MOTOR_OPEN, HIGH);

            // Stop opening when the endstop is triggered or too much time elapses.
            if (digitalRead(PIN_ENDSTOP_OPEN) == LOW || (millis() - lastEventStartTime > windowMovementTimeout)) {
                if (debug) Serial.println("Window is done opening.");

                digitalWrite(PIN_MOTOR_OPEN, LOW);
                currentState = WAITING_OPEN;
                lastEventStartTime = millis();
            }
            delay(LOOP_DELAY);
            break;

        case WAITING_OPEN:
            // Wait for awhile with the window open to give time for the slowpokes to make up their minds. If a news
            // valid authentication occurs, reset the duration counter.
            if (authenticateRFID() >= 0) {
                if (debug) Serial.println("Tag detected during window open time. Resetting the wait timer.");
                lastEventStartTime = millis();
            } else if (millis() - lastEventStartTime > stayOpenDuration) { // Time limit met for closing to occur.
                if (debug) Serial.println("Done waiting with the window open.");
                currentState = CLOSING;
                lastEventStartTime = millis();
            }
            delay(LOOP_DELAY);
            break;

        case CLOSING:
            digitalWrite(PIN_MOTOR_OPEN, LOW);
            digitalWrite(PIN_MOTOR_CLOSE, HIGH);

            // Stop opening when the endstop is triggered or too much time elapses. If a tag gets authenticated during
            // this time. Reopen the window for the duration we had been closing it (or until endstop).
            if (authenticateRFID() >= 0 || (ina260.readCurrent() > CURRENT_LIMIT && (millis() - lastEventStartTime) > 1000)) {
                if (debug) Serial.println("Tag authenticated during window closing. Reopening.");
                unsigned long timeToReopen = millis() - lastEventStartTime; // How much time have we been closing the window so far?
                lastEventStartTime = millis() - (windowMovementTimeout - timeToReopen); // Cheat the timeout. Set the current time to be in the future so the timeout triggers after the amount of time we had been closing the window elapses.
                currentState = OPENING; 
            } else if (digitalRead(PIN_ENDSTOP_CLOSE) == LOW || (millis() - lastEventStartTime > maxClosingTime)) {
                if (debug) Serial.println("Window is done closing.");

                digitalWrite(PIN_MOTOR_CLOSE, LOW);
                currentState = LOCKING;
            }
            delay(LOOP_DELAY);
            break;
        default:
            if (debug) Serial.println("Unknown  Major bug.");
    }
}
