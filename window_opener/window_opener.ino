#define VESC_SERIAL Serial4

#include <VescUart.h>
#include <SPI.h>

/* General operating parameters */
const bool debug = true;
const int LOOP_DELAY = 50; // Too short and entire messages might not come in in time between loops. One downside of polling everything.
enum State {
    IDLE_CLOSED,
    OPENING,
    WAITING_OPEN,
    CLOSING
};

// Start in closing mode, so if power is lost, the window can't get stuck open.
State currentState = CLOSING;

/* Vesc motor control parameters */
VescUart vesc;
const float CURRENT_LIMIT = 3.; // TODO tune this.
const float MOTOR_DUTY_BASELINE = 0.06; // for closing
const float MOTOR_DUTY_SLOW = 0.03;
long lastTach = 0;

float tachFullScale = 1800;
float currTach = 0.0;
float minDuty = 0.08; // for opening
float maxDuty = 0.10;

float tachClosed = 0.0;
float tachTarget = 0.0;

float buttonTach = tachFullScale/5.;

/* Switch parameters */
// Endstop pins.
const int PIN_ENDSTOP_CLOSE = 40;
const int PIN_ENDSTOP_OPEN = 39;

// Momentary switch to open the window without RFID.
const int MANUAL_OPEN_PIN = 41;

unsigned long lastEventStartTime;
const int stayOpenDuration = 2000; // Milliseconds.
const int windowMovementTimeout = 30000; // No single motion should take longer than this.

void setup() {
    if (debug) {
        Serial.begin(9600); // Default serial is for communicating with the computer for debugging. Should eventually be eliminated.
        Serial.println("beginning setup");
    }

    VESC_SERIAL.begin(19200);
    while (!VESC_SERIAL) { ; }
    vesc.setSerialPort(&VESC_SERIAL);

    pinMode(PIN_ENDSTOP_CLOSE, INPUT);
    pinMode(PIN_ENDSTOP_OPEN, INPUT);
    pinMode(MANUAL_OPEN_PIN, INPUT);

    lastEventStartTime = millis();
    if (debug) Serial.println("Setup complete.");
}

// Check if the manual open button is pressed.
bool checkManualOpenButton() {
    return digitalRead(MANUAL_OPEN_PIN);
}

float currSpeedTarget = 0;

void loop() {
    vesc.getVescValues();
    float currTach = vesc.data.tachometer;
    switch (currentState) {
        case IDLE_CLOSED:
            vesc.setCurrent(0.);
            tachClosed = currTach; // estimated position ticks in closed position.

            // If the manual button is pressed, open the window.
            if (checkManualOpenButton()) {
                tachTarget = tachClosed + buttonTach;
                lastEventStartTime = millis();
                currentState = OPENING;
            }
            delay(LOOP_DELAY);
            break;

        case OPENING:
            if (digitalRead(PIN_ENDSTOP_OPEN) || currTach >= tachTarget || (millis() - lastEventStartTime > windowMovementTimeout)) {
                if (debug) Serial.println("Window is done opening.");
                currentState = WAITING_OPEN;
                lastEventStartTime = millis();
            } else if (vesc.data.avgMotorCurrent > CURRENT_LIMIT && (millis() - lastEventStartTime) > 2000) {
                if (debug) {
                    Serial.print("current limit exceeded: ");
                    Serial.println(vesc.data.avgMotorCurrent);
                }
                lastEventStartTime = millis();
                currentState = WAITING_OPEN;
            } else if (currTach - tachClosed > 0.6 * (tachTarget - tachClosed)) {
                currSpeedTarget = currSpeedTarget * 0.5 + minDuty * 0.5;
                vesc.setDuty(currSpeedTarget);
            } else {
                currSpeedTarget = currSpeedTarget * 0.7 + maxDuty * 0.3;
                vesc.setDuty(currSpeedTarget);
            }
            delay(LOOP_DELAY);
            break;

        case WAITING_OPEN:
            // Wait for the set duration with the window open.
            if (millis() - lastEventStartTime > stayOpenDuration) {
                if (debug) Serial.println("Done waiting with the window open.");
                currentState = CLOSING;
                lastEventStartTime = millis();
            }
            delay(LOOP_DELAY);
            break;

        case CLOSING: {
            if (currTach - tachClosed <= 0.4 * tachFullScale) {
                vesc.setDuty(-MOTOR_DUTY_SLOW);
            } else {
                vesc.setDuty(-MOTOR_DUTY_BASELINE);
            }

            if (checkManualOpenButton()) {
                if (debug) Serial.println("Button pressed during window closing. Reopening.");
                tachTarget = tachClosed + buttonTach;
                lastEventStartTime = millis();
                currentState = OPENING;
            } else if (vesc.data.avgMotorCurrent > CURRENT_LIMIT && (millis() - lastEventStartTime) > 2000) {
                if (debug) {
                    Serial.print("current limit exceeded: ");
                    Serial.println(vesc.data.avgMotorCurrent);
                }
                lastEventStartTime = millis();
                currentState = OPENING;
            } else if (digitalRead(PIN_ENDSTOP_CLOSE) || (millis() - lastEventStartTime > windowMovementTimeout)) {
                lastEventStartTime = millis();
                currentState = IDLE_CLOSED;
                if (debug) Serial.println("Window is done closing.");
            }

            delay(LOOP_DELAY);
            break;
        }
        default:
            if (debug) Serial.println("Unknown  Major bug.");
    }
}
