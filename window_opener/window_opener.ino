#include <Servo.h>
#include <Adafruit_INA260.h>
#include <VescUart.h>

#define RFID Serial1
#define VESC_SERIAL Serial2
#include <SPI.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

VescUart vesc;
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

const float RPM_OPENING = 100; // TODO
const float RPM_CLOSING = -50; // TODO 

const bool debug = true;

enum State {
    IDLE_CLOSED,
    UNLOCKING,
    LOCKING,
    OPENING,
    WAITING_OPEN,
    CLOSING,
    CURFEW
};

// Start in closing mode, so if power is lost, the window can't get stuck open.
State currentState = CLOSING;

unsigned long lastEventStartTime;
const int stayOpenDuration = 4000; // Milliseconds.
const int catWindowTime = 8000;
const int dogWindowTime = 28000;
const int maxClosingTime = 45000;
int windowMovementTimeout = dogWindowTime;


// FOR ETHERNET TIME SYNCING.
// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
unsigned int localPort = 8888;       // local port to listen for UDP packets
const char timeServer[] = "time.nist.gov"; // time.nist.gov NTP server
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
const int timeCheckTimeout = 3000;
const int timezoneOffsetUTC= -7;
uint8_t hourTime = 0;
uint8_t minuteTime = 0;
uint8_t secondTime = 0;
unsigned long lastTimeUpdate = 0;
const unsigned long timeCheckInterval = 1 * (60 * 60 * 1000); // 1 hr in milliseconds. 
unsigned long lastEpochRetrieved = 0; // Seconds since 1970 retrieved from the internet.

const uint8_t curfewStartTime = 22; // 10PM
const uint8_t curfewEndTime = 7;

int setupEthernet() {
    // start Ethernet and UDP
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    return 0;
  }
  Udp.begin(localPort);
  return 1;
}

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

    setupEthernet();

    VESC_SERIAL.begin(19200);
    while (!VESC_SERIAL) {;}
    vesc.setSerialPort(&VESC_SERIAL);
    
    lastEventStartTime = millis();
    if (debug) Serial.println("Setup complete.");
}

// Update the current real time. Occasionlly check the internet. Keep track locally in between.
void updateTime() {

  unsigned long timeSinceLastUpdate = millis() - lastTimeUpdate;
  // Only synchonize from the internet occasionally.
  if (timeSinceLastUpdate > timeCheckInterval) {
    if (debug)
      Serial.println("About to check the current time from the web.");
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;
  
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    Udp.beginPacket(timeServer, 123); // NTP requests are to port 123
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
    if (debug) 
      Serial.println("Sent time check message. Waiting for a response from the server");
    bool packetReceived = true;
    int totalWaitTime = 0;
    while (!Udp.parsePacket()) {
      delay(100);
      totalWaitTime += 100;
  
      if (totalWaitTime > timeCheckTimeout) {
        packetReceived = false;
        if (debug)
          Serial.println("Time check timout exceeded. Failed to retrieve time.");
        break;
      }
    }

    if (packetReceived) {
      unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
      unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
      // combine the four bytes (two words) into a long integer
      // this is NTP time (seconds since Jan 1 1900):
      unsigned long secsSince1900 = highWord << 16 | lowWord;
      // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
      const unsigned long seventyYears = 2208988800UL;
      // subtract seventy years:
      lastEpochRetrieved = secsSince1900 - seventyYears;
      timeSinceLastUpdate = 0;
    }
    lastTimeUpdate = millis();
    Ethernet.maintain(); 
  }

  // In between updates, let the MCU keep track of time as best as it can.
  unsigned long epoch = lastEpochRetrieved + timeSinceLastUpdate;
  hourTime = ((epoch  % 86400L) / 3600 + timezoneOffsetUTC) % 24; // the hour (86400 equals secs per day)
  minuteTime = (epoch  % 3600) / 60;
  secondTime = epoch % 60;

  // print the hour, minute and second:
  if (debug && secondTime == 0) { // TODO I Want to print once per minute. I expect this will spam messages a few times at the beginning of each minute though.
    Serial.print("Hour: ");
    Serial.print(hourTime); 
    Serial.print(" Minute: "); 
    Serial.println(minuteTime);
  }
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
            // Don't check time or enforce curfew unless the system is idle.
            updateTime();

            if (hourTime > curfewStartTime || hourTime < curfewEndTime) {
              currentState = CURFEW;
              break;
            }
        
            // Be constantly checking the serial buffer for new RFID tag reads.
            // Unlock the window when a correct match occurs.
            if (authenticateRFID() >= 0) {
                currentState = UNLOCKING;
            }
            delay(LOOP_DELAY);
            break;

        case CURFEW:
            updateTime();
            if (hourTime < curfewStartTime && hourTime > curfewEndTime) {
              currentState = IDLE_CLOSED;
            }
            delay(LOOP_DELAY * 100);
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
            vesc.setRPM(RPM_OPENING);

            // Stop opening when the endstop is triggered or too much time elapses.
            if (digitalRead(PIN_ENDSTOP_OPEN) == LOW || (millis() - lastEventStartTime > windowMovementTimeout)) {
                if (debug) Serial.println("Window is done opening.");

                vesc.setCurrent(0.f);
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
            vesc.setRPM(RPM_CLOSING);

            // Stop opening when the endstop is triggered or too much time elapses. If a tag gets authenticated during
            // this time. Reopen the window for the duration we had been closing it (or until endstop).
            if (authenticateRFID() >= 0 || (ina260.readCurrent() > CURRENT_LIMIT && (millis() - lastEventStartTime) > 1000)) {
                if (debug) Serial.println("Tag authenticated during window closing. Reopening.");
                unsigned long timeToReopen = millis() - lastEventStartTime; // How much time have we been closing the window so far?
                lastEventStartTime = millis() - (windowMovementTimeout - timeToReopen); // Cheat the timeout. Set the current time to be in the future so the timeout triggers after the amount of time we had been closing the window elapses.
                currentState = OPENING; 
            } else if (digitalRead(PIN_ENDSTOP_CLOSE) == LOW || (millis() - lastEventStartTime > maxClosingTime)) {
                if (debug) Serial.println("Window is done closing.");

                vesc.setCurrent(0.f);
                currentState = LOCKING;
            }
            delay(LOOP_DELAY);
            break;
        default:
            if (debug) Serial.println("Unknown  Major bug.");
    }

}
