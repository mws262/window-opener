#define RFID_SERIAL Serial1
#define VESC_SERIAL Serial5

#include <VescUart.h>
#include <SPI.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

/* General operating parameters */
const bool enforceCurfew = false;
const uint8_t curfewStartTime = 22; // 10PM
const uint8_t curfewEndTime = 7;
const bool debug = true;
const int LOOP_DELAY = 50; // Too shaort and entire messages might not come in in time between loops. One downside of polling everything.
enum State {
    IDLE_CLOSED,
    OPENING,
    WAITING_OPEN,
    CLOSING,
    CURFEW
};

// Start in closing mode, so if power is lost, the window can't get stuck open.
State currentState = CLOSING;

/* Vesc motor control parameters */
VescUart vesc;
const float CURRENT_LIMIT = 1.0; // TODO tune this.
const float MOTOR_DUTY_BASELINE = 0.04;
const float MOTOR_DUTY_SLOW = 0.03;
long lastTach = 0;

  float tachFullScale = 1800;
  float currTach = 0.0;
  float minDuty = 0.03;
  float maxDuty = 0.15;
  float slowPoint = tachFullScale/12.;

  float tachClosed = 0.0;
  float tachTarget = 0.0;

  float catTach = tachFullScale / 2.0;
  float dogTach = tachFullScale;
  float buttonTach = tachFullScale;
 

/* RFID parameters */
const int NUM_CATS = 2;
const int NUM_DOGS = 2;
const int MAX_ID_LEN = 20;
const char catAuthID[NUM_CATS][MAX_ID_LEN] = {"F98200041199001878", // Tonks.
                                           "985141002571129"};
const char dogAuthID[NUM_DOGS][MAX_ID_LEN] = {"F9000202060000037A", // Piper collar.
                                           "todoPIPEREmbedded"};                                           

/* Switch parameters */
// Endstop pins.
const int PIN_ENDSTOP_CLOSE = 18;
const int PIN_ENDSTOP_OPEN = 19;

// Momentary switch to open the window without a tag read.
const int MANUAL_OPEN_PIN = 17;

unsigned long lastEventStartTime;
const int stayOpenDuration = 4000; // Milliseconds.
const int windowMovementTimeout = 15000; // No single motion should take longer than this.


/* Parameters for synchronizing clock via ethernet */
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

/* Initiate connection to online clock. */
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
    if (debug) Serial.begin(9600); // Default serial is for communicating with the computer for debugging. Should eventually be eliminated.

    RFID_SERIAL.begin(9600); // Serial 1 is for communication with the RFID board. Information only flows in from it.
    VESC_SERIAL.begin(19200);
    while (!VESC_SERIAL) {;}
    vesc.setSerialPort(&VESC_SERIAL);

    pinMode(PIN_ENDSTOP_CLOSE, INPUT);
    pinMode(PIN_ENDSTOP_OPEN, INPUT);
    pinMode(MANUAL_OPEN_PIN, INPUT);

    setupEthernet();

    lastEventStartTime = millis();
    if (debug) Serial.println("Setup complete.");
}

// Update the current real time. Occasionlly check the internet. Keep track of time locally in between.
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
  
    if (RFID_SERIAL.available()) {
      char id[64];
      char byteRead;
      
      int availableBytes = RFID_SERIAL.available();
      for (int i = 0; i < availableBytes; i++) {
         id[i] = RFID_SERIAL.read();
         id[i+1] = '\0'; // Append a null
      }

        if (debug) Serial.println(id);
        for (int i = 0; i < NUM_CATS; i++) {
            if (strstr(id, catAuthID[i])) {
                if (debug) {
                    Serial.print(catAuthID[i]);
                    Serial.println(" identified!");
                }
                tachTarget = tachClosed + catTach;
                return 1;
            }
        }

        for (int i = 0; i < NUM_DOGS; i++) {
            if (strstr(id, dogAuthID[i])) {
                if (debug) {
                    Serial.print(dogAuthID[i]);
                    Serial.println(" identified!");
                }
                tachTarget = tachClosed + dogTach;
                return 2;
            }
        }
    } else {
      if (digitalRead(MANUAL_OPEN_PIN) == LOW) { // Basically treat the switch as another tag.
        if (debug) {
          Serial.print("switch triggered");
        }
        tachTarget = tachClosed + buttonTach;
        return 999;
      }
    }
    return -1;
}

void loop() {
  vesc.getVescValues();
  float currTach = vesc.data.tachometer;

    switch(currentState) {
        case IDLE_CLOSED:
            vesc.setCurrent(0.);
            float tachClosed = currTach; // estimated position ticks in closed position.
            
            // Don't check time or enforce curfew unless the system is idle.
            updateTime();

            if (hourTime > curfewStartTime || hourTime < curfewEndTime) {
              currentState = CURFEW;
              break;
            }
        
            // Be constantly checking the serial buffer for new RFID tag reads.
            // Unlock the window when a correct match occurs.
            if (authenticateRFID() >= 0) {
              lastEventStartTime = millis();
              currentState = OPENING;
            }
            delay(LOOP_DELAY);
            break;

        case CURFEW:
            updateTime();
            if (hourTime < curfewStartTime && hourTime > curfewEndTime) {
              lastEventStartTime = millis();
              currentState = IDLE_CLOSED;
            }
            delay(LOOP_DELAY * 100);
            break;
            
        case OPENING:

            if (digitalRead(PIN_ENDSTOP_OPEN) || currTach >= tachTarget || (millis() - lastEventStartTime > windowMovementTimeout)) {
              if (debug) Serial.println("Window is done opening.");
              currentState = WAITING_OPEN;
              lastEventStartTime = millis();
            } else if (currTach - tachClosed > slowPoint) {
               vesc.setDuty(-(currTach - tachClosed - slowPoint)/(tachFullScale - slowPoint) * (maxDuty - minDuty) + maxDuty);
            } else {
               vesc.setDuty(maxDuty);
            }
            delay(LOOP_DELAY);
            break;

        case WAITING_OPEN:
            // Wait for awhile with the window open to give time for the slowpokes to make up their minds. If a news
            // valid authentication occurs, reset the duration counter.
            if (authenticateRFID() >= 0) {
                if (debug) Serial.println("Tag detected during window open time.");
                currentState = OPENING;
                lastEventStartTime = millis();
            } else if (millis() - lastEventStartTime > stayOpenDuration) { // Time limit met for closing to occur.
                if (debug) Serial.println("Done waiting with the window open.");
                currentState = CLOSING;
                lastEventStartTime = millis();
            }
            delay(LOOP_DELAY);
            break;

        case CLOSING:
            // Stop opening when the endstop is triggered or too much time elapses. If a tag gets authenticated during
            // this time. Reopen the window for the duration we had been closing it (or until endstop).
            if (currTach - tachClosed <= 0.4 * tachFullScale) {
              vesc.setDuty(-MOTOR_DUTY_SLOW);
            } else {
              vesc.setDuty(-MOTOR_DUTY_BASELINE);
            }
            
            int tagCheck = authenticateRFID();
            if (tagCheck >= 0 || ( vesc.data.avgMotorCurrent > CURRENT_LIMIT && (millis() - lastEventStartTime) > 1000)) { // currTach - lastTach <= tachFullScale * 0.6
                if (debug) Serial.println("Tag authenticated during window closing. Reopening.");
                lastEventStartTime = millis();
                currentState = OPENING; 
            } else if (digitalRead(PIN_ENDSTOP_CLOSE) || (millis() - lastEventStartTime > windowMovementTimeout)) {
                lastEventStartTime = millis();
                currentState = IDLE_CLOSED;
                if (debug) Serial.println("Window is done closing.");
            }
            
            delay(LOOP_DELAY);
            break;
        default:
            if (debug) Serial.println("Unknown  Major bug.");
    }
}
