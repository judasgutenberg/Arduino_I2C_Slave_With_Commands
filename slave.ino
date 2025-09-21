#include <Wire.h>

// ---- CONFIG ----
#define I2C_SLAVE_ADDR 20
#define REBOOT_PIN 7              // pin used to reset master
#define COMMAND_REBOOT 128
#define COMMAND_MILLIS 129
#define COMMAND_LASTWATCHDOGREBOOT 130
#define COMMAND_WATCHDOGREBOOTCOUNT 131
#define COMMAND_LASTWATCHDOGPET 132
#define COMMAND_LASTPETATBITE 133
#define COMMAND_WATCHDOGPETBASE 200

// ---- STATE ----
volatile long receivedValue = 0;
volatile long dataToSend = -1;
volatile unsigned long lastWatchdogPet = 0;
volatile unsigned long lastWatchdogReboot = 0;
volatile unsigned int lastTimeoutScale = 0;
volatile unsigned long lastPetAtBite = 0;

volatile unsigned long watchdogTimeout = 200;
volatile unsigned int rebootCount = 0;
volatile unsigned long timeLastPrinted = 0;

// forward declarations
void receiveEvent(int howMany);
void requestEvent();
void handleCommand(byte command, long value);
void writeWireLong(long val);

void setup() {
  pinMode(REBOOT_PIN, OUTPUT);
  digitalWrite(REBOOT_PIN, HIGH); // idle high

  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  //Serial.begin(115200);
  lastWatchdogPet = millis(); // start watchdog timer
}

void loop() {
  unsigned long now = millis();
  if(millis()-timeLastPrinted > 2000) {
    /*
    Serial.print(now);
    Serial.print(" ");
    Serial.print(lastWatchdogPet);
    Serial.print(" ");
    Serial.print((now-lastWatchdogPet)/1000);
    Serial.print(" ");
    Serial.println(watchdogTimeout);
    Serial.print("\n");
    */
    timeLastPrinted = millis();
  }
  unsigned long secondsLate = (now - lastWatchdogPet)/1000;
  if (secondsLate > watchdogTimeout && secondsLate < 40000) {
    rebootCount++;
    //Serial.println("REBOOT!!");
    // timeout -> toggle REBOOT_PIN low then high
    digitalWrite(REBOOT_PIN, LOW);
    delay(100);
    digitalWrite(REBOOT_PIN, HIGH);
    lastPetAtBite = secondsLate;
    //lastWatchdogPet = now; // reset watchdog timer
    lastWatchdogReboot = now;
    
  }
}

// ---- I2C callbacks ----

void requestEvent() {
  writeWireLong(dataToSend);
}


void receiveEvent(int howMany) {
  unsigned long now = millis();
  //lastWatchdogPet = now;  // any I2C traffic counts as watchdog pet //maybe not

  if (howMany < 1) return; // ignore empty packets

  byte command = Wire.read();
  long value = 0;

  int bytesRead = 0;
  byte buffer[4];

  while (Wire.available() && bytesRead < 4) {
    buffer[bytesRead++] = Wire.read();
  }

  // reconstruct long value from bytes (little-endian)
  for (int i = 0; i < bytesRead; i++) {
    value |= ((long)buffer[i] << (8 * i));
  }

  if (command >= 128) {
    handleCommand(command, value);

    // Always give master a safe response in case it does a requestFrom
    //dataToSend = 0;  

  } else if (bytesRead > 0) {
    // ---- Write to pin ----
    pinMode(command, OUTPUT);
    if (value == 0) {
      digitalWrite(command, LOW);
    } else if (value > 255) {
      analogWrite(command, value - 256);
    } else {
      digitalWrite(command, HIGH);
    }

  } else {
    // ---- Read from pin ----
    if (command > 63) {
      dataToSend = analogRead(command - 64);
    } else {
      pinMode(command, INPUT);
      dataToSend = digitalRead(command);
    }
  }
}




// ---- Command handler ----
void handleCommand(byte command, long value) {
  //Serial.print("command: ");
  Serial.println(command);
  switch (command) {
    case COMMAND_REBOOT:
      lastWatchdogReboot = millis();
      asm volatile ("jmp 0"); // software reset for AVR
      break;

    case COMMAND_MILLIS:
      dataToSend = millis();
      break;

    case COMMAND_LASTWATCHDOGREBOOT:
      dataToSend = lastWatchdogReboot;
      break;

    case COMMAND_WATCHDOGREBOOTCOUNT:
      dataToSend = rebootCount;
      break;

    case COMMAND_LASTPETATBITE:
      dataToSend = lastPetAtBite;
      break;
      
    case COMMAND_LASTWATCHDOGPET:
      dataToSend = lastWatchdogPet;
      break;

    default:
      if(command > 199 && command < 210) {
        
        byte watchdogTimingIndication =  command - COMMAND_WATCHDOGPETBASE;
        
        if(lastTimeoutScale != watchdogTimingIndication) {
          watchdogTimeout = 1;
          for (byte i = 0; i < watchdogTimingIndication; i++) { //better than pow()
            watchdogTimeout *= 10;
          }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
          lastTimeoutScale = watchdogTimingIndication;
        }
        lastWatchdogPet = millis();
      }
      break;
  }
}

// ---- Utility ----
void writeWireLong(long val) {
  byte buffer[4];
  buffer[0] = val & 0xFF;
  buffer[1] = (val >> 8) & 0xFF;
  buffer[2] = (val >> 16) & 0xFF;
  buffer[3] = (val >> 24) & 0xFF;
  Wire.write(buffer, 4);
}
