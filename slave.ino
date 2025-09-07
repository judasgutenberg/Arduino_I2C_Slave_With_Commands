/* Gus Mueller, extended Sept 2025
 * Arduino I2C slave with GPIO read/write and command handling
 * Compatible with Raspberry Pi Pico (RP2040 + Arduino-Pico core)
 */

#include "Wire.h"
#include "hardware/watchdog.h"   // optional if you later want HW watchdog
#include "pico/stdlib.h"         // for rp2040.reboot()



// ---- CONFIG ----
#define I2C_SLAVE_ADDR 20
#define REBOOT_PIN 7                  // pin used to reset master
#define WATCHDOG_TIMEOUT 60000UL      // 1 minute (ms)

// ---- STATE ----
volatile int receivedValue = 0;
volatile long dataToSend = -1;

unsigned long lastMasterSignal = 0;
unsigned long lastWatchdogPet = 0;

// forward declarations
void receieveEvent(int howMany);
void requestEvent();
void handleCommand(byte command, long value);
void writeWireLong(long val);

void setup() {
  pinMode(REBOOT_PIN, OUTPUT);
  digitalWrite(REBOOT_PIN, HIGH); // idle high

  Wire.begin(I2C_SLAVE_ADDR);   // Pico supports I2C slave in Philhower core
  Wire.onReceive(receieveEvent);
  Wire.onRequest(requestEvent);

  lastWatchdogPet = millis(); // start watchdog timer
}

void loop() {
  unsigned long now = millis();

  // watchdog check
  if ((now - lastWatchdogPet) > WATCHDOG_TIMEOUT) {
    // timeout -> toggle REBOOT_PIN low then high
    digitalWrite(REBOOT_PIN, LOW);
    delay(100);
    digitalWrite(REBOOT_PIN, HIGH);
    // reset watchdog timer so it doesn’t keep firing
    lastWatchdogPet = now;
  }
}

// ---- I2C callbacks ----

void requestEvent() {
  writeWireLong(dataToSend);
}

void receieveEvent(int howMany) {
  unsigned long now = millis();
  lastMasterSignal = now;

  byte byteCount = 0;
  byte byteCursor = 0;
  byte receivedValues[4];
  byte destination = 0;
  receivedValue = 0;

  while (Wire.available()) {
    byte byteRead = Wire.read();
    if (byteCount == 0) {
      destination = byteRead;
    } else {
      receivedValues[byteCursor++] = byteRead;
    }
    byteCount++;
  }

  // Build receivedValue from payload (if any)
  for (byte i = 0; i < byteCursor; i++) {
    receivedValue |= (receivedValues[i] << (8 * i));
  }

  if (destination >= 128) {
    // ---- Handle commands ----
    handleCommand(destination, receivedValue);
  } else if (byteCursor > 0) {
    // ---- Write to pin ----
    pinMode(destination, OUTPUT);
    if (receivedValue > 255) {
      analogWrite(destination, receivedValue - 256); 
    } else if (receivedValue == 0) {
      digitalWrite(destination, LOW);
    } else {
      digitalWrite(destination, HIGH);
    }
  } else {
    // ---- Read from pin ----
    if (destination > 63) {
      dataToSend = (long)analogRead(destination - 64);
    } else {
      pinMode(destination, INPUT);
      dataToSend = (long)digitalRead(destination);
    }
  }
}

// ---- Command handler ----
void handleCommand(byte command, long value) {
  switch (command) {
    case 128: // reboot slave
      rp2040.reboot();  // Pico-safe reboot
      break;

    case 129: // pet watchdog
      lastWatchdogPet = millis();
      break;

    // add more commands here:
    // case 130: doSomething(value); break;

    default:
      // unknown command
      break;
  }
}

// ---- Utility ----
void writeWireLong(long val) {
  byte buffer[4];
  buffer[0] = val >> 24;
  buffer[1] = val >> 16;
  buffer[2] = val >> 8;
  buffer[3] = val;
  Wire.write(buffer, 4);
}
