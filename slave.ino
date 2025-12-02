#include <Wire.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <EEPROM.h> // needed for EEPROM read/write

// ---- CONFIG ----
#define I2C_SLAVE_ADDR 20
#define REBOOT_PIN 7              // pin used to reset master

// Existing watchdog commands
#define COMMAND_REBOOT 128
#define COMMAND_MILLIS 129
#define COMMAND_LASTWATCHDOGREBOOT 130
#define COMMAND_WATCHDOGREBOOTCOUNT 131
#define COMMAND_LASTWATCHDOGPET 132
#define COMMAND_LASTPETATBITE 133
#define COMMAND_REBOOTMASTER 134
#define COMMAND_WATCHDOGPETBASE 200

// New EEPROM-style commands
#define COMMAND_EEPROM_SETADDR 150   // set pointer for read/write
#define COMMAND_EEPROM_WRITE    151  // sequential write mode
#define COMMAND_EEPROM_READ     152  // sequential read mode
#define COMMAND_EEPROM_NORMAL   153  // exit EEPROM mode, back to default behavior

#define EEPROM_SIZE 1024

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
volatile byte deferredCommand = 0;

// EEPROM mode state
byte eepromMode = 0;             // 0 = normal, 1 = write, 2 = read
unsigned int eepromAddress = 0;  // pointer within 0..1023

// forward declarations
void receiveEvent(int howMany);
void requestEvent();
void handleCommand(byte command, long value);
void writeWireLong(long val);

// ---- Early init hook ----
extern "C" void initVariant(void) {
    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
}

// ---- Setup ----
void setup() {
    pinMode(REBOOT_PIN, OUTPUT);
    digitalWrite(REBOOT_PIN, HIGH); // idle high

    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    lastWatchdogPet = millis(); // start watchdog timer
}

void loop() {
    unsigned long now = millis();

    if(millis()-timeLastPrinted > 2000) {
        // debug prints (optional)
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
        rebootMaster();
        lastPetAtBite = secondsLate;
        lastWatchdogPet = now; // allow master to start
        lastWatchdogReboot = now;
    }

    if(deferredCommand == COMMAND_REBOOT) { // async reboot
        software_reset();
        deferredCommand = 0;
    }
}

void rebootMaster() {
    rebootCount++;
    digitalWrite(REBOOT_PIN, LOW);
    delay(100);
    digitalWrite(REBOOT_PIN, HIGH);
}

// ---- I2C Callbacks ----
void requestEvent() {
    if (eepromMode == 2) { // sequential EEPROM read
        byte val = EEPROM.read(eepromAddress);
        Wire.write(val);
        eepromAddress = (eepromAddress + 1) % EEPROM_SIZE;
    } else {
        writeWireLong(dataToSend); // normal behavior
    }
}

void receiveEvent(int howMany) {
    if (howMany < 1) return;

    byte command = Wire.read();
    long value = 0;

    int bytesRead = 0;
    byte buffer[4];

    while (Wire.available() && bytesRead < 4) {
        buffer[bytesRead++] = Wire.read();
    }

    for (int i = 0; i < bytesRead; i++) {
        value |= ((long)buffer[i] << (8 * i));
    }

    // ---- EEPROM Commands ----
    if (command >= COMMAND_EEPROM_SETADDR && command <= COMMAND_EEPROM_NORMAL) {
        switch (command) {
            case COMMAND_EEPROM_SETADDR:
                eepromAddress = value & 0x03FF; // 0..1023
                break;

            case COMMAND_EEPROM_WRITE:
                eepromMode = 1;
                if (bytesRead > 0) {
                    for (int i = 0; i < bytesRead; i++) {
                        EEPROM.write(eepromAddress, buffer[i]);
                        eepromAddress = (eepromAddress + 1) % EEPROM_SIZE;
                    }
                }
                break;

            case COMMAND_EEPROM_READ:
                eepromMode = 2;
                break;

            case COMMAND_EEPROM_NORMAL:
                eepromMode = 0;  // back to normal behavior
                break;
        }
        return; // handled, skip other logic
    }

    // ---- Existing pin/write logic ----
    if (command >= 128) {
        handleCommand(command, value);
    } else if (bytesRead > 0) {
        pinMode(command, OUTPUT);
        if (value == 0) {
            digitalWrite(command, LOW);
        } else if (value > 255) {
            analogWrite(command, value - 256);
        } else {
            digitalWrite(command, HIGH);
        }
    } else {
        if (command > 63) {
            dataToSend = analogRead(command - 64);
        } else {
            pinMode(command, INPUT);
            dataToSend = digitalRead(command);
        }
    }
}

// ---- Command Handler ----
void handleCommand(byte command, long value) {
    switch (command) {
        case COMMAND_REBOOT:
            deferredCommand = command;
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

        case COMMAND_REBOOTMASTER:
            TWCR &= ~(_BV(TWEN));
            delay(100);
            rebootMaster();
            delay(200);
            TWCR |= _BV(TWEN);
            break;

        default:
            if(command > 199 && command < 210) {
                byte watchdogTimingIndication = command - COMMAND_WATCHDOGPETBASE;
                if(lastTimeoutScale != watchdogTimingIndication) {
                    watchdogTimeout = 1;
                    for (byte i = 0; i < watchdogTimingIndication; i++) {
                        watchdogTimeout *= 10;
                    }
                    lastTimeoutScale = watchdogTimingIndication;
                }
                lastWatchdogPet = millis();
            }
            break;
    }
}

// ---- Software Reset ----
void software_reset(void) {
    return; // disabled for safety
    cli();
    TWCR = 0;
    DDRC &= ~((1<<PC4) | (1<<PC5));
    PORTC |= (1<<PC4) | (1<<PC5);
    wdt_enable(WDTO_15MS);
    while(1) { }
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
