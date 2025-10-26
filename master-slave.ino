//this is code to be used by your master to store values in the EEPROM of your slave

#include <Wire.h>

// assume slave_i2c is defined somewhere in your config
extern uint8_t slave_i2c;

#define COMMAND_EEPROM_SETADDR 150
#define COMMAND_EEPROM_WRITE   151
#define COMMAND_EEPROM_READ    152
#define COMMAND_EEPROM_NORMAL  153

// ---- Write a string to slave EEPROM (optimized) ----
void writeStringToSlaveEEPROM(uint16_t eepromAddr, const char* str) {
    // Set EEPROM address
    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_SETADDR);
    Wire.write((eepromAddr >> 8) & 0xFF);
    Wire.write(eepromAddr & 0xFF);
    Wire.endTransmission();

    // Write string in chunks (Wire buffer max 32 bytes)
    const char* p = str;
    while (*p) {
        Wire.beginTransmission(slave_i2c);
        Wire.write(COMMAND_EEPROM_WRITE);

        uint8_t bytesInThisChunk = 0;
        while (*p && bytesInThisChunk < 30) { // 30 + 1 command byte = 31 < 32 limit
            Wire.write(*p++);
            bytesInThisChunk++;
        }

        Wire.endTransmission();
        delay(5); // give slave time to write EEPROM
    }

    // Null terminator
    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_WRITE);
    Wire.write(0);
    Wire.endTransmission();
    delay(5);

    // Return slave to normal mode
    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_NORMAL);
    Wire.endTransmission();
}

// ---- Read a string from slave EEPROM ----
void readStringFromSlaveEEPROM(uint16_t eepromAddr, char* buffer, size_t maxLen) {
    // Set EEPROM address
    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_SETADDR);
    Wire.write((eepromAddr >> 8) & 0xFF);
    Wire.write(eepromAddr & 0xFF);
    Wire.endTransmission();

    // Enable sequential read
    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_READ);
    Wire.endTransmission();

    size_t count = 0;
    while (count < maxLen - 1) {
        Wire.requestFrom(slave_i2c, (uint8_t)1); // read 1 byte at a time
        if (Wire.available()) {
            char c = Wire.read();
            if (c == 0) break;  // stop at null terminator
            buffer[count++] = c;
        } else {
            break; // no more data
        }
    }
    buffer[count] = '\0';

    // Return slave to normal mode
    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_NORMAL);
    Wire.endTransmission();
}



// ---- Write an int (2 bytes) ----
void writeIntToEEPROM(uint16_t addr, int value) {
    uint8_t bytes[2];
    bytes[0] = value & 0xFF;        // LSB
    bytes[1] = (value >> 8) & 0xFF; // MSB

    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_SETADDR);
    Wire.write((addr >> 8) & 0xFF);
    Wire.write(addr & 0xFF);
    Wire.endTransmission();

    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_WRITE);
    Wire.write(bytes[0]);
    Wire.write(bytes[1]);
    Wire.endTransmission();
    delay(5);

    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_NORMAL);
    Wire.endTransmission();
}

// ---- Read an int (2 bytes) ----
int readIntFromEEPROM(uint16_t addr) {
    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_SETADDR);
    Wire.write((addr >> 8) & 0xFF);
    Wire.write(addr & 0xFF);
    Wire.endTransmission();

    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_READ);
    Wire.endTransmission();

    int value = 0;
    Wire.requestFrom(slave_i2c, (uint8_t)2);
    if (Wire.available() >= 2) {
        value = Wire.read();
        value |= (Wire.read() << 8);
    }

    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_NORMAL);
    Wire.endTransmission();

    return value;
}

// ---- Write a long (4 bytes) ----
void writeLongToEEPROM(uint16_t addr, long value) {
    uint8_t bytes[4];
    bytes[0] = value & 0xFF;
    bytes[1] = (value >> 8) & 0xFF;
    bytes[2] = (value >> 16) & 0xFF;
    bytes[3] = (value >> 24) & 0xFF;

    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_SETADDR);
    Wire.write((addr >> 8) & 0xFF);
    Wire.write(addr & 0xFF);
    Wire.endTransmission();

    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_WRITE);
    Wire.write(bytes, 4);
    Wire.endTransmission();
    delay(5);

    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_NORMAL);
    Wire.endTransmission();
}

// ---- Read a long (4 bytes) ----
long readLongFromEEPROM(uint16_t addr) {
    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_SETADDR);
    Wire.write((addr >> 8) & 0xFF);
    Wire.write(addr & 0xFF);
    Wire.endTransmission();

    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_READ);
    Wire.endTransmission();

    long value = 0;
    Wire.requestFrom(slave_i2c, (uint8_t)4);
    if (Wire.available() >= 4) {
        value = Wire.read();
        value |= ((long)Wire.read() << 8);
        value |= ((long)Wire.read() << 16);
        value |= ((long)Wire.read() << 24);
    }

    Wire.beginTransmission(slave_i2c);
    Wire.write(COMMAND_EEPROM_NORMAL);
    Wire.endTransmission();

    return value;
}


// ---- Example usage ----
void setup() {
    Wire.begin(); // master
    Serial.begin(9600);

    writeStringToSlaveEEPROM(0x0000, "Hello, optimized I2C EEPROM!");

    char buf[50];
    readStringFromSlaveEEPROM(0x0000, buf, sizeof(buf));
    Serial.println(buf);
}

void loop() {
    // nothing
}

