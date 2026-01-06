#include <Wire.h>
#include <TimeLib.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <EEPROM.h> // needed for EEPROM read/write

#define VERSION 2023

#define INT_CONFIGS 10

#define RX_SIZE 100
#define TX_SIZE 60


//Indexes into integer configuration array
#define PARSE_MODE      5
#define BAUD_RATE_LEVEL 6
#define I2C_ADDRESS     7
#define REBOOT_PIN      8
#define SERIAL_MODE     9

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
#define COMMAND_EEPROM_SETADDR  150   // set pointer for read/write
#define COMMAND_EEPROM_WRITE    151  // sequential write mode
#define COMMAND_EEPROM_READ     152  // sequential read mode
#define COMMAND_EEPROM_NORMAL   153  // exit EEPROM mode, back to default behavior

#define COMMAND_VERSION         160
#define COMMAND_COMPILEDATETIME 161
#define COMMAND_TEMPERATURE     162
#define COMMAND_FREEMEMORY      163
#define COMMAND_GET_SLAVE_CONFIG 164


//serial commands
#define COMMAND_SERIAL_SET_BAUD_RATE        170
#define COMMAND_RETRIEVE_SERIAL_BUFFER      171
#define COMMAND_POPULATE_SERIAL_BUFFER      172
#define COMMAND_GET_LAST_PARSE_TIME         173
#define COMMAND_GET_PARSED_SERIAL_DATA      174
#define COMMAND_SET_PARSED_OFFSET           175
#define COMMAND_GET_PARSED_DATUM            176
#define COMMAND_GET_PARSE_CONFIG_NUMBER     177
#define COMMAND_GET_PARSED_PACKET_SIZE      178
#define COMMAND_SET_SERIAL_MODE             179

#define COMMAND_SET_UNIX_TIME               180
#define COMMAND_GET_UNIX_TIME               181
#define COMMAND_GET_CONFIG                  182
#define COMMAND_SET_CONFIG                  183

#define EEPROM_SIZE 1024
#define SLAVE_CONFIG 512  //where local slave configs live

#define MAX_BLOCKS 3
#define MAX_ADDRS  3
#define MAX_OFFSETS 8

#define MAX_CFG_LEN 120
#define PARSED_BUF_MAX 30   // bytes (15 values)

uint8_t parsedBuf[PARSED_BUF_MAX];// = {0xff, 0xff, 0x04, 0x00, 0x0a, 0x00, 0x01, 0x10}; //a packed register for returning parsed serial values
uint8_t parsedStringPacketLen = 0;
uint8_t parsedStringConfigCount = 0; //determined by actually looking at the EEPROM

struct ConfigBlock {
  char start[32];
  char end[32];
  uint8_t addrCount;
  char addr[MAX_ADDRS][12];
  uint8_t offsets[MAX_ADDRS][MAX_OFFSETS];
  uint8_t offsetCount[MAX_ADDRS];
};

//sample block population from a config like so: Characteristic #2|0x3ffbb61c|0x3ffbb5fc|4|5|6|7|8|9|10|11|0x3ffbb60c|0|1
/*

start = "Characteristic #2"
end   = "0x3ffbb61c"
addr[0] = "0x3ffbb5fc"
offsets[0] = {4,5,6,7,8,9,10,11}
addr[1] = "0x3ffbb60c"
offsets[1] = {0,1}
offsetCount[1] = ??
 */

ConfigBlock blocks[MAX_BLOCKS];
uint8_t blockCount = 0;
int8_t activeBlock = -1;

struct CircularBuffer {
  uint8_t* data;
  uint16_t size;
  volatile uint16_t head; // modified in ISR / main
  volatile uint16_t tail;
  volatile uint16_t count;
};


// ---- CONFIG ----

//for config items stored locally in the EEPROM
uint16_t cis[INT_CONFIGS];

uint8_t rxStorage[RX_SIZE];
uint8_t txStorage[TX_SIZE];
            // pin used to reset master

CircularBuffer rxBuffer = { rxStorage, RX_SIZE, 0, 0, 0 };
CircularBuffer txBuffer = { txStorage, TX_SIZE, 0, 0, 0 };

// ---- STATE ----
volatile long receivedValue = 0;
volatile unsigned long dataToSend = 0;
volatile unsigned long lastWatchdogPet = 0;
volatile unsigned long lastWatchdogReboot = 0;
volatile unsigned int lastTimeoutScale = 0;
volatile unsigned long lastPetAtBite = 0;

volatile unsigned long watchdogTimeout = 200;
volatile unsigned int rebootCount = 0;
 
volatile byte deferredCommand = 0;
volatile byte eepromReadCount = 1; 

volatile uint8_t eepromWriteBuffer[32];
volatile uint8_t eepromWriteLength = 0;
volatile bool eepromWritePending = false;
volatile bool retrieveSerialData = false;
volatile bool retrieveParsedSerialData = false;
 

volatile uint32_t unixTime = 0;
volatile uint32_t lastDataParseTime = 0;

volatile uint8_t parsedReadOffset = 0;

// EEPROM mode state
byte eepromMode = 0;             // 0 = normal, 1 = write, 2 = read
unsigned int eepromAddress = 0;  // pointer within 0..1023
 
// forward declarations
void receiveEvent(int howMany);
void requestEvent();
void handleCommand(byte command, uint32_t value);
void writeWireLong(long val);

// ---- Setup ----
void setup() {
    initDefaultConfig();
    //Serial.begin(115200);
    initSlaveConfigFromEeprom();//might not do anything good
 
    //while (Serial.available()) Serial.read(); 
    Serial.println("Started");

    memset(rxStorage, 0, RX_SIZE);
    rxBuffer.head = rxBuffer.tail = rxBuffer.count = 0;
    
    memset(txStorage, 0, TX_SIZE);
    txBuffer.head = txBuffer.tail = txBuffer.count = 0;

    Wire.begin((byte)cis[I2C_ADDRESS]);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    
    pinMode(cis[REBOOT_PIN], OUTPUT);
    digitalWrite(cis[REBOOT_PIN], HIGH); // idle high
    setupTimer1();
    lastWatchdogPet = millis(); // start watchdog timer
}

void loop() {
    unsigned long now = millis();
    //Serial.println(now);
    if(cis[BAUD_RATE_LEVEL] > 0 && cis[SERIAL_MODE] == 2) {
      processSerialStream();
    }
    
    if (eepromWritePending) {
      eepromWritePending = false;
      noInterrupts();
      for (uint8_t i = 0; i < eepromWriteLength; i++) {
        yield();
        //Serial.print((char)eepromWriteBuffer[i]); 
        EEPROM.update(eepromAddress, eepromWriteBuffer[i]);
        eepromAddress = (eepromAddress + 1) % EEPROM_SIZE;
        //delay(20);
        yield();
      }
      interrupts();
      //Serial.println("*");
    }

    if(cis[BAUD_RATE_LEVEL] > 0 && cis[SERIAL_MODE] == 1) {
     // Serial.println("yer");
      cbSendLatest(txBuffer);
      
      if (Serial.available()) {
        //noInterrupts();
        uint8_t b = Serial.read();
        cbPutByte(rxBuffer, b, 1);
        //delay(1);
        yield();
        //interrupts();
      }
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

ISR(TIMER1_COMPA_vect) {
  //most accurate way to do this:
  if(unixTime > 0) { //don't increment unixTime if it was not set
    unixTime++;
  }
}

void setupTimer1() {
    cli();

    TCCR1A = 0;
    TCCR1B = 0;

    OCR1A = 15624;               // 1 second
    TCCR1B |= (1 << WGM12);      // CTC mode
    TCCR1B |= (1 << CS12) | (1 << CS10); // prescaler 1024
    TIMSK1 |= (1 << OCIE1A);     // enable compare interrupt

    sei();
}

void rebootMaster() {
    rebootCount++;
    digitalWrite(cis[REBOOT_PIN], LOW);
    delay(100);
    digitalWrite(cis[REBOOT_PIN], HIGH);
}

// ---- I2C Callbacks ----
void requestEvent() {
  if (eepromMode == 2) {
    for (int i = 0; i < eepromReadCount; i++) {
      Wire.write(EEPROM.read(eepromAddress));
      //Serial.print(eepromAddress);
      //Serial.print(": ");
      //Serial.println(EEPROM.read(eepromAddress));
      eepromAddress = (eepromAddress + 1) % EEPROM_SIZE;
    }
    // Optional: reset mode
    eepromMode = 0;
  } else if (retrieveSerialData) {
    //Serial.println("SENDY!");
    cbSendLatestWire(rxBuffer, 32);
    retrieveSerialData = false;
  } else if (retrieveParsedSerialData) {
    //Serial.println("parsedStringPacketLen: ");
    //Serial.println(parsedStringPacketLen);
    //parsedStringPacketLen = 60;
    if(parsedStringPacketLen > 0){      
      uint8_t remaining = parsedStringPacketLen - parsedReadOffset;
      uint8_t chunk = remaining > 30 ? 30 : remaining;
      
      for (uint8_t i = 0; i < chunk; i++) {
        Wire.write(parsedBuf[parsedReadOffset + i]);
        /*
        Serial.print("in that loop: ");
        Serial.print(parsedStringPacketLen);
        Serial.print("; ");
        Serial.print(parsedReadOffset);
        Serial.print("; ");
        
        Serial.print(parsedReadOffset + i);
        Serial.print(": ");
        Serial.println(parsedBuf[parsedReadOffset + i]);
        */
      }
   
      parsedReadOffset += chunk;
    
      if (parsedReadOffset >= parsedStringPacketLen) {
        retrieveParsedSerialData = false;  // done
      }

    } else {
      retrieveParsedSerialData = false;  // done
    }
  } else {
    writeWireLong(dataToSend);
  }
}

void receiveEvent(int howMany) {
    if (howMany < 1) {
      return;
    }

    byte command = Wire.read();
    byte ordinalLocation = 0;
    uint32_t value = 0;
    //Serial.println(command);
    int bytesRead = 0;
    byte buffer[32];
    //Serial.println(command);
    while (Wire.available() && bytesRead < 32) {
        byte val = Wire.read();
        buffer[bytesRead++] = val;
        //Serial.print("val: ");
        //Serial.println(val);
    }
    uint8_t valueByteStart = 0;
    //if we issue a COMMAND_SET_CONFIG then the first byte after that command is an ordinal into the config array
    //and all the rest is the value that is to be set. this can be used for other purposes, but for now we just use it for this
    if(command == COMMAND_SET_CONFIG) { 
      ordinalLocation = buffer[0];
      valueByteStart = 1;
    }
    for (uint8_t i = valueByteStart; i < bytesRead; i++) {
        value |= ((long)buffer[i] << (8 * (i-valueByteStart)));
    }

    if (command == COMMAND_POPULATE_SERIAL_BUFFER) {
      //Serial.println("populating serial buffer");
      cbPutArray(txBuffer, buffer, bytesRead);
    } else if (command == COMMAND_RETRIEVE_SERIAL_BUFFER) {
      retrieveSerialData = true;
    } else if (command == COMMAND_FREEMEMORY) {
      dataToSend = (unsigned long)freeMemory();
    } else if (command == COMMAND_SET_UNIX_TIME) {
      unixTime = value;
    } else if (command == COMMAND_GET_UNIX_TIME) {
      dataToSend = unixTime;
    } else if (command == COMMAND_GET_PARSE_CONFIG_NUMBER) {
      dataToSend = parsedStringConfigCount;
    } else if (command == COMMAND_GET_PARSED_PACKET_SIZE) {
      dataToSend = parsedStringPacketLen;
    } else if (command == COMMAND_SET_SERIAL_MODE) {
      cis[SERIAL_MODE] = value;
    } else if (command == COMMAND_GET_CONFIG) {
      //Serial.println(cis[value]);
      dataToSend = cis[value]; //so far we can only retrieve individual integer config items
      //Serial.println(dataToSend);
    } else if (command == COMMAND_SET_CONFIG) {
      //Serial.println(ordinalLocation);
      //Serial.println(value);
      cis[ordinalLocation] = value; //so far we can only set individual integer config items
    } else if (command == COMMAND_GET_LAST_PARSE_TIME) {
      dataToSend = lastDataParseTime;
    } else if (command == COMMAND_GET_SLAVE_CONFIG) {
      dataToSend = SLAVE_CONFIG;
    } else if (command == COMMAND_SET_PARSED_OFFSET) {
      parsedReadOffset = value;
    } else if (command == COMMAND_GET_PARSED_SERIAL_DATA) {
      retrieveParsedSerialData = true;
    } else if (command == COMMAND_GET_PARSED_DATUM) {
      uint16_t retrieved = 0;
      if(value + 1 < PARSED_BUF_MAX) {
        uint8_t byteNumber = 0;
        for(uint8_t i = value * 2; i< (value* 2) + 2; i++) { //all the values are ints
          Serial.print((int)i);
          Serial.print(": ");
          Serial.println(parsedBuf[i]);
          retrieved |= ((uint16_t)parsedBuf[i] << (8 * byteNumber));
          byteNumber++;
        }
      }
      dataToSend = (uint32_t)retrieved;
    // ---- EEPROM Commands ----
    } else if (command >= COMMAND_EEPROM_SETADDR && command <= COMMAND_EEPROM_NORMAL) {
        switch (command) {
            case COMMAND_EEPROM_SETADDR:
                eepromAddress = value & 0x03FF; // 0..1023
                break;

            case COMMAND_EEPROM_WRITE:
                eepromMode = 1;
                if (bytesRead > 0) {
                    for (int i = 0; i < bytesRead; i++) {
                        //EEPROM.update(eepromAddress, buffer[i]);
                        //eepromAddress = (eepromAddress + 1) % EEPROM_SIZE;
                        //Serial.print((char)buffer[i]); 
                        eepromWriteBuffer[i] = buffer[i];
                        
                        eepromWriteLength = bytesRead;
                        
                    }
                    //Serial.println("^");
                    eepromWritePending = true;
                }
                
                break;
                
            case COMMAND_EEPROM_READ:
                //Serial.print("EEPROM READ MODE");
                //Serial.print(": ");
                //Serial.println((int)value);
                eepromMode = 2;
                eepromReadCount = value;
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
void handleCommand(byte command, uint32_t value) {
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

        case COMMAND_COMPILEDATETIME:
            dataToSend = compileUnixTime();
            break;

        case COMMAND_VERSION:
            dataToSend = VERSION;
            break;

        case COMMAND_TEMPERATURE:
            dataToSend = readInternalTemp;
            break;

        case COMMAND_SERIAL_SET_BAUD_RATE:
          setSerialRate(value);
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
                    //Serial.println(value);
                    if(value > 0) {
                      unixTime = value;
                    }
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

bool isInteger(const char *s) {
  if (s == NULL || *s == '\0') return false;

  unsigned int i = 0;

  // Optional sign
  if (s[0] == '-' || s[0] == '+') {
    if (s[1] == '\0') return false; // only "+" or "-"
    i = 1;
  }

  for (; s[i] != '\0'; i++) {
    if (!isDigit(s[i])) return false;
  }

  return true;
}

void writeWireLong(long val) {
    byte buffer[4];
    buffer[0] = val & 0xFF;
    buffer[1] = (val >> 8) & 0xFF;
    buffer[2] = (val >> 16) & 0xFF;
    buffer[3] = (val >> 24) & 0xFF;
    Wire.write(buffer, 4);
}

unsigned long compileUnixTime() {
  // __DATE__ = "Dec 04 2025"
  // __TIME__ = "14:37:05"
  const char monthNames[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char monthStr[4];
  strncpy(monthStr, __DATE__, 3);
  monthStr[3] = '\0';
  int month = (strstr(monthNames, monthStr) - monthNames) / 3 + 1;
  int day = atoi(__DATE__ + 4);
  int year = atoi(__DATE__ + 7);
  int hour = atoi(__TIME__);
  int minute = atoi(__TIME__ + 3);
  int second = atoi(__TIME__ + 6);
  tmElements_t tm;
  tm.Year = year - 1970; // TimeLib counts from 1970
  tm.Month = month;
  tm.Day = day;
  tm.Hour = hour;
  tm.Minute = minute;
  tm.Second = second;
  return makeTime(tm);
}

long readInternalTemp() {
  // Enable the temperature sensor
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
  ADCSRA |= _BV(ADEN);
  delay(5); // Allow sensor to settle
  // Fake conversion to settle multiplexer
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC));
  // Real conversion
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC));
  uint16_t adc = ADC;
  // Convert according to datasheet typical values
  // Note: Calibration varies per-chip.
  // slope ˜ 1.22 mV/°C
  // offset ˜ 314 mV @ 25°C
  //
  // Convert ADC?mV using 1.1V reference:
  // mV = adc * 1100 / 1024
  long millivolts = (long)adc * 1100L / 1024L;
  // Convert mV to °C × 10:
  // C = (mV - 314) / 1.22 + 25
  // C10 = 10 * C
  long C10 = ((millivolts - 314) * 100L / 122L) + 250;
  return (int16_t)C10;
}



////////////////////////////////////////
//serial functions
////////////////////////////////////////

uint32_t setSerialRate(byte baudRateLevel) {
  uint32_t baudRates[] = {
    0,
    300,
    1200,
    2400,
    4800,
    9600,
    19200,
    38400,
    57600,
    115200,
    230400,
    250000,
    500000,
    1000000,
    2000000
  };
  uint32_t newBaud = baudRates[baudRateLevel];
 
  if (newBaud > 0) {
    Serial.end();
    delay(2);
    Serial.begin(newBaud);
    delay(2);

    while (Serial.available()) {
      Serial.read();    
    }
  }
  return newBaud;
}



void cbSendLatest(CircularBuffer &cb) {
  for (uint16_t i = 0; i < cb.count; i++) {
    Serial.write(cb.data[cb.tail]);
    cb.tail = (cb.tail + 1) % cb.size;
  }
  cb.count = 0;
}

void cbSendLatestWire(CircularBuffer &cb, uint16_t maxToSend) {
  uint16_t toSend = (maxToSend < cb.count) ? maxToSend : cb.count; 
  for (uint16_t i = 0; i < toSend; i++) {
    //Serial.println(cb.data[cb.tail]);
    Wire.write(cb.data[cb.tail]);
    cb.tail = (cb.tail + 1) % cb.size;
  }
  cb.count -= toSend; // subtract exactly what was sent
}

void cbPutArray(CircularBuffer &cb, const uint8_t* bytes, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    cb.data[cb.head] = bytes[i];
    //Serial.println((int)bytes[1]);
    cb.head = (cb.head + 1) % cb.size;
    
    if (cb.count < cb.size) {
      cb.count++;
    } else {
      // buffer full → overwrite oldest
      cb.tail = (cb.tail + 1) % cb.size;
    }
  }
}


void cbPutByte(CircularBuffer &cb, byte inByte, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    //Serial.println(inByte);
    cb.data[cb.head] = inByte;
    cb.head = (cb.head + 1) % cb.size;
    
    if (cb.count < cb.size) {
      cb.count++;
    } else {
      // buffer full → overwrite oldest
      cb.tail = (cb.tail + 1) % cb.size;
    }
  }
}


int cbGetLatest(const CircularBuffer &cb) {
  if (cb.count == 0)
  return -1; // empty
  
  uint16_t pos = (cb.head == 0) ? (cb.size - 1) : (cb.head - 1);
  return cb.data[pos];
}

int freeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

//////////////////////////////////////////////
//UTILITY:

//////////////////////////////////////////////
//reading local EEPROM:
bool eepromReadCString(int addr, char *out, uint8_t maxLen, int &nextAddr, uint8_t blockOrdinal) {
  uint8_t i = 0;
  //Serial.println("^^^^^^^^^^^^^^^^^^^^");
  //Serial.println(addr);
  //Serial.println(maxLen);
  //Serial.println("=^^^^^^^^^^^^^^^^^^^^=");
  while (i < maxLen - 1) {
    byte b = EEPROM.read(addr++);
    delay(12); 
    yield();
    //Serial.println(addr);
    //Serial.print("Byte: ");
    //Serial.print((int) b);
    //Serial.print(" ");
    //Serial.println((char) b);
    if(blockOrdinal > 1) {
      //Serial.print((int)b);
      //Serial.print('#');
    }
    if (b > 128 || (b == 0 && i == 0)) { //if we hit an ASCII character out of range, we are in EEPROM that does not contain parser config strings
      parsedStringConfigCount = blockOrdinal;

      //Serial.print("number of blocks found: ");
      //Serial.println(parsedStringConfigCount);
    }
    if (b == 0) {
      //Serial.print("Fail out:");
      //Serial.println(i);
      out[i] = 0;
      nextAddr = addr;
      return true;
    }
    out[i++] = (char)b;
  }

  out[i] = 0;
  nextAddr = addr;
  return false; // truncated, but usable
}


//load default config in case we don't have anything in EEPROM
void initDefaultConfig() {
  cis[PARSE_MODE]       = 0;
  cis[BAUD_RATE_LEVEL]  = 9;
  cis[I2C_ADDRESS]      = 20;
  cis[REBOOT_PIN]       = 7;
  cis[SERIAL_MODE]      = 2;
}

void initSlaveConfigFromEeprom() {
  char magic[5];

  if(cis[BAUD_RATE_LEVEL] > 0) {
    setSerialRate(cis[BAUD_RATE_LEVEL]);
    
  }

  // Read magic header
  for (uint8_t i = 0; i < 4; i++) {
    magic[i] = EEPROM.read(SLAVE_CONFIG + i);
  }
  magic[4] = 0;
  //Serial.println(magic);
  //Serial.println("starting init: ");
  if (strcmp(magic, "DATA") != 0) {
    blockCount = 0;   // no parsing enabled
    //Serial.println("no data");
    return;
  } else {
    
    for (int i = 0; i < INT_CONFIGS; i++) {
        uint16_t address = SLAVE_CONFIG + 4 + i * 2;
        uint16_t value = (uint16_t)EEPROM.read(address + 1) | ((uint16_t)EEPROM.read(address) << 8);
        /*
        Serial.print(i);
        Serial.print(": ");
        Serial.println(value);
        */
        //comment this out if your EEPROM is screwed up:
        cis[i] = value;
    }

  }

  char cfg[MAX_BLOCKS][MAX_CFG_LEN];
   
  int addr = SLAVE_CONFIG + 4;
  addr += 21; //gotta bypass the integers at the bottom, assuming we know how many there are.  there should be five
  for(uint8_t i=0; i< MAX_BLOCKS; i++) {
    eepromReadCString(addr, cfg[i], sizeof(cfg[i]), addr, i);
    parseConfigString(cfg[i], blocks[i]);
    //dumpConfigBlock(blocks[i]);
    blockCount++;
  }
}

//////////////////////////////////////////////
//parsing serial:

//only for debugging:
void dumpConfigBlock(const ConfigBlock &b) {
  Serial.println(F("=== ConfigBlock ==="));

  Serial.print(F("start: "));
  Serial.println(b.start);

  Serial.print(F("end:   "));
  Serial.println(b.end);

  Serial.print(F("addrCount: "));
  Serial.println(b.addrCount);

  for (uint8_t i = 0; i < b.addrCount; i++) {
    Serial.print(F("  addr["));
    Serial.print(i);
    Serial.print(F("]: "));
    Serial.println(b.addr[i]);

    Serial.print(F("    offsets ("));
    Serial.print(b.offsetCount[i]);
    Serial.print(F("): "));

    for (uint8_t j = 0; j < b.offsetCount[i]; j++) {
      Serial.print(b.offsets[i][j]);
      if (j + 1 < b.offsetCount[i]) Serial.print(F(", "));
    }
    Serial.println();
  }

  Serial.println(F("==================="));
}

void parseConfigString(const char *cfg, ConfigBlock &out) {
  char buf[128];
  //Serial.println(cfg);
  //Serial.println(strlen(cfg));
  strncpy(buf, cfg, sizeof(buf));
  buf[sizeof(buf) - 1] = 0;

  char *tok = strtok(buf, "|");
  //Serial.println("xxxxxxx");
  if (!tok) {
    //Serial.println("yyyyyyy");
    return;
  }
  strncpy(out.start, tok, sizeof(out.start));
  //Serial.println("-----");
  //Serial.println( out.start);
  //Serial.println( out.end);
  tok = strtok(NULL, "|");
  if (!tok) return;
  strncpy(out.end, tok, sizeof(out.end));

  out.addrCount = 0;
  uint8_t curAddr = 255;

  while ((tok = strtok(NULL, "|"))) {
    //Serial.println(tok);
    if (!isInteger(tok)) { //any token in the config that is not explicitly a decimal integer is a string to be searched for
      curAddr = out.addrCount++;
      
      //Serial.println(tok);
      strncpy(out.addr[curAddr], tok, sizeof(out.addr[curAddr]));
      out.offsetCount[curAddr] = 0;
    } else if (curAddr != 255) {
      //Serial.print(out.offsetCount[curAddr] );
      //Serial.print(": ");
      //Serial.println(tok );
      if (out.offsetCount[curAddr] < MAX_OFFSETS) {
        out.offsets[curAddr][out.offsetCount[curAddr]++] = atoi(tok);
        //increment the global containing the parsed packet size. since each 16 bit value in the packet gets a pair of offsets, 
        //if we increment with every offset we get the correct number of bytes in the packet
        parsedStringPacketLen++; 
        //Serial.println(parsedStringPacketLen);
      }
    }
  }
  //dumpConfigBlock(out);
}


// Append 16-bit little-endian to global buffer
inline void appendU16(uint16_t v,
                      uint16_t bytePacketStart,
                      int8_t  blockIdx,
                      uint8_t addrIdx,
                      uint8_t offsetIdx,
                      uint8_t off1,
                      uint8_t off2,
                      uint8_t byteCount)
{
  //Serial.print(parsedStringPacketLen + 2);
  //Serial.print(": ");
  //Serial.print(PARSED_BUF_MAX);
  if (parsedStringPacketLen + 2 > PARSED_BUF_MAX) return;

  uint8_t lo = (uint8_t)(v & 0xFF);
  uint8_t hi = (uint8_t)(v >> 8);

  parsedBuf[bytePacketStart] = lo;
  parsedBuf[bytePacketStart+1] = hi;
  /*
  // ---- DEBUG TRACE ----
  Serial.print(F("[PARSE] blk="));
  Serial.print(blockIdx);

  Serial.print(F(" loc="));
  Serial.print(bytePacketStart);

  Serial.print(F(" addr="));
  Serial.print(addrIdx);

  Serial.print(F(" offIdx="));
  Serial.print(offsetIdx);

  Serial.print(F(" offs="));
  Serial.print(off1);
  if (off1 != off2) {
    Serial.print(',');
    Serial.print(off2);
  }

  Serial.print(F(" / "));
  Serial.print(byteCount);

  Serial.print(F("  bytes="));
  if (lo < 16) Serial.print('0');
  Serial.print(lo, HEX);
  Serial.print(' ');
  if (hi < 16) Serial.print('0');
  Serial.print(hi, HEX);

  Serial.print(F("  val="));
  Serial.println(v);
  */
}


uint16_t calculateOffsetIndex(const ConfigBlock *blocks,
                         uint8_t blockCount,
                         uint8_t blk,
                         uint8_t addr)
{
  uint16_t sum = 0;

  // 1. Sum all offsets in all previous blocks
  for (uint8_t b = 0; b < blk && b < blockCount; b++) {
    for (uint8_t a = 0; a < blocks[b].addrCount; a++) {
      sum += blocks[b].offsetCount[a];
    }
  }

  // 2. Sum offsets in previous addresses of this block
  if (blk < blockCount) {
    for (uint8_t a = 0; a < addr && a < blocks[blk].addrCount; a++) {
      sum += blocks[blk].offsetCount[a];
    }
  }

  return sum;
}


// Fill bytes[] with numeric values from a line, ignoring text tokens
uint8_t extractHexBytes(const char *line,
                             uint8_t *out,
                             uint8_t maxOut)
{
  uint8_t count = 0;

  while (*line && count < maxOut) {

    // must start with hex digit
    if (isxdigit(line[0]) &&
        isxdigit(line[1]) &&
        // word boundary before
        (line == 0 || !isxdigit(line[-1])) &&
        // word boundary after
        !isxdigit(line[2]))
    {
      char tmp[3] = { line[0], line[1], 0 };
      out[count++] = (uint8_t)strtoul(tmp, nullptr, 16);
      line += 2;
    } else {
      line++;
    }
  }
  return count;
}

void processSerialStream() {
  static char line[100];
  static uint8_t bytes[32];
 
  if (!readSerialLine(line, sizeof(line))) {
    return;
  }
  //Serial.println("------------------------------");
  //Serial.println(line);
  //Serial.println("++------------------------------");
  //Serial.println(sizeof(line));


  //Serial.println(line);
  /* ---- BLOCK START DETECTION ---- */
  for (uint8_t i = 0; i < blockCount; i++) {
    if (strlen(blocks[i].start) > 0 && strstr(line, blocks[i].start)) {
      //Serial.print(blocks[i].start);
      activeBlock = i;
      //parsedStringPacketLen = 0;              // reset output buffer
      return;                     // wait for data lines
    }
  }
  if (activeBlock < 0) {
    return;
  }

  /* ---- BLOCK END DETECTION ---- */
  if (strlen(blocks[activeBlock].end) > 0 && strstr(line, blocks[activeBlock].end)) {
    //Serial.println(blocks[activeBlock].end);
    activeBlock = -1;
    return;
  }
  

  /* ---- ADDRESS LINES ---- */
  ConfigBlock &blk = blocks[activeBlock];

  for (uint8_t a = 0; a < blk.addrCount; a++) {
    //Serial.println(blk.addr[a]);
    //Serial.println(line);
    if (!strstr(line, blk.addr[a])) {
      //Serial.println("fail");
      continue;
    }
    //Serial.println("success");
    uint8_t byteCount = extractHexBytes(line, bytes, sizeof(bytes));
    /*
    if (strcmp(blk.addr[a], "0x3ffbb60c") == 0) {
      Serial.println("--------------------------");
      Serial.println((int)byteCount);
      Serial.println(line);
    }
    */
    if (byteCount == 0) {

      return;
    }

    /* ---- OFFSET PROCESSING ---- */
    for (uint8_t o = 0; o + 1 < blk.offsetCount[a]; o += 2) {
      uint8_t off1 = blk.offsets[a][o];
      uint8_t off2 = blk.offsets[a][o + 1];
      if (off1 >= byteCount) {
        continue;
      }
    
      uint16_t v;
      
      if (off1 == off2) {
        // single byte, zero-extended
        v = bytes[off1];
      } else {
        if (off2 >= byteCount) {
          continue;
        }
        v = (uint16_t)bytes[off1] | ((uint16_t)bytes[off2] << 8);
      }


      uint16_t bytePacketStart = calculateOffsetIndex(blocks, parsedStringConfigCount, activeBlock, a);
     

      
      //Serial.println(line);
      appendU16(
        v,
        bytePacketStart + o,
        (int8_t)activeBlock,
        a,           // address index
        o,           // offset rule index
        off1,
        off2,
        byteCount
      );
    }

    //break;   // only one address per line
  }
}



bool readSerialLine(char *line, uint8_t maxLen) {
  static uint8_t idx = 0;
  while (Serial.available()) {
    char c = Serial.read();
    //Serial.print(c);
    if (c == '\n') {
      line[idx] = 0;
      idx = 0;
      return true;
    }
    if (idx < maxLen - 1) {
      line[idx++] = c;
    }
  }
  return false;
}

 
