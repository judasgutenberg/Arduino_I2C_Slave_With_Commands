#include <Wire.h>
#include <TimeLib.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <EEPROM.h> // needed for EEPROM read/write



#define VERSION 2019


#define RX_SIZE 100
#define TX_SIZE 60


//indexes into configuration array
#define BAUD_RATE_LEVEL 6
#define I2C_ADDRESS 7
#define REBOOT_PIN 8

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
#define CMD_SET_PARSED_OFFSET               175

#define COMMAND_SET_UNIX_TIME 180
#define COMMAND_GET_UNIX_TIME 181
 

#define EEPROM_SIZE 1024
#define SLAVE_CONFIG 512  //where local slave configs live

#define MAX_BLOCKS 2
#define MAX_ADDRS  4
#define MAX_OFFSETS 8

#define MAX_CFG_LEN 70
#define PARSED_BUF_MAX 30   // bytes (15 values)

uint8_t parsedBuf[PARSED_BUF_MAX]; //a packed register for returning parsed serial values
uint8_t parsedLen = 0;

struct ConfigBlock {
  char start[32];
  char end[32];
  uint8_t addrCount;
  char addr[MAX_ADDRS][12];
  uint8_t offsets[MAX_ADDRS][MAX_OFFSETS];
  uint8_t offsetCount[MAX_ADDRS];
};

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
uint16_t cis[10];



 

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

    processSerialStream();


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

    if(cis[BAUD_RATE_LEVEL] > 0) {
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
    uint8_t remaining = parsedLen - parsedReadOffset;
    uint8_t chunk = remaining > 30 ? 30 : remaining;
  
    for (uint8_t i = 0; i < chunk; i++) {
      Wire.write(parsedBuf[parsedReadOffset + i]);
    }
    parsedReadOffset += chunk;
  
    if (parsedReadOffset >= parsedLen) {
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

    for (int i = 0; i < bytesRead; i++) {
        value |= ((long)buffer[i] << (8 * i));
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
    } else if (command == COMMAND_GET_LAST_PARSE_TIME) {
      dataToSend = lastDataParseTime;
    } else if (command == COMMAND_GET_SLAVE_CONFIG) {
      dataToSend = SLAVE_CONFIG;
    } else if (command == CMD_SET_PARSED_OFFSET) {
      parsedReadOffset = value;
    } else if (command == COMMAND_GET_PARSED_SERIAL_DATA) {
      retrieveParsedSerialData = true;
 
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
inline void appendU16(uint16_t v) {
  if (parsedLen + 2 > PARSED_BUF_MAX) return;  // hard stop
  parsedBuf[parsedLen++] = (uint8_t)(v & 0xFF);
  parsedBuf[parsedLen++] = (uint8_t)(v >> 8);
}

//////////////////////////////////////////////
//reading local EEPROM:
bool eepromReadCString(int addr, char *out, uint8_t maxLen, int &nextAddr) {
  uint8_t i = 0;

  while (i < maxLen - 1) {
    byte b = EEPROM.read(addr++);
    if (b == 0) {
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
  cis[BAUD_RATE_LEVEL] = 9;
  cis[I2C_ADDRESS] = 20;
  cis[REBOOT_PIN] = 7;
}


//////////////////////////////////////////////
//parsing serial:

void initSlaveConfigFromEeprom() {
  char magic[5];

  // Read magic header
  for (uint8_t i = 0; i < 4; i++) {
    magic[i] = EEPROM.read(SLAVE_CONFIG + i);
  }
  magic[4] = 0;

  if (strcmp(magic, "DATA") != 0) {
    blockCount = 0;   // no parsing enabled
    return;
  }
  if(cis[BAUD_RATE_LEVEL] > 0) {
    setSerialRate(cis[BAUD_RATE_LEVEL]);
    
  }
  char cfg0[MAX_CFG_LEN];
  char cfg1[MAX_CFG_LEN];
  int addr = SLAVE_CONFIG + 4;
  if (!eepromReadCString(addr, cfg0, sizeof(cfg0), addr)) {
    blockCount = 0;
    return;
  }
  if (!eepromReadCString(addr, cfg1, sizeof(cfg1), addr)) {
    blockCount = 0;
    return;
  }

  // Parse configs
  parseConfigString(cfg0, blocks[0]);
  parseConfigString(cfg1, blocks[1]);

  blockCount = 2;
}

void parseConfigString(const char *cfg, ConfigBlock &out) {
  char buf[128];
  strncpy(buf, cfg, sizeof(buf));
  buf[sizeof(buf) - 1] = 0;

  char *tok = strtok(buf, "|");
  if (!tok) return;
  strncpy(out.start, tok, sizeof(out.start));

  tok = strtok(NULL, "|");
  if (!tok) return;
  strncpy(out.end, tok, sizeof(out.end));

  out.addrCount = 0;
  uint8_t curAddr = 255;

  while ((tok = strtok(NULL, "|"))) {
    if (strncmp(tok, "0x", 2) == 0 || tok[0] == 'I') {
      curAddr = out.addrCount++;
      strncpy(out.addr[curAddr], tok, sizeof(out.addr[curAddr]));
      out.offsetCount[curAddr] = 0;
    } else if (curAddr != 255) {
      out.offsets[curAddr][out.offsetCount[curAddr]++] = atoi(tok);
    }
  }
}


bool readSerialLine(char *line, uint8_t maxLen) {
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();
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

uint8_t extractHexBytes(const char *line, uint8_t *bytes, uint8_t maxBytes) {
  uint8_t count = 0;
  while (*line && count < maxBytes) {
    if (isxdigit(line[0]) && isxdigit(line[1]) && line[2] == ' ') {
      char tmp[3] = { line[0], line[1], 0 };
      bytes[count++] = strtoul(tmp, NULL, 16);
      line += 3;
    } else {
      line++;
    }
  }
  return count;
}

void processSerialStream() {
  static char line[128];
  static uint8_t bytes[32];

  if (!readSerialLine(line, sizeof(line))) return;

  // Detect block start
  for (uint8_t i = 0; i < blockCount; i++) {
    if (strstr(line, blocks[i].start)) {
      activeBlock = i;
      parsedLen = 0;        // reset output buffer
      return;
    }
  }

  if (activeBlock < 0) return;

  // Detect block end
  if (strstr(line, blocks[activeBlock].end)) {
    activeBlock = -1;
    return;
  }

  // Process addresses in config order
  for (uint8_t a = 0; a < blocks[activeBlock].addrCount; a++) {
    if (!strstr(line, blocks[activeBlock].addr[a])) continue;

    uint8_t byteCount = extractHexBytes(line, bytes, sizeof(bytes));

    for (uint8_t o = 0; o < blocks[activeBlock].offsetCount[a]; o++) {
      uint8_t off = blocks[activeBlock].offsets[a][o];
      if (off < byteCount) {
        uint16_t v = bytes[off];   // zero-extend u8 → u16
        //Serial.println(v);
        appendU16(v);
      }
    }
  }
  lastDataParseTime = unixTime;  //save this info so we know how stale our data is
}
