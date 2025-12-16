#include <Wire.h>
#include <TimeLib.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <EEPROM.h> // needed for EEPROM read/write

// ---- CONFIG ----
#define I2C_SLAVE_ADDR 20
#define REBOOT_PIN 7              // pin used to reset master

#define VERSION 2016

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
#define COMMAND_SERIAL_SET_BAUD_RATE 170
#define COMMAND_RETRIEVE_SERIAL_BUFFER 171
#define COMMAND_POPULATE_SERIAL_BUFFER 172


#define COMMAND_SET_UNIX_TIME 180
#define COMMAND_GET_UNIX_TIME 181

#define EEPROM_SIZE 1024
#define SLAVE_CONFIG 512  //where local slave configs live

struct CircularBuffer {
  uint8_t* data;
  uint16_t size;
  volatile uint16_t head; // modified in ISR / main
  volatile uint16_t tail;
  volatile uint16_t count;
};

#define RX_SIZE 400
#define TX_SIZE 60

uint8_t rxStorage[RX_SIZE];
uint8_t txStorage[TX_SIZE];

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
volatile unsigned long timeLastPrinted = 0;
volatile byte deferredCommand = 0;
volatile byte eepromReadCount = 1; 

volatile uint8_t eepromWriteBuffer[32];
volatile uint8_t eepromWriteLength = 0;
volatile bool eepromWritePending = false;
volatile bool retrieveSerialData = false;
uint32_t baudRate = 0;

volatile uint32_t unixTime = 0;


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
    //Serial.begin(115200);
    //while (Serial.available()) Serial.read(); 
    //Serial.println("Started");

    memset(rxStorage, 0, RX_SIZE);
    rxBuffer.head = rxBuffer.tail = rxBuffer.count = 0;
    
    memset(txStorage, 0, TX_SIZE);
    txBuffer.head = txBuffer.tail = txBuffer.count = 0;

    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    
    pinMode(REBOOT_PIN, OUTPUT);
    digitalWrite(REBOOT_PIN, HIGH); // idle high
    setupTimer1();
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

    if(baudRate > 0) {
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
  unixTime++;
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
    digitalWrite(REBOOT_PIN, LOW);
    delay(100);
    digitalWrite(REBOOT_PIN, HIGH);
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
    } else if (command == COMMAND_GET_SLAVE_CONFIG) {
      dataToSend = SLAVE_CONFIG;
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
                    Serial.println(value);
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

void setSerialRate(byte baudRateLevel) {
  /*
  while(Serial.available()) {
    Serial.read();
  }
  delay(2);
  Serial.flush();
  Serial.end();
  */
  //Serial.begin(115200);

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
  baudRate = newBaud;

  if (newBaud > 0) {
    Serial.end();
    delay(2);
    Serial.begin(newBaud);
    delay(2);

    while (Serial.available()) {
      Serial.read();   // THIS is the missing piece
    }
  }
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
