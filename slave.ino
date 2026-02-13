//this is code for a comprehensive Arduino-based Atmega I2C slave that can do various additional functions to support a master
//an I2C slave is a port expander, a place to persist configuration data, and a flexible serial port monitor & parser
//Gus Mueller, 2024 - 2026
//based on this earlier, simpler version:  https://github.com/judasgutenberg/Generic_Arduino_I2C_Slave

#include <avr/sleep.h>
#include <avr/power.h>
#include <Wire.h>
#include <TimeLib.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <EEPROM.h> // needed for EEPROM read/write

#define VERSION 2088 //enabled COMMAND_REBOOT, set unix time for last data parse, allow jump to bootloader for Atmega328p and Atmega644p

#define MEMSIZE ((RAMEND - SRAM_START) + 1) / 1024

#define INT_CONFIGS 10
#define LONG_CONFIGS 4

#define BOOT_MAGIC_ADDR 510 
#define BOOT_MAGIC_VALUE 0xB007
//4k for atmega328p: 0x7000
//4k for atmega644p: 0xF000
//but we no longer even need this:
//#define BOOTLOADER_START 0xF000 

#if MEMSIZE * 10 <= 1
  #define RX_SIZE 10
  #define TX_SIZE 10
#elif MEMSIZE <= 2
  #define RX_SIZE 100
  #define TX_SIZE 100
#elif MEMSIZE <= 4
  #define RX_SIZE 1000
  #define TX_SIZE 500 
#elif MEMSIZE <= 8
  #define RX_SIZE 2000
  #define TX_SIZE 2000 
#else
  #define RX_SIZE 2500
  #define TX_SIZE 2500 
#endif


//Indexes into integer configuration array
#define POWER_MODE      4
#define PARSING_STYLE   5
#define BAUD_RATE_LEVEL 6
#define I2C_ADDRESS     7
#define REBOOT_PIN      8
#define SERIAL_MODE     9 

// Existing commands
#define COMMAND_REBOOT              128   //reboots the slave asynchronously using the watchdog system
#define COMMAND_MILLIS              129   //returns the millis() value of the slave 
#define COMMAND_LASTWATCHDOGREBOOT  130   //millis() of the last time the slave sent a reboot signal to the master
#define COMMAND_WATCHDOGREBOOTCOUNT 131   //number of times the slave has rebooted the master since it was itself rebooted
#define COMMAND_LASTWATCHDOGPET     132   //millis() of the last time the master petted the slave in its watchdog function
#define COMMAND_LASTPETATBITE       133   //how many seconds late the last watchdog pet was when the slave sent a reboot signal
#define COMMAND_REBOOTMASTER        134   //reboot the master now by asserting the reboot line
#define COMMAND_SLEEP               135   //go into the kind of sleep where I2C will wake it up
#define COMMAND_DEEP_SLEEP          136   //go into unreachably deep sleep for n seconds

#define COMMAND_WATCHDOGPETBASE     200   //commands above 200 are used to tell the slave how often it needs to be petted.  this command can also update the slave's unix timestamp

// New EEPROM-style commands
#define COMMAND_EEPROM_SETADDR      150   // set pointer for read/write
#define COMMAND_EEPROM_WRITE        151   // sequential write mode
#define COMMAND_EEPROM_READ         152   // sequential read mode
#define COMMAND_EEPROM_NORMAL       153   // exit EEPROM mode, back to default behavior

#define COMMAND_VERSION             160   //returns the human-updated version number of the firmware source code.  this version began at 2000
#define COMMAND_COMPILEDATETIME     161   //unix timestamp of when the firmware was compiled
#define COMMAND_TEMPERATURE         162   //a pseudo-random poor approximation of temperature
#define COMMAND_FREEMEMORY          163   //returns free memory on the slave
#define COMMAND_GET_SLAVE_CONFIG    164   //returns where in the EEPROM the slave's local configuration is persisted
#define COMMAND_GET_PROCESSOR_TYPE  165   //returns an int representing the processor type
#define COMMAND_GET_MEMORY_SIZE     166   //returns an int representing the processor type

//serial commands
#define COMMAND_PARSE_BUFFER                169   //explicitly parse data in the txBuffer using the serial parser system
#define COMMAND_SERIAL_SET_BAUD_RATE        170   //using an ordinal to set common serial baud rates.  1 is 300, 5 is 9600, 9 is 115200
#define COMMAND_RETRIEVE_SERIAL_BUFFER      171   //retrieves values from the serial read buffer if we are in serial mode #1
#define COMMAND_POPULATE_SERIAL_BUFFER      172   //sets values in the serial buffer that the slave will transmit via serial
#define COMMAND_GET_LAST_PARSE_TIME         173   //retrieves the unix time of the last serial parse, if unix time is known
#define COMMAND_GET_PARSED_SERIAL_DATA      174   //returns a whole packet of parsed data
#define COMMAND_SET_PARSED_OFFSET           175   //if parsed data packet is large, this will set a pointer into it for retrieval from the master
#define COMMAND_GET_PARSED_DATUM            176   //returns a specific value found by the serial parser given an ordinal into a 16 bit sequence in the packet
#define COMMAND_GET_PARSE_CONFIG_NUMBER     177   //returns the number of parser configs (blocks used by the serial parser, equivalent to items in css array)
#define COMMAND_GET_PARSED_PACKET_SIZE      178   //returns the size of the parsed data packet
#define COMMAND_SET_SERIAL_MODE             179   //sets serial mode:  
                                                  //0 - no serial
                                                  //1 - serial pass-through to master
                                                  //2 - slave parses incoming values in serial but cannot transmit serial values
                                                  //3 - gather interesting serial lines from parser for master to pick up
                                                  //4 - parses incoming values in serial, though can still transmit data serial port
                                                  //5 - fakes the reception of data via serial using I2C data sent from master using send slave serial (command COMMAND_POPULATE_SERIAL_BUFFER)
#define COMMAND_SET_UNIX_TIME               180   //sets unix timestamp, which the slave the automatically advances with reasonable accuracy
#define COMMAND_GET_UNIX_TIME               181   //returns unix timestamp as known to the slave
#define COMMAND_GET_CONFIG                  182   //gets a config item by ordinal number (from the configuration cis[] array)
#define COMMAND_SET_CONFIG                  183   //sets a config item by ordinal and value
#define COMMAND_GET_LONG                    184   //gets a config long item in cls by ordinal number (from the configuration cis[] array)
#define COMMAND_SET_LONG                    185   //sets a config long item in cls by ordinal and value

#define COMMAND_ENTER_BOOTLOADER            190   //reflash bootloader

#define EEPROM_SIZE 1024
#define SLAVE_CONFIG 512  //where local slave configs live

//These configure the serial parser using data from the css array on the master (css is never actually produced on the slave - but it is stored in the EEPROM on the slave)
//Making MAX_BLOCKS bigger than 3 will easily max-out the 2K of memory on an Atmega328. If you have complex parsing needs, use an Atmega2560 (8k of RAM!)

#if MEMSIZE <= 1
  #define MAX_BLOCKS 1
  #define MAX_ADDRS  1
  #define MAX_OFFSETS 1
  #define MAX_CFG_LEN 120
#elif MEMSIZE <= 2
  #define MAX_BLOCKS 3
  #define MAX_ADDRS  3
  #define MAX_OFFSETS 8
  #define MAX_CFG_LEN 120
#elif MEMSIZE <= 4
  #define MAX_BLOCKS 4
  #define MAX_ADDRS  3
  #define MAX_OFFSETS 8
  #define MAX_CFG_LEN 120
#elif MEMSIZE <= 8
  #define MAX_BLOCKS 5
  #define MAX_ADDRS  4
  #define MAX_OFFSETS 10
  #define MAX_CFG_LEN 150
#else
  #define MAX_BLOCKS 6
  #define MAX_ADDRS  6
  #define MAX_OFFSETS 10
  #define MAX_CFG_LEN 220
#endif



#define PARSED_BUF_MAX 30   // bytes (15 values)

#define PS_BIG_ENDIAN   0x04
#define PS_CHAR_OFFSET  0x02
#define PS_ASCII_VALUE  0x01

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


//The serial parser (cis[SERIAL_MODE] == 2, 3 or 4) is configured with cstrings stored in EEPROM starting after the "DATA" intro at SLAVE_CONFIG bytes into the EEPROM
//a typical parser config looks like: Characteristic #2;0x3ffbb61c;0x3ffbb5fc;4;5;6;7;8;9;10;11;0x3ffbb60c;0;1
//where the parser examines data between the strings "Characteristic #2" and "0x3ffbb61c" found in the serial stream.
//it then looks inside that for the string "0x3ffbb5fc" and returns low-endian 16-bit values between the offsets 4 & 5, 6 & 7, 8 & 9...
//it then looks further for the string "0x3ffbb60c" and returns low-endian 16-bit values between the offsets 0 & 1
//The parsed values end up in the data packet parsedBuf
//for parsing purposes, the configuration strings are put in structs of type ConfigBlock
//Sample ConfigBlock population from a config like so: Characteristic #2;0x3ffbb61c;0x3ffbb5fc;4;5;6;7;8;9;10;11;0x3ffbb60c;0;1
/*

start = "Characteristic #2"
end   = "0x3ffbb61c"
addr[0] = "0x3ffbb5fc"
offsets[0] = {4,5,6,7,8,9,10,11}
addr[1] = "0x3ffbb60c"
offsets[1] = {0,1}
offsetCount[0] = 8
offsetCount[1] = 2
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
uint32_t cls[LONG_CONFIGS];   //RAM values for use by master, mostly to save state through a reboot
uint16_t cis[INT_CONFIGS];    //the configuration array, which contains unsigned 16 bit values. This gets defaults from initDefaultConfig()
//and if there are EEPROM values, those are overridden

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

volatile uint8_t multiRequestOffset = 0;
volatile uint32_t deferredParameter = 0;
volatile uint8_t powerState = 0;

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
    setupTimer1(); //used to accurately increment unixTime once set
    lastWatchdogPet = millis(); // start watchdog timer
}

void loop() {
    unsigned long now = millis();
    powerState++;
    //Serial.println(now);
    if(cis[BAUD_RATE_LEVEL] > 0 && (cis[SERIAL_MODE] == 2  || cis[SERIAL_MODE] == 3 || cis[SERIAL_MODE] == 4)) {
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
    if(cis[BAUD_RATE_LEVEL] > 0 && (cis[SERIAL_MODE] == 1 || cis[SERIAL_MODE] == 3 || cis[SERIAL_MODE] == 4)) {
      cbSendLatest(txBuffer);  
    }
    if(cis[BAUD_RATE_LEVEL] > 0 && cis[SERIAL_MODE] == 1 ) {
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
        deferredCommand = 0;
        softwareReset();
    }
    if(deferredCommand == COMMAND_SLEEP) { // async sleep
        deferredCommand = 0;
        goToSleepIdle();
    }
    if(deferredCommand == COMMAND_DEEP_SLEEP) { // async sleep
        deferredCommand = 0;
        deepSleepForSeconds(deferredParameter);
        deferredParameter = 0;
    }
    //if we've done an I2C interrupt handler and then been through a loop once, we can sleep if in powermode 2.  the larger the powermode, the less wasteful
    if(powerState > 1 && cis[POWER_MODE] == 5 || powerState > 10  && cis[POWER_MODE] == 4 || powerState > 20  && cis[POWER_MODE] == 3 || powerState > 40  && (cis[POWER_MODE] == 2 || cis[POWER_MODE] == 1)){
      goToSleepIdle();
    }
    //these power_modes should only be used if you aren't controlling any devices with your slave. you might also want to turn off the watchdog functionality
    //although this system should come up in time to receive the next pet
   if(powerState > 1 && cis[POWER_MODE] == 9  || powerState > 10 && cis[POWER_MODE] == 8 || powerState > 20 && cis[POWER_MODE] == 7 || powerState > 40 && cis[POWER_MODE] == 6) {
      deepSleepForSeconds(watchdogTimeout);
   }
}

ISR(TIMER1_COMPA_vect) {
  //most accurate way to do this:
  if(unixTime > 0) { //don't increment unixTime if it was not set
    unixTime++;
  }
}

void setupTimer1(void)
{
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 15624;                 // 1 second @ 8 MHz, prescaler 1024
    TCCR1B |= (1 << WGM12);        // CTC mode
    TCCR1B |= (1 << CS12) | (1 << CS10); // prescaler 1024
#if defined(TIMSK1)
    TIMSK1 |= (1 << OCIE1A);       // ATmega328/168/etc
#else
    TIMSK |= (1 << OCIE1A);        // ATmega32 / ATmega32A
#endif
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
  sleep_disable();
  powerState = 1;
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
    cbSendLatestWire(rxBuffer, 32);
    retrieveSerialData = false;
  } else if (retrieveParsedSerialData) {
    //Serial.println("parsedStringPacketLen: ");
    //Serial.println(parsedStringPacketLen);
    //parsedStringPacketLen = 60;
    if(parsedStringPacketLen > 0){      
      uint8_t remaining = parsedStringPacketLen - multiRequestOffset;
      uint8_t chunk = remaining > 30 ? 30 : remaining;
      
      for (uint8_t i = 0; i < chunk; i++) {
        Wire.write(parsedBuf[multiRequestOffset + i]);
        /*
        Serial.print("in that loop: ");
        Serial.print(parsedStringPacketLen);
        Serial.print("; ");
        Serial.print(multiRequestOffset);
        Serial.print("; ");
        
        Serial.print(multiRequestOffset + i);
        Serial.print(": ");
        Serial.println(parsedBuf[multiRequestOffset + i]);
        */
      }
   
      multiRequestOffset += chunk;
    
      if (multiRequestOffset >= parsedStringPacketLen) {
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
    sleep_disable();
    powerState = 1;
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
        //if we are sending serial data from the master to the slave via I2C and we are in serial mode 5
        //then what actually happens is the data never gets sent as serial from the slave
        //instead it is processed by the serial parser as if it arrived via serial 
        //this can be used for debugging or processing data collected from serial by other methods
        //Serial.print("val: ");
        //Serial.println(val);
    }
    uint8_t valueByteStart = 0;
    //if we issue a COMMAND_SET_CONFIG then the first byte after that command is an ordinal into the config array
    //and all the rest is the value that is to be set. this can be used for other purposes, but for now we just use it for this
    if(command == COMMAND_SET_CONFIG || command == COMMAND_SET_LONG) { 
      ordinalLocation = buffer[0];
      valueByteStart = 1;
    }  
    
    for (uint8_t i = valueByteStart; i < bytesRead; i++) {
        value |= ((long)buffer[i] << (8 * (i-valueByteStart)));
    }

    if (command == COMMAND_POPULATE_SERIAL_BUFFER) {
      //Serial.println("populating serial buffer");
      cbPutArray(txBuffer, buffer, bytesRead);


    } else if (command == COMMAND_ENTER_BOOTLOADER) {
        /*
        //various attempts:
        EEPROM.put(SLAVE_CONFIG-2, BOOT_MAGIC_VALUE); //let's try this via EEPROM
        
        // Ensure write completes before reset
        asm volatile ("cli");
        wdt_enable(WDTO_15MS);
        */
        _delay_ms(10);       // ensure EEPROM write completes


   
            
        EEPROM.put(BOOT_MAGIC_ADDR, BOOT_MAGIC_VALUE);
        //EEPROM.commit(); // optional on some platforms
    
        // Optionally wait a moment to ensure write completes
        delay(10);
        //let's try something else
        // Soft jump to bootloader


        // Enable watchdog reset immediately
        wdt_enable(WDTO_15MS); // shortest timeout
        while(1);              // wait for reset
      
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
    } else if (command == COMMAND_PARSE_BUFFER) {
      processSerialStream();
    } else if (command == COMMAND_GET_CONFIG) {
      //Serial.println(cis[value]);
      dataToSend = cis[value]; //so far we can only retrieve individual integer config items
    } else if (command == COMMAND_GET_LONG) {
      dataToSend = cls[value];  
      //Serial.println(dataToSend);
    } else if (command == COMMAND_SET_LONG) {
      cls[ordinalLocation] = value; //so far we can only set individual integer config items
    } else if (command == COMMAND_SET_CONFIG) {
      //Serial.println(ordinalLocation);
      //Serial.println(value);
      cis[ordinalLocation] = value; //so far we can only set individual integer config items
    } else if (command == COMMAND_GET_LAST_PARSE_TIME) {
      dataToSend = lastDataParseTime;
    } else if (command == COMMAND_GET_SLAVE_CONFIG) {
      dataToSend = SLAVE_CONFIG;
    } else if (command == COMMAND_SET_PARSED_OFFSET) {
      multiRequestOffset = value;
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
            deferredParameter = value;
            break;
        case COMMAND_SLEEP:
            deferredCommand = command;
            deferredParameter = value;
            break;
        case COMMAND_DEEP_SLEEP:
            deferredCommand = command;
            deferredParameter = value;
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

        case COMMAND_GET_PROCESSOR_TYPE:
            dataToSend = processorType();
            break;

        case COMMAND_GET_MEMORY_SIZE:
            dataToSend = RAMEND+1;
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
                    if(value > 0) {
                      unixTime = value;
                    }
                }
                lastWatchdogPet = millis();
                dataToSend = lastWatchdogPet;
            }
            break;
    }
}

// ---- Software Reset ----
void softwareReset(void) {
    //return; // disabled for safety
    cli();
    TWCR = 0;
    DDRC &= ~((1<<PC4) | (1<<PC5));
    PORTC |= (1<<PC4) | (1<<PC5);
    wdt_enable(WDTO_15MS);
    while(1) { }
}

// ---- Utility ----

bool cstrContainsChar(const char* str, char needle)
{
  if (!str) return false;

  while (*str) {
    if (*str == needle) {
      return true;
    }
    ++str;
  }
  return false;
}

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

void circularBufferToCStr(
  CircularBuffer* cb,
  char* out,
  size_t outSize,
  bool advanceTail   // true = consume bytes
)
{
  uint16_t tail, count;

  // Snapshot state atomically
  noInterrupts();
  tail  = cb->tail;
  count = cb->count;
  interrupts();

  uint16_t n = count;
  if (n >= outSize) {
    n = outSize - 1;
  }

  uint16_t idx = tail;

  for (uint16_t i = 0; i < n; i++) {
    out[i] = (char)cb->data[idx];
    idx++;
    if (idx >= cb->size) {
      idx = 0;
    }
  }

  out[n] = '\0';

  if (advanceTail && n > 0) {
    noInterrupts();
    cb->tail  = (cb->tail + n) % cb->size;
    cb->count -= n;
    interrupts();
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

void deepSleepForSeconds(uint32_t seconds) {
  while (seconds > 0) {
    if (seconds >= 8) {
      #if defined(__AVR_ATmega32A__) || defined(__AVR_ATmega32__)
        // ATmega32A max interval ~2s
        setupWatchdogInterval((1 << WDP2) | (1 << WDP1)); // ~2s
      #else
        // modern AVR (328P, 168, 644P)
        setupWatchdogInterval((1 << WDP3) | (1 << WDP0)); // ~8s
      #endif
      sleepOnce();
      seconds -= 8;
    } else if (seconds >= 4) {
      #if defined(__AVR_ATmega32A__) || defined(__AVR_ATmega32__)
        #define WDT_INTERVAL ((1<<WDP2) | (1<<WDP1)) // ~2s max on 32A
      #else
        #define WDT_INTERVAL (1<<WDP3)                // ~4s on modern AVRs
      #endif
      setupWatchdogInterval(WDT_INTERVAL);
      sleepOnce();
      seconds -= 4;
    } else if (seconds >= 2) {
      setupWatchdogInterval((1 << WDP2) | (1 << WDP1)); // ~2s
      sleepOnce();
      seconds -= 2;
    } else if (seconds >= 1) {
      setupWatchdogInterval((1 << WDP2)); // ~1s
      sleepOnce();
      seconds -= 1;
    } else {
      // fractional remainder: use 500 ms
      setupWatchdogInterval((1 << WDP1) | (1 << WDP0)); // ~500ms
      sleepOnce();
      seconds = 0;
    }
  }
}

void goToSleepIdle() {
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();

  // Optional but recommended: disable brown-out during sleep
  //sleep_bod_disable(); //this is available on an Atmega328

  sleep_cpu();   // CPU sleeps here

  // Execution resumes here after I2C activity
  sleep_disable();
}

void sleepOnce() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  //sleep_bod_disable(); //this is available on an Atmega328
  sleep_cpu();
  sleep_disable();
}

void setupWatchdogInterval(uint8_t wdpBits) {
  cli();
  MCUSR &= ~(1 << WDRF);
  #if defined(__AVR_ATmega32A__) || defined(__AVR_ATmega32__)
    WDTCR |= (1 << WDTOE) | (1 << WDE);  // ATmega32/32A
  #else
    WDTCSR |= (1 << WDCE) | (1 << WDE);  // modern AVRs
  #endif
  #if defined(__AVR_ATmega32A__) || defined(__AVR_ATmega32__)
    WDTCR = (1 << WDE) | (1 << WDP2) | (1 << WDP1);  // ~2s max
  #else
    WDTCSR = (1 << WDIE) | wdpBits; // modern AVRs, e.g., ~4s or 8s
  #endif
  sei();
}


void goToDeepSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  //sleep_bod_disable(); //this is available on an Atmega328
  sleep_cpu();          // sleep here
  sleep_disable();
}

ISR(WDT_vect) {
  // nothing required
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
  cis[POWER_MODE]       = 2; //default to mild power savings
  cis[PARSING_STYLE]    = 0;
  cis[BAUD_RATE_LEVEL]  = 9;
  cis[I2C_ADDRESS]      = 20;
  cis[REBOOT_PIN]       = 7;
  cis[SERIAL_MODE]      = 2;
                        //0 - no serial
                        //1 - serial pass-through to master
                        //2 - slave parses incoming values in serial but cannot transmit serial values
                        //3 - gather interesting serial lines from parser for master to pick up
                        //4 - parses incoming values in serial, though can still transmit data serial port
                        //5 - fakes the reception of data via serial using I2C data sent from master using send slave serial (command COMMAND_POPULATE_SERIAL_BUFFER)
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

  char *tok = strtok(buf, ";");
  if (!tok) {
    return;
  }
  strncpy(out.start, tok, sizeof(out.start));
  //Serial.println("-----");
  //Serial.println( out.start);
  //Serial.println( out.end);
  tok = strtok(NULL, ";");
  if (!tok) return;
  strncpy(out.end, tok, sizeof(out.end));

  out.addrCount = 0;
  uint8_t curAddr = 255;

  while ((tok = strtok(NULL, ";"))) {
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

void processSerialStream()
{
  static char line[100];
  if(cis[SERIAL_MODE] < 5) {
    if (!readSerialLine(line, sizeof(line))) {
      return;
    }
  } else { //get "serial data" from the master via I2C using the txBuffer
    circularBufferToCStr(&txBuffer, line, 100, true);
    //Serial.println(line);
  }
  /* ---- BLOCK START DETECTION ---- */
  for (uint8_t i = 0; i < blockCount; i++) {
    if (strlen(blocks[i].start) > 0 &&
        strstr(line, blocks[i].start)) {

      activeBlock = i;
      return;   // wait for data lines
    }
  }

  if (activeBlock < 0) {
    return;
  }

  /* ---- BLOCK END DETECTION ---- */
  if (strlen(blocks[activeBlock].end) > 0 &&
      strstr(line, blocks[activeBlock].end)) {

    activeBlock = -1;
    return;
  }

  /* ---- ADDRESS LINES ---- */
  ConfigBlock &blk = blocks[activeBlock];

  for (uint8_t a = 0; a < blk.addrCount; a++) {

    // fast reject if address not present
    if (!strstr(line, blk.addr[a])) {
      continue;
    }
    if(cis[SERIAL_MODE] == 3) { //used for just shipping the data-bearing lines back to the master to be forwarded to the backend for logging and analysis
      cbPutArray(rxBuffer, line, strlen(line));
      return;
    }
    /* ---- OFFSET PROCESSING ---- */
    for (uint8_t o = 0; o + 1 < blk.offsetCount[a]; o += 2) {

      uint8_t off1 = blk.offsets[a][o];
      uint8_t off2 = blk.offsets[a][o + 1];

      uint16_t v;

      if (!readValueAtOffset(
              line,
              blk.addr[a],
              off1,
              off2,
              cis[PARSING_STYLE],
              v))
      {
        continue;   // parse failed for this offset pair
      }
      lastDataParseTime = unixTime;

      uint16_t bytePacketStart =
        calculateOffsetIndex(
          blocks,
          parsedStringConfigCount,
          activeBlock,
          a
        );

      appendU16(
        v,
        bytePacketStart + o,
        (int8_t)activeBlock,
        a,      // address index
        o,      // offset rule index
        off1,
        off2,
        0       // byteCount no longer relevant here
      );
    }
  }
}

bool hexByteAt(const char *p, uint8_t &out)
{
  if (!isxdigit(p[0]) || !isxdigit(p[1])) {
    return false;
  }

  char tmp[3] = { p[0], p[1], 0 };
  out = (uint8_t)strtoul(tmp, nullptr, 16);
  return true;
}

const char *findNthToken(const char *s, uint8_t n)
{
  uint8_t count = 0;

  while (*s) {
    while (*s == ' ') s++;
    if (!*s) break;

    if (count++ == n) {
      return s;
    }

    while (*s && *s != ' ') s++;
  }

  return nullptr;
}


bool readValueAtOffset(
    const char *line,
    const char *addrStr,
    uint8_t off1,
    uint8_t off2,
    uint8_t parsingStyle,
    uint16_t &outValue)
{
  const char *addrPos = strstr(line, addrStr);
  if (!addrPos) return false;

  addrPos += strlen(addrStr);

  const char *p1 = nullptr;
  const char *p2 = nullptr;

  /* ---- OFFSET RESOLUTION ---- */

  if (parsingStyle & PS_CHAR_OFFSET) {
    // character-based offset
    p1 = addrPos + off1;
    p2 = addrPos + off2;
  } else {
    // token-based offset
    p1 = findNthToken(addrPos, off1);
    p2 = findNthToken(addrPos, off2);
  }

  if (!p1) return false;
  if (off2 != off1 && !p2) return false;

  uint8_t b0 = 0;
  uint8_t b1 = 0;

  /* ---- VALUE INTERPRETATION ---- */

  if (parsingStyle & PS_ASCII_VALUE) {
    // raw ASCII characters
    b0 = (uint8_t)p1[0];
    b1 = (off2 != off1 && p2) ? (uint8_t)p2[0] : 0;
  } else {
    // hex interpretation
    if (!hexByteAt(p1, b0)) return false;
    if (off2 != off1 && !hexByteAt(p2, b1)) return false;
  }

  /* ---- ENDIANNESS ---- */

  if (off1 == off2) {
    outValue = b0;
  } else if (parsingStyle & PS_BIG_ENDIAN) {
    outValue = ((uint16_t)b0 << 8) | b1;
  } else {
    outValue = ((uint16_t)b1 << 8) | b0;
  }

  return true;
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

uint16_t processorType() {
  uint16_t mcu = 0;
  #if defined(__AVR_ATmega32__)
    mcu = 32;
  #elif defined(__AVR_ATmega32A__)
    mcu = 33;
  #elif defined(__AVR_ATmega8__)
    mcu = 8;  
  #elif defined(__AVR_ATmega8515__)
    mcu = 8515;
  #elif defined(__AVR_ATmega8535__)
    mcu = 8535;
  #elif defined(__AVR_ATmega16__)
    mcu = 16;
  #elif defined(__AVR_ATmega64__)
    mcu = 64;
  #elif defined(__AVR_ATmega128__)
   mcu = 128;
  #elif defined(__AVR_ATmega162__)
    mcu = 162;
  #elif defined(__AVR_ATmega88__)
    mcu = 88;
  #elif defined(__AVR_ATmega168__)
    mcu = 168;  
  #elif defined(__AVR_ATmega328P__)
    mcu = 328;  
  #elif defined(__AVR_ATmega32U__)
    mcu = 34;  
  #elif defined(__AVR_ATmega2560__)
    mcu = 2560;  
  #elif defined(__AVR_ATmega1284P__)
    mcu = 1285;  
  #elif defined(__AVR_ATmega1284__)
    mcu = 1284;  
  #endif
  return mcu;
}

 
 

 
