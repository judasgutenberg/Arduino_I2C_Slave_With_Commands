# Arduino I2C Slave With Commands

This is based on the simpler <a href=https://github.com/judasgutenberg/Generic_Arduino_I2C_Slave>Generic_Arduino_I2C_Slave</a> and this firmware is compatible with I2C masters only expecting that firmware.  An I2C Slave is a microcontroller connected to a master via I2C signals that responds to instructions sent via I2C from the master.  The master can use a slave to control additional GPIO pins, access its EEPROM, or even have the slave act as a watchdog to reset it should it fail to occasionally "pet" the slave.

master_slave.ino (and master_slave.h if you want master_slave.ino to instead be a .cpp file) are a library of functions for communicating with this slave from a master.  This is taken directly from my ESP8266_Remote repository and will probably need tweaking to work with your code (for example, it refers to a global, config, and utility files that aren't in this repository).


This version adds the ability for an Arduino Slave to run a number of commands issued by a master.  These commands are all pin actions to pins numbered between 128 and 255. On an Arduino, such actions are to pin numbers that are beyond those likely to be present (at least as of 2025), so they are available for a wide range of uses.  For now the commands are:


128: reboot slave  

129: return millis() value of slave

130: return millis() of last watchdog reboot from slave (0 for never rebooted)

131: return number of reboots since the slave booted

132: return millis of last time watchdog was petted (0 for never petted)

133: seconds ago that the watchdog was petted when it last bit (0 for never bit)

134: make slave reboot the master now

150: set pointer for EEPROM read/write

151: sequential EEPROM write mode

152: sequential EEPROM read mode

153: exit EEPROM mode, back to default behavior

160: get firmware version

161: get unix time of last firmware compile

162: get onboard temperature (or whatever that glitchy value is)

163: get free memory

164:  get slave configuration

170: set serial baud rate and start serial port

171: retrieve data from serial buffer

172: populate serial buffer for transmission

173: get unix time of last successful serial data parse

174: get last parsed data arriving via serial

175: set parsed serial packet offset

176: get specific parsed serial datum (by ordinal)

177: get number of serial parse configurations

178: get size of parsed serial packet in bytes

179: set serial mode (0: no serial, 1: serial monitor, 2: parse serial for data using parse configuration)

180: set UNIX time

181: get UNIX time

182: get slave configuration given index

183: set slave configuration at index with value

20X: pet hardware watchdog - if this is implemented, the slave will reboot the master if it is not petted frequently enough.  This mechanism also allows the master to send the Unix timestamp so the the slave can keep track of Unix time independently. There are a range of pet-the-watchdog commands:

200: pets the watchdog, telling it that it must be petted once per second or it can bite (reset!) -- that's way too fast, don't do that.

201: pets the watchdog, telling it that it must be petted once per every ten seconds or it can bite (reset!) -- that's a bit too fast, so probably not too useful.

202: pets the watchdog, telling it that it must be petted once every 100 seconds or it can bite (reset!) -- that's a workable rate, but it's a bit too fussy.

203: pets the watchdog, telling it that it must be petted once every 1000 seconds or it can bite (reset!) -- that will keep your master alive without a lot of unneeded restarts.

204: pets the watchdog, telling it that it must be petted once every 10000 seconds or it can bite (reset!) -- only requires a pet once every 3 hours.  Also a useful rate.

One particularly useful feature is the serial parser.  This is a system where a slave can be configured to monitor a serial line (one that may not actually attach to the master) looking for certain passages of text to look focus on.  When it detects these, it can then look for data at offsets within these passages and then assemble integers from the data to place in a data packet for retrieval by the master.  This completely offloads serial parsing from the master.  I am using this feature to monitor serial traffic between a SolArk inverter and its WiFi dongle to capture important solar inverter data.

205: pets the watchdog, telling it that it must be petted once every 100000 seconds or it can bite (reset!) -- only requires a pet once every day.  Useful, but not super responsive if the master should lock up.


Since an Atmega328 has a kilobyte of EEPROM, this can be used to store semi-volatile configuration data for a master, as EEPROM has much better wear characteristics than flash (the only storage option on a stock ESP32 or ESP8266 board).  An advantage of storing configuration data on the slave is that the slave then becomes the "personality module."  This means the masters can all run identical firmware, and you distinguish them by either giving them slaves containing different EEPROM configurations or you set their personalities up remotely.
