# Arduino I2C Slave With Commands

This is based on the simpler <a href=https://github.com/judasgutenberg/Generic_Arduino_I2C_Slave>Generic_Arduino_I2C_Slave</a> and this firmware is compatible with masters only expecting that firmware.


This adds the ability for an Arduino Slave to run a number of commands issued by a master.  These commands are all reads in the address space between 128 and 255. On an Arduino, such reads are to pins that cannot exist.  For now the commands are:


128: reboot slave (this doesn't work, because doing it breaks I2C irreperably)

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

20X: pet hardware watchdog - if this is implemented, the slave will reboot the master if it is not petted frequently enough.  There are several pet-the-watchdog commands:

200: pets the watchdog, telling it that it must be petted once per second or it can bite (reset!) -- that's way too fast, don't do that.

201: pets the watchdog, telling it that it must be petted once per every ten seconds or it can bite (reset!) -- that's a bit too fast, so probably not too useful.

202: pets the watchdog, telling it that it must be petted once every 100 seconds or it can bite (reset!) -- that's a workable rate, but it's a bit too fussy.

203: pets the watchdog, telling it that it must be petted once every 1000 seconds or it can bite (reset!) -- that will keep your master alive without a lot of unneeded restarts.

204: pets the watchdog, telling it that it must be petted once every 10000 seconds or it can bite (reset!) -- only requires a pet once every 3 hours.  Also a useful rate.

205: pets the watchdog, telling it that it must be petted once every 100000 seconds or it can bite (reset!) -- only requires a pet once every day.  Useful, but not super responsive if the master should lock up.


Since an Atmega328 has a kilobyte of EEPROM, this can be used to store semi-volatile configuration data for a master, as EEPROM has much better wear characteristics than flash (the only storage option on a stock ESP32 or ESP8266 board).
