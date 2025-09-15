# Arduino I2C Slave With Commands

This is based on the simpler <a href=https://github.com/judasgutenberg/Generic_Arduino_I2C_Slave>Generic_Arduino_I2C_Slave</a>


This adds the ability for an Arduino Slave to run a number of commands issued by a master.  These commands are all reads in the address space between 128 and 255. On an Arduino, such reads are to pins that cannot exist.  For now the two commands are:


128: reboot slave

129: return millis() value

130: return millis() of last watchdog reboot

20X: pet hardware watchdog - if this is implemented, the slave will reboot the master if it is not petted frequently enough.  There are several pet-the-watchdog commands:

200: pets the watchdog, telling it that it must be petted once per second or it can bite (reset!) -- that's way too fast, don't do that.

201:  pets the watchdog, telling it that it must be petted once per every ten seconds or it can bite (reset!) -- that's a bit too fast, so probably not too useful.

202: pets the watchdog, telling it that it must be petted once every 100 seconds or it can bite (reset!) -- that's a workable rate, but it's a bit too fussy.

203: pets the watchdog, telling it that it must be petted once every 1000 seconds or it can bite (reset!) -- that will keep your master alive without a lot of unneeded restarts.

204: pets the watchdog, telling it that it must be petted once every 10000 seconds or it can bite (reset!) -- only requires a pet once every 3 hours.  Also a useful rate.

205: pets the watchdog, telling it that it must be petted once every 100000 seconds or it can bite (reset!) -- only requires a pet once every day.  Useful, but not super responsive if the master should lock up.
