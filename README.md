# Arduino I2C Slave With Commands

This is based on the simpler <a href=https://github.com/judasgutenberg/Generic_Arduino_I2C_Slave>Generic_Arduino_I2C_Slave</a>


This adds the ability for an Arduino Slave to run a number of commands issued by a master.  These commands are all reads in the address space between 128 and 255. On an Arduino, such reads are to pins that cannot exist.  For now the two commands are:


128: reboot slave

129: pet hardware watchdog - if this is implemented, the slave will reboot the master if it is not petted frequently enough.
