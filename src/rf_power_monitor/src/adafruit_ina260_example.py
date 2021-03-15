#!/usr/bin/env python3

import time
import board
import adafruit_ina260
 
i2c = board.I2C()
ina260 = adafruit_ina260.INA260(i2c)
while True:
    print(
        "Current: %.2f mA Voltage: %.2f V Power:%.2f mW"
        % (ina260.current, ina260.voltage, ina260.power)
    )
    time.sleep(1)

# Pi user is a member of these groups...
#
# uid=1000(pi) gid=1000(pi) groups=1000(pi),4(adm),20(dialout),24(cdrom),27(sudo),29(audio),44(video),46(plugdev),60(games),100(users),105(input),109(netdev),997(gpio),998(i2c),999(spi)
#
# Rumblefish a member of....
#
# uid=1001(rumblefish) gid=1001(rumblefish) groups=1001(rumblefish),27(sudo)
#
# Missing but added groups: 29(audio), 100(users), 105(input), 109(netdev),
#                           997(gpio), 998(i2c), 999(spi)


# Setup I2C
#
# sudo apt-get install -y python-smbus
# sudo apt-get install -y i2c-tools
#
# Use sudo raspi-config to enable i2c
#
#
# Should be able to run: sudo i2cdetect -y 1
# if not need to add user to groups gpio and i2c
# 


    
