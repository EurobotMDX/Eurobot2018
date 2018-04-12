Eurobot project 2018

======

# Development Process

### Configured git
### python2 or python3

Cloning a project to local directory

https://github.com/jarzab3/Eurobot2018.git


### After cloning, please go to `robotConfig.py` file and edit settings according to robot specifications

* Change wheel spacing as well as the name for the robot
* Change name and adjust other settings


SENSOR I2C ADDRESS CONVERSION: https://www.raspberrypi.org/forums/viewtopic.php?t=134204

### Library used for a servo board
https://github.com/adafruit/Adafruit_Python_PCA9685

### Issues:
* Disabling a logger for Adafruit i2c

logger = logging.getLogger('Adafruit_I2C.Device')

logger.setLevel(logging.CRITICAL)

### Sensors SRF08

## Docs:
http://coecsl.ece.illinois.edu/ge423/DevantechSRF08UltraSonicRanger.pdf

__Team Leaders:__
- Heeney Michael (m.heeney@mdx.ac.uk)
- Knott Calum (c.knott@mdx.ac.uk)

__Team members:__

- Cole Timbo (timbo@timbotek.com)
- Galabov Momchil (mg1143@live.mdx.ac.uk) 
- Jarzebak Adam (adam@jarzebak.eu)
- Simon Klimek (simonklimek91@gmail.com) 



