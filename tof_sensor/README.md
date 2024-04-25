# Set Up VL53L0X Python interface on Raspberry Pi/Jetson TX2

Notes on Multiple sensor support:
- In order to have multiple sensors on the same bus, you must have the shutdown pin of each sensor tied to individual GPIO's so that they can be individually enabled and the addresses set.
- Both the Adafruit and Pololu boards for VL53L0X have I2C pull ups on the board. Because of this, the number of boards that can be added will be limited to only about 5 or 6 before the pull-up becomes too strong.
- Changes to the platform and python_lib c code allow for up to 16 sensors.
- Address changes are volatile so setting the shutdown pin low or removing power will change the address back to the default 0x29.

Notes on using TCA9548A I2C Multiplexer:
- If limited on GPIO's that would be needed to set a new addresses for each sensor, using a TCA9548A I2C Multiplexer is a good option since it allows using up to 8 sensors without using GPIO's.
- The TCA9548A is also a good option if using multiple boards on the same I2C bus and the total of all the combined I2C pullups would cause the bus not to function. 
- Theoretically you can connect multiple TCA9548A Multiplexers, each with up to 8 sensors as long each TCA9548A has a different address. This has not been tested but should work in theory.

### Installation
```bash
# Python2
pip2 install git+https://github.com/pimoroni/VL53L0X-python.git
# Python3
pip3 install git+https://github.com/pimoroni/VL53L0X-python.git
```

### Compilation

* To build on raspberry pi, first make sure you have the right tools and development libraries:
```bash
sudo apt-get install build-essential python-dev
```

Then use following commands to clone the repository and compile:
```bash
cd your_git_directory
git clone https://github.com/pimoroni/VL53L0X_rasp_python.git
cd VL53L0X-python
make
```

* In the Python directory are the following python files:

VL53L0X.py - This contains the python ctypes interface to the ST Library

VL53L0X_example.py - This example accesses a single sensor with the default address.

VL53L0X_example_livegraph.py - This example plots the distance data from a single sensor in a live graph. This example requires matplotlib. Use `sudo pip install matplotlib` to install matplotlib.

VL53L0X_multi_example.py - This example accesses 2 sensors, setting the first to address 0x2B and the second to address 0x2D. It uses GPIOs 20 and 16 connected to the shutdown pins on the 2 sensors to control sensor activation.

![VL53L0X_multi_example.py Diagram](https://raw.githubusercontent.com/johnbryanmoore/VL53L0X_rasp_python/master/VL53L0X_Mutli_Rpi3_bb.jpg "Fritzing Diagram for VL53L0X_multi_example.py")

VL53L0X_TCA9548A_example.py - This example accesses 2 sensors through a TCA9548A I2C Multiplexer with the first connected to bus 1 and the second on bus 2 on the TCA9548A.

![VL53L0X_TCA9548A_example.py Diagram](https://raw.githubusercontent.com/johnbryanmoore/VL53L0X_rasp_python/master/VL53L0X_TCA9548A_Rpi3_bb.jpg "Fritzing Diagram for VL53L0X_TCA9548A_example.py")