# TMCSerial

Arduino-style library to communicate with Trinamic's single-wire UART based ICs.

Supported ICs include :
 - TMC7300
 - TMC2300
 - TMC516X Series
 - TMC22XX Series

This library abstracts writing/reading to/from the chip registers, without hiding functionality behind another layer.

### Including in projects

This library depends on the TMCField library.
The two can be easily included in a platformio project by just cloning into the _/lib_ folder as such:

```
cd /lib
git clone https://github.com/A-Bielefeld/TMCField.git
git clone https://github.com/A-Bielefeld/TMCSerial.git
```

The better way is to add these two as submodules in a git repository of the project :

```
git submodule add https://github.com/A-Bielefeld/TMCField.git /lib
git submodule add https://github.com/A-Bielefeld/TMCSerial.git /lib
git commit -m "Added TMCSerial and its dependencies to project."
git push
```

Then pulling updates is as easy as running :

```
git submodule update --init --recursive
```

## Basic usage

### Terminology :
Trinamic chip have a register based interface made up of 32 bits-wide registers, mapped to a 7 bits address space (For a maximum of 128 registers). A register often holds more than one functionality, and the 32 bits can be portionned into several _fields_. This library abstracts registers away and allows direct reading and writing of _fields_.

### Setup :

Include the TMCSerial header, as well as the needed _Fields header for the chosen IC.

```C++
#include"TMCSerial.h"
#include"TMCXXXX/TMCXXXX_Fields.h"
```

One just needs to create a TMCSerial object, and call the begin() method to initialize it. 

Outside setup():
```C++
TMCSerial TMCxxxx(Serial, 115200, 0);
```
 - _Serial port :_ Any serial port can be used as long as the hardware is set up correctly (see hardware section).
 - _Baudrate :_ The library was tested from 9600 bauds to 500 kbauds.
  - _Address :_ The Chip address is defined by the AD0 and AD1 pins, and is between 0 and 3.

Inside setup()/loop():

```C++
TMCxxxx.begin();
```

Up to 4 chips can be placed on the same bus, in which case the TMCSerial.begin() function needs only be called once per UART bus used since it only hides a call to the Arduino Serial.begin() function.

### Reading and writing to/from registers

Use the writeField and readField methods using the objects defined in the included TMCxxxx_Fields.h.

Field objects follow the naming conventions from the Trinamic datasheets.

```C++
void writeField(TMCField field, uint32_t value);
uint32_t readField(TMCField field);
```

Example :

```C++
/* Use the external capacitor mode for TMC7300 */
TMCxxxx.writeField(TMC7300_EXTCAP, 1);

/* Check if driver is in error state */
if (TMCxxx.readField(TMC7300_DRV_ERR) == 1)
{
    /* Do something... */
}

```

## Hardware setup

This library is meant to be used with classical full duplex uart ports. This requires some setup to work with Trinamic single-uart system.

 - A 1K resistor needs to be placed between Rx and Tx, with the Rx pin directly connected to the single-wire line, and the Tx pin connected through the resistor.
 - The AD0 and AD1 pins need to be set to determined voltage, simplest way is to connect both to ground to use default address 00.
 - The driver Enable pin can be pulled to ground for first tests, but some registers won't allow write access if the driver is not enabled.
 - __The driver won't answer at all__ if it doesn have VM power. For most stepper drivers, this means at least 12V on the VM pin!
 - Of course the driver needs its VCC_IO supply, 3.3V is the default, sometimes the drivers are 5V compliant.

## ToDo:

_Documentation:_
 - [ ] Add illustrations for Hardware setup
 - [ ] Document advanced usage
 - [ ] Scope shots for advanced features (Timing)
 - [ ] Document use with RS485
 - [ ] Document multi-chip use

_Tests :_
 - [x] Test on Teensy LC
 - [ ] Test on Teensy 4.0
 - [ ] Test on Arduino Uno
 - [ ] Test on Feather M0 Express
 - [ ] Test with RS485
 - [ ] Test SlaveMaster delay update
 - [ ] Test IFCNT Checks

_Code :_
 - [x] Add registers for remaining supported ICs
 - [ ] Add map for TMC516X
 - [x] Add map for TMC2300
 - [ ] Add examples for TMC7300
 - [ ] Add examples for TMC2209
 - [ ] Restructure into submodulable library
 - [ ] Publish as Arduino library?