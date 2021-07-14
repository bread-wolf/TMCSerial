/*
 * TMCSerial.h
 *
 *  Created on: 09.05.2021
 *      Author: AB
 */

#ifndef TMCSERIAL_H_
#define TMCSERIAL_H_

#include<Arduino.h>
#include<stdint.h>
#include<stdbool.h>

#include"TMCField.hpp"

#define TMCSERIAL_READ_ERROR        -1   // Invalid checksum in read
#define TMCSERIAL_READ_TIMEOUT      -2   // Timed out waiting for whole frame
#define TMCSERIAL_WRITE_ERROR       -3   // Write transaction didn't increment IFCNT
#define TMCSERIAL_RW_SUCCESS         0

class TMCSerial
{
    public:
        /* Create instance, set up attributes. */
        TMCSerial(HardwareSerial& serialPort, uint32_t baudrate, uint8_t chipAddress);

        /* Initialize Serial Port. Takes no arguments. */
        void begin();

        /* Safe register update, calls on read/write functions, and forwards error code. */
        void writeField(TMCField field, uint32_t value, int8_t &tmcError);
        uint32_t readField(TMCField field, int8_t &tmcError);

        /* Overloaded register functions, not using error code for simpler syntax. */
        void writeField(TMCField field, uint32_t value);
        uint32_t readField(TMCField field);

        /* Enable/Disable checking for success of write operations. Disabled by default. */
        void enableVerifyWrite(bool enable);

        /* Enable/Disable Checksum calculation on receive data. Disabled by default. */
        void enableReadChecksum(bool enable);

        /* Sets delay for bus switching both for master side, and inside TMC Chip in slaveConf register.
         * Lower 3 bits of bitTimes will be ignored which makes valid times :
         * 8*1, 8*3, 8*5, 8*7, 8*9, 8*11, 8*13, 8*15, values over 120 are ignored. 
         * This is for use with RS485 transceiver that might need delay for switching between transmitting and receiving. */
        void setMasterSlaveDelay(uint32_t bitTimes);
    
    private:
        /* Chip communication parameters */
        HardwareSerial& _serialPort;
        uint32_t _baudrate;
        uint8_t _chipAddress;
    
        /* Read and Write raw registers using its address, returns error code. */
        int8_t writeRegister(uint8_t address, uint32_t &registerValue);
        int8_t readRegister(uint8_t address, uint32_t &registervalue);

        /* Calculate CRC for outgoing and incoming data */
        bool _enableReadChecksum;
        uint8_t calcCRC(uint8_t data[], uint8_t dataLength);        

        /* Check counter for successful write feature */
        bool _enableVerifyWrite;
        uint32_t _writeCounter;
        bool isCounterIncreased();

        /* Sets delay to switch while bus switches from master to slave, and vice-versa. 
         * Input is the number of bits times to wait, _masterSlaveDelay is a value in microseconds. 
         * If sendToChip is True, function will attempt to write to the chip SLAVECONF register. */
        uint32_t _masterSlaveDelay;
        void updateMasterSlaveDelay(uint32_t bitTimes, bool sendToChip);
};

#endif /* TMCSERIAL_H_ */