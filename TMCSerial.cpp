/*
 * TMCSerial.cpp
 *
 *  Created on: 09.05.2021
 *      Author: AB
 */

#include"TMCSerial.hpp"

/* Protocol constants */
#define SYNC_BYTE           0x55
#define WRITE_BIT           0x80

/* These register addresses are the same for all TMC chips which use the single-wire UART IP. 
 * Verified for : 2300, 7300, 2202, 2208, 2209, 2224, 5160, 5161 */
#define TMC_IFCNT           0x02
#define TMC_SLAVECONF       0x03

/* Default constants for protocol timing constraints */
#define DEFAULT_US_DELAY       8
#define DEFAULT_MS_TIMEOUT     5


TMCSerial::TMCSerial(HardwareSerial& serialPort, uint32_t baudrate, uint8_t chipAddress)
    : _serialPort(serialPort), _baudrate(baudrate), _chipAddress(chipAddress), _enableReadChecksum(false), _enableVerifyWrite(false), _writeCounter(0)
{
    /* Set default master to slave delay for correct bus switching. */
    updateMasterSlaveDelay(DEFAULT_US_DELAY, false);
}

void TMCSerial::begin()
{
    /* Initialize Serial port */
    _serialPort.begin(_baudrate);    
}

void TMCSerial::writeField(TMCField field, uint32_t value, uint8_t &tmcError)
{
    /* Get value of old register. */
    uint32_t oldRegisterValue, newRegisterValue;
    tmcError = readRegister(field.address(), oldRegisterValue);

    /* If read fails, return now and forward error code. */
    if (tmcError < 0)
    {
        return;
    }

    /* Update the field, write to chip, and forward error code. */
    newRegisterValue = field.setField(value, oldRegisterValue);
    tmcError = writeRegister(field.address(), newRegisterValue);
}

uint32_t TMCSerial::readField(TMCField field, uint8_t &tmcError)
{   
    uint32_t registerValue;
    tmcError = readRegister(field.address(), registerValue);

    return field.getField(registerValue);
}

void TMCSerial::writeField(TMCField field, uint32_t value)
{
    /* Simply call original function and discard error code. */
    uint8_t errorCode;
    writeField(field, value, errorCode);
}

uint32_t TMCSerial::readField(TMCField field)
{
    uint8_t errorCode;
    return readField(field, errorCode);
}

uint8_t TMCSerial::writeRegister(uint8_t address, uint32_t &registerValue)
{
    /* Prepare write datagram */
    uint8_t addressWrite = address | WRITE_BIT;
    uint8_t writeRequest[8] = {SYNC_BYTE, _chipAddress, addressWrite, 0, 0, 0, 0, 0};

    /* Fill data to be written */
    writeRequest[3] = (registerValue >> 24) & 0xFF;
    writeRequest[4] = (registerValue >> 16) & 0xFF;
    writeRequest[5] = (registerValue >>  8) & 0xFF;
    writeRequest[6] = (registerValue      ) & 0xFF;

    /* Calculate CRC */
    writeRequest[7] = calcCRC(writeRequest, 7);

    /* Write data */
    for (uint8_t j = 0; j < 8; j++)
    {
        _serialPort.write(writeRequest[j]);
    }

    /* Wait until the frame was transmitted fully. 
     * Then wait to ensure clean transition from master to slave. */
    _serialPort.flush();
    delayMicroseconds(_masterSlaveDelay);

    /* If option is enabled, check that write incremented counter */
    if (_enableVerifyWrite && !isCounterIncreased())
    {
        return TMCSERIAL_WRITE_ERROR;
    }

    /* If disabled, or if write correct, increment shadow counter and return SUCCESS */
    return TMCSERIAL_RW_SUCCESS;    
}

uint8_t TMCSerial::readRegister(uint8_t address, uint32_t &registerValue)
{
    /* Prepare read datagram */
    uint8_t readRequest[4] = {SYNC_BYTE, _chipAddress, address, 0};
    uint8_t readReply[8] = {0};

    /* Calculating CRC for read request datagram */
    readRequest[3] = calcCRC(readRequest, 3);

    /* Clear receive buffer in case anything is leftover from previous operations. */
    _serialPort.clear();

    /* Sending read access datagram */
    for (uint8_t j = 0; j < 4; j++)
    {
        _serialPort.write(readRequest[j]);
    }

    /* Wait to collect all 12 bytes (4 loopsback from the request, 8 from the reply). */
    uint8_t byteCounter = 0;
    uint32_t startTick = millis();
    while (byteCounter <= 11)
    {
        if (_serialPort.available())
        {
            /* Discard the loopback bytes (4) */
            if (byteCounter < 4) _serialPort.read();

            /* Save read reply bytes (8) */
            else readReply[byteCounter - 4] = _serialPort.read();

            byteCounter++;
        }
        /* Timeout error. */
        if ((millis() - startTick) > DEFAULT_MS_TIMEOUT)
        {
            registerValue = ~0u;
            return TMCSERIAL_READ_TIMEOUT;
        }
    }
    /* We need to ensure a clean transition from master to slave in case of continous transactions. */
    delayMicroseconds(_masterSlaveDelay);

    /* If option is enabled, calculate read checksum and forward error code. */
    if (_enableReadChecksum && (calcCRC(readReply, 7) != readReply[7]))
    {
        /* Return READ_ERROR, and set value to 0xFFFFFFFF */
        registerValue = ~0u;
        return TMCSERIAL_READ_ERROR;
    }

    /* If option is disabled or checksum is correct we extract the value and return RW_SUCCESS. */
    registerValue = (readReply[3] << 24) | (readReply[4] << 16) | (readReply[5] << 8) | (readReply[6]);
    return TMCSERIAL_RW_SUCCESS;
}

void TMCSerial::enableVerifyWrite(bool enable)
{
    _enableVerifyWrite = enable;
}

void TMCSerial::enableReadChecksum(bool enable)
{
    _enableReadChecksum = enable;
}

void TMCSerial::setMasterSlaveDelay(uint32_t bitTimes)
{
    updateMasterSlaveDelay(bitTimes, true);
}

uint8_t TMCSerial::calcCRC(uint8_t data[], uint8_t dataLength)
{
    uint8_t crc, currentByte;

    /* Initialize crc at 0 */
    crc = 0;
    for (uint8_t j = 0; j < dataLength; j++)
    {
        currentByte = data[j];
        for (uint8_t k = 0; k < 8; k++)
        {
            if ((crc >> 7) ^ (currentByte & 0x01))
            {
                /* Use 0b100000111 as polynomial */
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc = (crc << 1);
            }
            currentByte >>= 1;
        }
    }
    return crc;
}

bool TMCSerial::isCounterIncreased(){
    uint32_t newWriteCounter;
    readRegister(TMC_IFCNT, newWriteCounter);
    if (newWriteCounter != _writeCounter + 1)
    {
        return false;
    }
    else
    {
        _writeCounter++;
        return true;
    }
}

void TMCSerial::updateMasterSlaveDelay(uint32_t bitTimes, bool sendToChip)
{
    /* Maximum supported should be 15 bytes times, or 120 bit times. 
     * To mirror the max value in TMC UART IP from slaveConf register. */
    if (bitTimes > 120)
    {
        bitTimes = 120;
    }

    /* Ignore lower 3 bits of bitTimes value. */
    bitTimes = bitTimes & 0xFFFFFFF8;

    /* Calculate delay in microseconds, depending on set baudrate. */
    _masterSlaveDelay = (1000000 * bitTimes) / _baudrate;

    if (sendToChip)
    {
        uint32_t tmcBitTimes = bitTimes >> 3;
        writeRegister(TMC_SLAVECONF, tmcBitTimes);
    }
}