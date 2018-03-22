/**
 * Copyright (c) 2015, Yaacov Zamir <kobi.zamir@gmail.com>
 * Copyright (c) 2017, Andrew Voznytsa <andrew.voznytsa@gmail.com>, FC_WRITE_REGISTER and FC_WRITE_MULTIPLE_COILS support
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF  THIS SOFTWARE.
 */

#include <assert.h>
#include <string.h>
#include "ModbusSlave.h"

/******************************************************************************/

/**
 * Init the modbus object.
 *
 * @param unitID the modbus slave id.
 * @param ctrlPin the digital out pin for RS485 read/write control.
 */
Modbus::Modbus(uint8_t _unitID, 
               int     _ctrlPin):
Modbus(Serial, _unitID, _ctrlPin) {
}

/******************************************************************************/

/**
 * Init the modbus object.
 *
 * @param serial the serial port used for the modbus communication
 * @param unitID the modbus slave id.
 * @param ctrlPin the digital out pin for RS485 read/write control.
 */
Modbus::Modbus(Stream & _serial, 
               uint8_t  _unitID, 
               int      _ctrlPin):
serial(_serial) {
    unitID = _unitID;
    ctrlPin = _ctrlPin;
}

/******************************************************************************/

void     
Modbus::
setSlaveId(uint8_t address) {
	unitID = address;
}

/******************************************************************************/

/**
 * Begin serial port and set timeout.
 *
 * @param boud the serial port boud rate.
 */
void
Modbus::
begin(unsigned long boud) {
    if (ctrlPin) {
        pinMode(ctrlPin, OUTPUT);
    }

    // set the timeout for 3.5 chars.
    serial.setTimeout(0);

    // set the T35 interframe timeout
    if (boud > 19200) {
        timeout = 1750;
    } else {
        timeout = 35000000 / boud; // 1T * 3.5 = T3.5
    }

    // init last received values
    last_receive_len  = 0;
    last_receive_time = 0;
}

/******************************************************************************/

/**
 * Calculate buffer CRC16
 *
 * @param buf the data buffer.
 * @param length the length of the buffer without CRC.
 *
 * @return the calculated CRC16.
 */
uint16_t 
Modbus::calcCRC(uint8_t * buf, 
                int       length) {
    int      i, j;
    uint16_t crc = 0xFFFF;
    uint16_t tmp;

    for (i = 0; i < length; i++) {
        crc = crc ^ buf[i];
        for (j = 0; j < 8; j++) {
            tmp = crc & 0x0001;
            crc = crc >> 1;
            if (tmp) {
              crc = crc ^ 0xA001;
            }
        }
    }
    return crc;
}

/******************************************************************************/

/**
 * wait for end of frame, parse request and answer it.
 */
int Modbus::poll() {
    int      lengthIn;
    int      lengthOut;
    uint8_t  cb_status = STATUS_OK;
    
    // check if we have data in buffer.
    uint16_t available_len = serial.available();
    if (available_len != 0) {
        // if we have new data, update last received time and length.
        if (available_len != last_receive_len) {
            last_receive_len  = available_len;
            last_receive_time = micros();
            return 0;
        }
        // if no new data, wait for T35 microseconds.
        if (micros() < (last_receive_time + timeout)) {
            return 0;
        }
        // we waited for the inter-frame timeout, read the frame.
        lengthIn = serial.readBytes(bufIn, MAX_BUFFER);
        last_receive_len = 0;
        last_receive_time = 0;
    } else {
        return 0;
    }

    // check unit-id
    if (bufIn[0] != unitID) {
      return 0;
    }
    // check minimum length.
    if (lengthIn < 8) { 
      return 0;
    }

    uint8_t  fc      = bufIn[1];
    uint16_t address = word(bufIn[2], bufIn[3]); // first register.
    uint16_t length  = word(bufIn[4], bufIn[5]); // number of registers to act upone or status.

    // Output length sanity check, and remove trailing noise from message.
    switch (fc) {
        case FC_READ_COILS:             // read coils (digital out state)
        case FC_READ_DISCRETE_INPUT:    // read input state (digital in)
        case FC_READ_HOLDING_REGISTERS: // read holding registers (analog out state)
        case FC_READ_INPUT_REGISTERS:   // read input registers (analog in)
            if (length > MAX_BUFFER) {
                return respondNow(STATUS_ILLEGAL_DATA_ADDRESS, lengthOut, fc, cb_status);
            }
            lengthIn = 8;
            break;
        case FC_WRITE_COIL:
            lengthIn = 8;
            break;
        case FC_WRITE_REGISTER:
            lengthIn = 8;
            break;
        case FC_WRITE_MULTIPLE_COILS:
            if (length > MAX_BUFFER) {
                return respondNow(STATUS_ILLEGAL_DATA_ADDRESS, lengthOut, fc, cb_status);
            }
            // check buffer in size.
            if (lengthIn < (int)(7 + (length + 7) / 8 + 2)) { 
              return 0;
            }
            // ignore tailing nulls.
            lengthIn = (int)(7 + (length + 7) / 8 + 2);
            break;
        case FC_WRITE_MULTIPLE_REGISTERS:
            if (length > MAX_BUFFER) {
                return respondNow(STATUS_ILLEGAL_DATA_ADDRESS, lengthOut, fc, cb_status);
            }

            // check buffer in size.
            if (lengthIn < (int)(7 + length * 2 + 2)) {
              return 0;
            }
            lengthIn = (int)(7 + length * 2 + 2);
            break;
        default: // unknown command
            return respondNow(STATUS_ILLEGAL_FUNCTION, lengthOut, fc, cb_status);
    }

    uint16_t crc = word(bufIn[lengthIn - 1], bufIn[lengthIn - 2]);
    if (calcCRC(bufIn, lengthIn - 2) != crc) {
        // standard modbus doesn't respond in case of crc error
        return 0;
    }

    switch (fc) {
        case FC_READ_COILS: // read coils (digital out state)
        case FC_READ_DISCRETE_INPUT: // read input state (digital in)
            lengthOut = 3 + (length - 1) / 8 + 1 + 2;
            bufOut[2] = (length - 1) / 8 + 1;
            // clear data out.
            memset(bufOut + 3, 0, bufOut[2]);
            cb_status = processUserCallback(CB_READ_COILS, fc, address, length);
            break;
        case FC_READ_HOLDING_REGISTERS: // read holding registers (analog out state)
        case FC_READ_INPUT_REGISTERS: // read input registers (analog in)
            lengthOut = 3 + 2 * length + 2;
            bufOut[2] = 2 * length;
            // clear data out.
            memset(bufOut + 3, 0, bufOut[2]);
            cb_status = processUserCallback(CB_READ_REGISTERS, fc, address, length);
            break;
        case FC_WRITE_COIL: // write one coil (digital out)
            lengthOut = 8;
            memcpy(bufOut + 2, bufIn + 2, 4);
            cb_status = processUserCallback(CB_WRITE_COILS, fc, address, 1);
            break;
        case FC_WRITE_REGISTER:
            lengthOut = 8;
            memcpy(bufOut + 2, bufIn + 2, 4);
            cb_status = processUserCallback(CB_WRITE_REGISTERS, fc, address, 1);
            break;
        case FC_WRITE_MULTIPLE_COILS: // write coils (digital out)
            lengthOut = 8;
            memcpy(bufOut + 2, bufIn + 2, 4);
            cb_status = processUserCallback(CB_WRITE_COILS, fc, address, length);
            break;
        case FC_WRITE_MULTIPLE_REGISTERS: // write holding registers (analog out)
            lengthOut = 8;
            memcpy(bufOut + 2, bufIn + 2, 4);
            cb_status = processUserCallback(CB_WRITE_REGISTERS, fc, address, length);
            break;
    }
    return respondNow(STATUS_OK, lengthOut, fc, cb_status);
}

/******************************************************************************/

int Modbus::respondNow(uint8_t    error, 
                       int        lengthOut, 
                       uint8_t    fc, 
                       uint8_t    cb_status) {
    buildAnswer(error, lengthOut, fc, cb_status);
    transmit(lengthOut);
    return lengthOut;  
}

/******************************************************************************/

uint8_t Modbus::processUserCallback(int      callbackType, 
                                    uint8_t  fc, 
                                    uint16_t address, 
                                    uint16_t length) {
    if (cbVector[callbackType]) {
        return cbVector[callbackType](fc, address, length);
    }
    return STATUS_ILLEGAL_FUNCTION;
}

/******************************************************************************/

void Modbus::buildAnswer(uint8_t    error, 
                         int        lengthOut,
                         uint8_t    fc, 
                         uint8_t    cb_status) {
    bufOut[0] = unitID;
    bufOut[1] = fc;
    
    if (error != STATUS_OK) { // error code should have higher priotity over callback
        bufOut[1] |= 0x80;
        bufOut[2] = error;
        lengthOut = 5;
    } else if (cb_status != STATUS_OK) {
        bufOut[1] |= 0x80;
        bufOut[2] = cb_status;
        lengthOut = 5;
    }
    uint16_t crc = calcCRC(bufOut, lengthOut - 2);
    bufOut[lengthOut - 2] = crc & 0xff;
    bufOut[lengthOut - 1] = crc >> 8;
}

/******************************************************************************/

void Modbus::transmit(int lengthOut) {
    if (ctrlPin) {
        // set rs485 control pin to write
        digitalWrite(ctrlPin, HIGH);

        // send buffer
        serial.write(bufOut, lengthOut);

        // wait for the transmission of outgoing data
        // to complete and then set rs485 control pin to read
        // [ on SoftwareSerial use delay ? ]
        serial.flush();
        digitalWrite(ctrlPin, LOW);
    } else {
        // just send the buffer.
        serial.write(bufOut, lengthOut);
    }
}
/******************************************************************************/

/**
 * Read coil state from input buffer.
 *
 * @param offset the coil offset from first coil in this buffer.
 * @return the coil state from buffer (true / false)
 */
int Modbus::readCoilFromBuffer(int offset) {
    if (bufIn[1] == FC_WRITE_COIL) {
       assert(offset == 0);
       return word(bufIn[4], bufIn[5]) == COIL_ON;
    }

    assert(bufIn[1] == FC_WRITE_MULTIPLE_COILS);

    int address = 7 + offset / 8;
    int bit = offset % 8;

    return bitRead(bufIn[address], bit);
}

/******************************************************************************/

/**
 * Read register value from input buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @return the register value from buffer.
 */
uint16_t Modbus::readRegisterFromBuffer(int offset) {
    if (bufIn[1] == FC_WRITE_REGISTER) {
       assert(offset == 0);
       return word(bufIn[4], bufIn[5]);
    }

    assert(bufIn[1] == FC_WRITE_MULTIPLE_REGISTERS);

    int address = 7 + offset * 2;

    return word(bufIn[address], bufIn[address + 1]);
}

/******************************************************************************/

/**
 * Write coil state to output buffer.
 *
 * @param offset the coil offset from first coil in this buffer.
 * @param state the coil state to write into buffer (true / false)
 */
void Modbus::writeCoilToBuffer(int offset, int state) {
    int address = 3 + offset / 8;
    int bit = offset % 8;

    if (state == HIGH) {
        bitSet(bufOut[address], bit);
    } else {
        bitClear(bufOut[address], bit);
    }
}

/******************************************************************************/

/**
 * Write register value to output buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @param value the register value to write into buffer.
 */
void Modbus::writeRegisterToBuffer(int offset, uint16_t value) {
    int address = 3 + offset * 2;

    bufOut[address] = value >> 8;
    bufOut[address + 1] = value & 0xff;
}

/******************************************************************************/

/**
 * Write arbitrary string of uint8_t to output buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @param str the string to write into buffer.
 * @param length the string length.
 * @return STATUS_OK in case of success, STATUS_ILLEGAL_DATA_ADDRESS
 *      if data doesn't fit in buffer
 */
uint8_t Modbus::writeStringToBuffer(int offset, uint8_t * str, uint8_t length) {
    int address = 3 + offset * 2;

    // check string length.
    // MAX_BUFFER-2 because we ned two bytes for crc. 
    if ((address + length) >= MAX_BUFFER-2) return STATUS_ILLEGAL_DATA_ADDRESS;

    memcpy(bufOut + address, str, length);
    return STATUS_OK;
}