#include <EEPROM.h>
#include "ModbusSlave.h"
#include "pin.h"

const int DEFAULT_SLAVE_ID = 1;
const int CTRL_PIN         = 2;
const int BAUDRATE         = 9600;

const int ADDR_CFG_PIN_0   = 3;
const int ADDR_CFG_PIN_1   = 4;
const int ADDR_CFG_PIN_2   = 5;
const int ADDR_CFG_PIN_3   = 6;

const int START_CONFIGURABLE_PIN = 7;
const int TOTAL_CONFIGURABLE_PIN = 7;

const int EEPROM_SIZE      = 1024; // Bytes

Modbus slave(DEFAULT_SLAVE_ID, CTRL_PIN);
Pin    pinConfig[TOTAL_CONFIGURABLE_PIN];

/******************************************************************************/

void configurePinMode(uint16_t pinIndex, 
                      uint16_t value) {
    if (CTRL_PIN == pinIndex) {
        return;
    }
    unsigned char mode = value & 1;
    pinConfig[pinIndex].number = pinIndex;
    pinConfig[pinIndex].mode   = mode;
    pinConfig[pinIndex].timer  = 3;//value & 0xFE;
    pinConfig[pinIndex].timerActivated = 0;
    
    switch(mode) {
        case PIN_MODE_INPUT:  pinMode(START_CONFIGURABLE_PIN + pinIndex, INPUT);
                              break;
        case PIN_MODE_OUTPUT: pinMode(START_CONFIGURABLE_PIN + pinIndex, OUTPUT);
                              break;
        default:              pinMode(START_CONFIGURABLE_PIN + pinIndex, OUTPUT);
    }
}

/******************************************************************************/

/**
 * Handle Read Input Status (FC=02)
 * write back the values from digital in pins (input status).
 *
 * handler functions must return void and take:
 *      uint8_t  functionCode - modbus function code.
 *      uint16_t address - first register/coil address.
 *      uint16_t length/status - length of data / coil status.
 */
uint8_t
readDigitalIn(uint8_t  functionCode, 
              uint16_t address, 
              uint16_t length) {
    if (FC_READ_COILS == functionCode ||
        FC_READ_DISCRETE_INPUT == functionCode) {
      for (int coilIndex = 0; coilIndex < length; coilIndex++) {
        slave.writeCoilToBuffer(coilIndex, digitalRead(address + coilIndex));
      }
      return STATUS_OK;
    }
    return STATUS_ILLEGAL_FUNCTION;
}

/******************************************************************************/

void
activateTimer(unsigned char pin) {
    pinConfig[pin].timerActivated = 1;
    pinConfig[pin].internalTimer  = millis();
}

/******************************************************************************/

unsigned char 
timerIsOut(unsigned char pin) {
    if (millis() - pinConfig[pin].internalTimer > 
        pinConfig[pin].internalTimer * 1000) { 
        return 1;
    }
    return 0;
}

/******************************************************************************/

/* Handle Force Single Coil (FC=05) and Force Multiple Coils (FC=15)
 * set digital output pins (coils).
 */
uint8_t 
writeDigitalOut(uint8_t  function, 
                uint16_t address, 
                uint16_t length) {
    (void)function;
    // set digital pin state(s).
    for (int currentCoilIndex = 0; currentCoilIndex < length; currentCoilIndex++) {
        int coilStatus = slave.readCoilFromBuffer(currentCoilIndex);
        digitalWrite(address + currentCoilIndex, coilStatus);
        
        if (1 == coilStatus) {
            activateTimer(0);
        }
    }
    return STATUS_OK;
}

/******************************************************************************/

/* Handle Write Holding Register(s) (FC=06, FC=16)
 * write data into eeprom.
 */
uint8_t
writeMemory(uint8_t  function, 
            uint16_t address, 
            uint16_t length) {
    (void)function;
    uint16_t value;
    uint16_t registerIndex;
    
    // write to eeprom.
    for (int currentRegisterIndex = 0; currentRegisterIndex < length; currentRegisterIndex++) {
        registerIndex = address + currentRegisterIndex;
        // get uint16_t value from the request buffer.
        value = slave.readRegisterFromBuffer(currentRegisterIndex);
        EEPROM.put(registerIndex * 2, value);
        configurePinMode(registerIndex * 2,  value);
    }
    return STATUS_OK;
}

/******************************************************************************/

/* Handle Read Input Registers (FC=04)
 * write back the values from analog in pins (input registers).
 */
uint8_t 
readAnalogIn(uint16_t address, 
             uint16_t length) {
    for (int currentAnalogPin = 0; currentAnalogPin < length; currentAnalogPin++) {
        // write uint16_t value to the response buffer.
        slave.writeRegisterToBuffer(currentAnalogPin, analogRead(address + currentAnalogPin));
    }
}

/******************************************************************************/

/* Handle Read Holding Registers (FC=03)
 * write back the values from eeprom (holding registers).
 */
uint8_t
readMemory(uint8_t  function, 
           uint16_t address, 
           uint16_t length) {
    if (FC_READ_INPUT_REGISTERS == function) { //function 4
        readAnalogIn(address, length);
        return STATUS_OK;
    }
    if (address >= (EEPROM_SIZE/2)-1) { // SIZE/2 = convert in 16 bit length
      return STATUS_ILLEGAL_DATA_ADDRESS;
    }
    uint16_t value;
    for (int registerIndex = 0; registerIndex < length; registerIndex++) {
        EEPROM.get((address + registerIndex) * 2, value);
        slave.writeRegisterToBuffer(registerIndex, value);
    }
    return STATUS_OK;
}

/******************************************************************************/

void checkTimers() {
    for (uint16_t pinIndex = 0; 
         pinIndex < TOTAL_CONFIGURABLE_PIN; 
         pinIndex++) {
        if (0 >= pinConfig[pinIndex].timer || 
            0 == pinConfig[pinIndex].timerActivated) { // no timer for this pin !
            continue;
        }
        if (1 == timerIsOut(pinIndex)) {
            pinConfig[pinIndex].timerActivated = 0;
            pinConfig[pinIndex].internalTimer  = 0;
            //digitalWrite(START_CONFIGURABLE_PIN + pinIndex, 1);
            digitalWrite(8, 0);
        }
    }
}

/******************************************************************************/

void setup() {
    pinMode(ADDR_CFG_PIN_0, INPUT);
    pinMode(ADDR_CFG_PIN_1, INPUT);
    pinMode(ADDR_CFG_PIN_2, INPUT);
    pinMode(ADDR_CFG_PIN_3, INPUT);
    // Read 4 pins (from dipswitch) for address configuration
    int modbusAddr =  (digitalRead(ADDR_CFG_PIN_0) & 0x1) | 
                     ((digitalRead(ADDR_CFG_PIN_1) & 0x1) << 1) |
                     ((digitalRead(ADDR_CFG_PIN_2) & 0x1) << 2) |
                     ((digitalRead(ADDR_CFG_PIN_3) & 0x1) << 3);
    slave.setSlaveId(modbusAddr);

    unsigned char relativePin = 0;
    for (uint16_t pinIndex = START_CONFIGURABLE_PIN; 
         pinIndex < START_CONFIGURABLE_PIN + TOTAL_CONFIGURABLE_PIN; 
         pinIndex++, relativePin++) {
        uint16_t pin = EEPROM.read(pinIndex * 2); //2 for 16 bit registers
        configurePinMode(relativePin, pin); 
    }
    // RS485 control pin must be output
    pinMode(CTRL_PIN, OUTPUT);
    
    // register handler functions into the modbus slave callback vector.
    slave.cbVector[CB_READ_COILS]      = readDigitalIn;
    slave.cbVector[CB_WRITE_COILS]     = writeDigitalOut;
    slave.cbVector[CB_READ_REGISTERS]  = readMemory;
    slave.cbVector[CB_WRITE_REGISTERS] = writeMemory;
    
    Serial.begin(BAUDRATE);
    slave.begin(BAUDRATE);
}

/******************************************************************************/

void loop() {
    /* listen for modbus commands con serial port.
     * on a request, handle the request.
     * if the request has a user handler function registered in cbVector.
     * call the user handler function.
     */ 
    slave.poll();
    checkTimers();
}

/******************************************************************************/

