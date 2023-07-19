/*
 * I2C_MPR121.cpp
 *
 *  Created on: Oct 14, 2013
 *      Author: Victor Zappi
 */


#include "I2C_MPR121.h"
#include <iostream>

I2C_MPR121::I2C_MPR121() {

}
////start here////
/////////
boolean I2C_MPR121::begin(uint8_t bus, uint8_t i2caddr) {
  _i2c_address = i2caddr;
	
  if(initI2C_RW(bus, i2caddr, 0) > 0)
	  return false;
	  
  // soft reset
  //writeRegister(MPR121_SOFTRESET, 0x63);
  //usleep(1000);
  //delay(1);
  
  //writeRegister(MPR121_ECR, 0x0);
	std::cout << "checking connection\n";
	uint8_t c = readRegister8(APDS9960_ID);
  
  if (c != 0xAB) {
	  rt_printf("MPR121 read 0x%x instead of 0xAB\n", c);
	  return false;
  } else {
  	std::cout << static_cast<int>(c)<< "\n";
  }
  /* Set default integration time and gain */
	//setADCIntegrationTime(iTimeMS);
	std::cout << "reset everything\n";
	enableProximity(false);
	disableProximityInterrupt();
	usleep(1000);
	rt_printf("hello lee\n");
	return true;
}

void I2C_MPR121::setThresholds(uint8_t touch, uint8_t release) {
  for (uint8_t i=0; i<12; i++) {
    writeRegister(MPR121_TOUCHTH_0 + 2*i, touch);
    writeRegister(MPR121_RELEASETH_0 + 2*i, release);
  }
}

uint16_t  I2C_MPR121::filteredData(uint8_t t) {
  if (t > 12) return 0;
  return readRegister16(MPR121_FILTDATA_0L + t*2);
}

uint16_t  I2C_MPR121::baselineData(uint8_t t) {
  if (t > 12) return 0;
  uint16_t bl = readRegister8(MPR121_BASELINE_0 + t);
  return (bl << 2);
}

uint16_t  I2C_MPR121::touched(void) {
  uint16_t t = readRegister16(MPR121_TOUCHSTATUS_L);
  return t & 0x0FFF;
}

/*!
 *  @brief  Read proximity data
 *  @return Proximity
 */
uint8_t I2C_MPR121::readProximity() {
	//return readRegister8(APDS9960_PDATA); 
	return readRegister8(APDS9960_STATUS); // works to get the ID
}

/*!
 *  @brief  Enable proximity readings on APDS9960
 *  @param  en
 *          Enable (True/False)
 */
void I2C_MPR121::enableProximity(boolean en) {
  _enable.PEN = en;
  std::cout << "PEN sent as " << en << "\n";
    writeRegister(APDS9960_ENABLE, _enable.get());
}
/*!
 *  @brief  Returns the Proximity gain on the APDS9960
 *  @return Proxmity gain
 */
apds9960PGain_t I2C_MPR121::getProxGain() {
  return (apds9960PGain_t)((readRegister8(APDS9960_CONTROL) & 0x0C) >> 2);
}
/*!
 *  @brief  Adjusts the Proximity gain on the APDS9960
 *  @param  pGain
 *          Gain
 */
void I2C_MPR121::setProxGain(apds9960PGain_t pGain) {
  _control.PGAIN = pGain;

  /* Update the timing register */
  writeRegister(APDS9960_CONTROL, _control.get());
}

/*!
 *  @brief  Sets number of proxmity pulses
 *  @param  pLen
 *          Pulse Length
 *  @param  pulses
 *          Number of pulses
 */
void I2C_MPR121::setProxPulse(apds9960PPulseLen_t pLen, uint8_t pulses) {
  if (pulses < 1)
    pulses = 1;
  if (pulses > 64)
    pulses = 64;
  pulses--;
  _ppulse.PPLEN = pLen;
  _ppulse.PPULSE = pulses;
  writeRegister(APDS9960_PPULSE, _ppulse.get());
}
/*!
 *  @brief  Enable proximity interrupts
 */
void I2C_MPR121::enableProximityInterrupt() {
  _enable.PIEN = 1;
  writeRegister(APDS9960_ENABLE, _enable.get());
  clearInterrupt();
}

/*!
 *  @brief  Disable proximity interrupts
 */
void I2C_MPR121::disableProximityInterrupt() {
  _enable.PIEN = 0;
  writeRegister(APDS9960_ENABLE, _enable.get());
}

/*!
 *  @brief  Clears interrupt
 */
void I2C_MPR121::clearInterrupt() {
	std::cout << "clear interrupt\n";
  writeRegister(APDS9960_AICLEAR, 0); // need to work on this
}
/*!
 *  @brief  Sets the integration time for the ADC of the APDS9960, in millis
 *  @param  iTimeMS
 *          Integration time
 */
void I2C_MPR121::setADCIntegrationTime(uint16_t iTimeMS) {
  float temp;

  // convert ms into 2.78ms increments
  temp = iTimeMS;
  temp /= 2.78;
  temp = 256 - temp;
  if (temp > 255)
    temp = 255;
  if (temp < 0)
    temp = 0;

  /* Update the timing register */
  writeRegister(APDS9960_ATIME, (uint8_t)temp);
}
/*!
 *  @brief  Returns the integration time for the ADC of the APDS9960, in millis
 *  @return Integration time
 */
float I2C_MPR121::getADCIntegrationTime() {
  float temp;

  temp = readRegister8(APDS9960_ATIME);

  // convert to units of 2.78 ms
  temp = 256 - temp;
  temp *= 2.78;
  return temp;
}
/*!
 *  @brief  Set proxmity interrupt thresholds
 *  @param  low
 *          Low threshold
 *  @param  high
 *          High threshold
 *  @param  persistence
 *          Persistence
 */
void I2C_MPR121::setProximityInterruptThreshold(uint8_t low,
                                                       uint8_t high,
                                                       uint8_t persistence) {
  writeRegister(APDS9960_PILT, low);
  writeRegister(APDS9960_PIHT, high);

  if (persistence > 7)
    persistence = 7;
  _pers.PPERS = persistence;
  writeRegister(APDS9960_PERS, _pers.get());
}

/*********************************************************************/


uint8_t I2C_MPR121::readRegister8(uint8_t reg) {
    i2c_char_t inbuf, outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
    std::cout << "command sent to device " << (int)reg << "\n";
    outbuf = reg;
    messages[0].addr  = _i2c_address;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    /* The data will get returned in this structure */
    messages[1].addr  = _i2c_address;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = &inbuf;

    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(i2C_file, I2C_RDWR, &packets) < 0) {
        rt_printf("Unable to send data 8\n");
        return 0;
    }
    //std::cout << static_cast<int>(inbuf)<< "\n";
    return inbuf;
}



uint16_t I2C_MPR121::readRegister16(uint8_t reg) {
    i2c_char_t inbuf[2], outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
    outbuf = reg;
    messages[0].addr  = _i2c_address;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    /* The data will get returned in this structure */
    messages[1].addr  = _i2c_address;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = inbuf;

    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(i2C_file, I2C_RDWR, &packets) < 0) {
        rt_printf("Unable to send data 16\n");
        return 0;
    }

    return (uint16_t)inbuf[0] | (((uint16_t)inbuf[1]) << 8);
}

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void I2C_MPR121::writeRegister(uint8_t reg, uint8_t value) {
	uint8_t buf[2] = { reg, value };
	std::cout << "Writing " << (int)reg << " " <<(int)value << " to register\n";
	if(write(i2C_file, buf, 2) != 2)
	{
		std::cout << "Failed to write register " << (int)reg << " " <<(int)value << " on MPR121\n";
		return;
	}
}



