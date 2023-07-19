/*
 * MPR121 Bela demo
 * 
 * Andrew McPherson
 * Based on Adafruit library by Limor Fried/Ladyada
 */

#ifndef I2CTK_H_
#define I2CTK_H_

#include <I2c.h>
#include <Utilities.h>
#include <stdint.h>

typedef bool boolean;

#define MPR121_I2CADDR_DEFAULT 0x39

#define MPR121_TOUCHSTATUS_L 0x00
#define MPR121_TOUCHSTATUS_H 0x01
#define MPR121_FILTDATA_0L  0x04
#define MPR121_FILTDATA_0H  0x05
#define MPR121_BASELINE_0   0x1E
#define MPR121_MHDR         0x2B
#define MPR121_NHDR         0x2C
#define MPR121_NCLR         0x2D
#define MPR121_FDLR         0x2E
#define MPR121_MHDF         0x2F
#define MPR121_NHDF         0x30
#define MPR121_NCLF         0x31
#define MPR121_FDLF         0x32
#define MPR121_NHDT         0x33
#define MPR121_NCLT         0x34
#define MPR121_FDLT         0x35

#define MPR121_TOUCHTH_0    0x41
#define MPR121_RELEASETH_0    0x42
#define MPR121_DEBOUNCE 0x5B
#define MPR121_CONFIG1 0x5C
#define MPR121_CONFIG2 0x5D
#define MPR121_CHARGECURR_0 0x5F
#define MPR121_CHARGETIME_1 0x6C
#define MPR121_ECR 0x5E
#define MPR121_AUTOCONFIG0 0x7B
#define MPR121_AUTOCONFIG1 0x7C
#define MPR121_UPLIMIT   0x7D
#define MPR121_LOWLIMIT  0x7E
#define MPR121_TARGETLIMIT  0x7F

#define MPR121_GPIODIR  0x76
#define MPR121_GPIOEN  0x77
#define MPR121_GPIOSET  0x78
#define MPR121_GPIOCLR  0x79
#define MPR121_GPIOTOGGLE  0x7A

#define MPR121_SOFTRESET 0x80

/** I2C Registers */
enum {
  APDS9960_RAM = 0x00,
  APDS9960_ENABLE = 0x80,
  APDS9960_ATIME = 0x81,
  APDS9960_WTIME = 0x83,
  APDS9960_AILTIL = 0x84,
  APDS9960_AILTH = 0x85,
  APDS9960_AIHTL = 0x86,
  APDS9960_AIHTH = 0x87,
  APDS9960_PILT = 0x89,
  APDS9960_PIHT = 0x8B,
  APDS9960_PERS = 0x8C,
  APDS9960_CONFIG1 = 0x8D,
  APDS9960_PPULSE = 0x8E,
  APDS9960_CONTROL = 0x8F,
  APDS9960_CONFIG2 = 0x90,
  APDS9960_ID = 0x92,
  APDS9960_STATUS = 0x93,
  APDS9960_CDATAL = 0x94,
  APDS9960_CDATAH = 0x95,
  APDS9960_RDATAL = 0x96,
  APDS9960_RDATAH = 0x97,
  APDS9960_GDATAL = 0x98,
  APDS9960_GDATAH = 0x99,
  APDS9960_BDATAL = 0x9A,
  APDS9960_BDATAH = 0x9B,
  APDS9960_PDATA = 0x9C,
  APDS9960_POFFSET_UR = 0x9D,
  APDS9960_POFFSET_DL = 0x9E,
  APDS9960_CONFIG3 = 0x9F,
  APDS9960_GPENTH = 0xA0,
  APDS9960_GEXTH = 0xA1,
  APDS9960_GCONF1 = 0xA2,
  APDS9960_GCONF2 = 0xA3,
  APDS9960_GOFFSET_U = 0xA4,
  APDS9960_GOFFSET_D = 0xA5,
  APDS9960_GOFFSET_L = 0xA7,
  APDS9960_GOFFSET_R = 0xA9,
  APDS9960_GPULSE = 0xA6,
  APDS9960_GCONF3 = 0xAA,
  APDS9960_GCONF4 = 0xAB,
  APDS9960_GFLVL = 0xAE,
  APDS9960_GSTATUS = 0xAF,
  APDS9960_IFORCE = 0xE4,
  APDS9960_PICLEAR = 0xE5,
  APDS9960_CICLEAR = 0xE6,
  APDS9960_AICLEAR = 0xE7,
  APDS9960_GFIFO_U = 0xFC,
  APDS9960_GFIFO_D = 0xFD,
  APDS9960_GFIFO_L = 0xFE,
  APDS9960_GFIFO_R = 0xFF,
};

/** Proxmity gain settings */
typedef enum {
  APDS9960_PGAIN_1X = 0x00, /**< 1x gain */
  APDS9960_PGAIN_2X = 0x01, /**< 2x gain */
  APDS9960_PGAIN_4X = 0x02, /**< 4x gain */
  APDS9960_PGAIN_8X = 0x03  /**< 8x gain */
} apds9960PGain_t;

/** Pulse length settings */
typedef enum {
  APDS9960_PPULSELEN_4US = 0x00,  /**< 4uS */
  APDS9960_PPULSELEN_8US = 0x01,  /**< 8uS */
  APDS9960_PPULSELEN_16US = 0x02, /**< 16uS */
  APDS9960_PPULSELEN_32US = 0x03  /**< 32uS */
} apds9960PPulseLen_t;


class I2C_MPR121 : public I2c
{
public:
	// Hardware I2C
	I2C_MPR121();

	boolean begin(uint8_t bus = 1, uint8_t i2caddr = MPR121_I2CADDR_DEFAULT);
	

	uint16_t filteredData(uint8_t t);
	uint16_t  baselineData(uint8_t t);
	// proximity
	void setADCIntegrationTime(uint16_t iTimeMS);
  float getADCIntegrationTime();
  void enableProximity(boolean en = true);
  void setProxGain(apds9960PGain_t gain);
  apds9960PGain_t getProxGain();
  void setProxPulse(apds9960PPulseLen_t pLen, uint8_t pulses);
  void enableProximityInterrupt();
  void disableProximityInterrupt();
  uint8_t readProximity();
  void setProximityInterruptThreshold(uint8_t low, uint8_t high,
                                      uint8_t persistence = 4);
  bool getProximityInterrupt();
  void clearInterrupt();

	uint8_t readRegister8(uint8_t reg);
	uint16_t readRegister16(uint8_t reg);
	void writeRegister(uint8_t reg, uint8_t value);
	uint16_t touched(void);
 
 	void setThresholds(uint8_t touch, uint8_t release);
	
	int readI2C() { return 0; } // Unused
	
private:
	int _i2c_address;
	uint8_t read(uint8_t reg, uint8_t *buf, uint8_t num);
	void writeC(uint8_t reg, uint8_t *buf, uint8_t num);
	struct enable {

    // power on
    uint8_t PON : 1;

    // ALS enable
    uint8_t AEN : 1;

    // Proximity detect enable
    uint8_t PEN : 1;

    // wait timer enable
    uint8_t WEN : 1;

    // ALS interrupt enable
    uint8_t AIEN : 1;

    // proximity interrupt enable
    uint8_t PIEN : 1;

    // gesture enable
    uint8_t GEN : 1;

    uint8_t get() {
      return (GEN << 6) | (PIEN << 5) | (AIEN << 4) | (WEN << 3) | (PEN << 2) |
             (AEN << 1) | PON;
    };
  };
  struct enable _enable;
  struct pers {
    // ALS Interrupt Persistence. Controls rate of Clear channel interrupt to
    // the host processor
    uint8_t APERS : 4;

    // proximity interrupt persistence, controls rate of prox interrupt to host
    // processor
    uint8_t PPERS : 4;

    uint8_t get() { return (PPERS << 4) | APERS; };
  };
  pers _pers;
  struct control {
    // ALS and Color gain control
    uint8_t AGAIN : 2;

    // proximity gain control
    uint8_t PGAIN : 2;

    // led drive strength
    uint8_t LDRIVE : 2;

    uint8_t get() { return (LDRIVE << 6) | (PGAIN << 2) | AGAIN; }
  };
  control _control;
  struct ppulse {

    /*Proximity Pulse Count. Specifies the number of proximity pulses to be
    generated on LDR. Number of pulses is set by PPULSE value plus 1.
    */
    uint8_t PPULSE : 6;

    // Proximity Pulse Length. Sets the LED-ON pulse width during a proximity
    // LDR pulse.
    uint8_t PPLEN : 2;

    uint8_t get() { return (PPLEN << 6) | PPULSE; }
  };
  ppulse _ppulse;
};


#endif /* I2CTK_H_ */
