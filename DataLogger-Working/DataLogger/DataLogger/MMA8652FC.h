/*
 * MMA8652FC.h
 *
 * Created: 01/02/2016 18:35:12
 *  Author: Tiago
 */ 

#ifndef MMA8652FC_H_
#define MMA8652FC_H_

#include <stdint.h>

#define MMA_ADDRESS (0x1D<<1)



//Register map
#define F_STATUS		0x00
#define OUT_X_MSB		0x01
#define OUT_X_LSB		0x02
#define OUT_Y_MSB		0x03
#define OUT_Y_LSB		0x04
#define	OUT_Z_MSB		0x05
#define OUT_Z_LSB		0x06
#define F_SETUP			0x09
#define TRIG_CFG		0x0A
#define SYSMOD			0x0B
#define INT_SOURCE		0x0C
#define WHO_AM_I		0x0D
#define XYZ_DATA_CFG	0x0E
#define HP_FILTER_CUT	0x0F
#define PL_STATUS		0x10
#define PL_CFG			0x11
#define PL_COUNT		0x12
#define PL_BF_ZCOMP		0x13
#define PL_THS_REG		0x14
#define FF_MT_CFG		0x15
#define FF_MT_SRC		0x16
#define FF_MT_THS		0x17
#define FF_MT_COUNT		0x18

#define TRANS_CFG		0x1D
#define TRANS_SRC		0x1E
#define TRANS_THS		0x1F
#define TRANS_COUNT		0x20
#define PULSE_CFG		0x21
#define PULSE_SRC		0x22
#define PULSE_THSX		0x23
#define PULSE_THSY		0x24
#define PULSE_THSZ		0x25
#define PULSE_TMLT		0x26
#define PULSE_LTCY		0x27
#define PULSE_WIND		0x28
#define ASLP_COUNT		0x29
#define CTRL_REG1		0x2A
#define CTRL_REG2		0x2B
#define CTRL_REG3		0x2C
#define CTRL_REG4		0x2D
#define CTRL_REG5		0x2E
#define OFF_X			0x2F
#define OFF_Y			0x30
#define OFF_Z			0x31

#define EXPECTED_ID		0x4A	//0b01001010

enum STATUS_bits {
	XDR = 0,
	YDR = 1,
	ZDR = 2,
	ZYXDR = 3,
	XOW = 4,
	YOW = 5,
	ZOW = 6,
	ZYXOW = 7
};

enum FIFO_bits {
	 F_WATERMARK = 6,
	 F_OVF
};
	
enum FIFO_SET_bits {			//00 -> Disabled
	F_MODE0 = 6,		//01 -> Circular buffer
	F_MODE1				//10 -> FIFO stops after overflow
};						//11 -> Trigger mode, read up to 
	
enum TRIGGER_bits {			//0x0A - TRIG_CFG
	TRIG_FF_MT = 2,		//Freefall or motion
	TRIG_PULSE,			//Pulse interrupt
	TRIG_LNDPRT,		//Landscape portrait orientation interrupt
	TRIG_TRANS			//Transient interrupt
};

enum SYS_MOD_bits {
	SMOD0,				// 00 - Standby(default)
	SMOD1,				// 01 Wake mode  10 SLEEP
	FGERR = 7			// 1 -> FIFO gate error detected
};

enum INTSOURCE_bits {
	SRC_DRDY,			//data is ready
	SRC_FF_MT = 2,		//The Freefall/Motion function interrupt is active.
	SRC_PULSE,			//An interrupt was generated due to single and/or double pulse event
	SRC_LNDPRT,			//An interrupt was generated due to a change in the device orientation status
	SRC_TRANS,			//An acceleration transient value greater than user-specified threshold has occurred
	SRC_FIFO,			//A FIFO interrupt event (such as an overflow event or watermark) has occurred
	SRC_ASLP			//An interrupt event that can cause a WAKE-to-SLEEP or SLEEP-to-WAKE system mode transition has occurred.
};

enum XYZ_DATA_bits {
	RANGE_2G,			//Default is range 2g
	RANGE_4G,
	RANGE_8G,
	HPF_OUT = 4			//High pass filter on/off
};

enum HP_FILTER_bits {
	SEL0,				//Read page 32 of datasheet to understand this
	SEL1,
	PULSE_LPF_EN=3,		//LPF is on for pulse processing
	PULSE_HPF_BYP		//HPF is bypassed for pulse processing
};

enum PL_STATUS_bits {
	BAFRO,				//device is in back or front
	LAPO0,				//00 -> Port up, 01 -> port down
	LAPO1,				//10 -> Landscape right, 11-> Landscape left
	LO = 6,				//Ztilt lockout trip angle as been exceede
	NEWLP				//BAFRO and or LAPOT and or Z-Tilt have changed, poll them to know more
};

enum PL_CFG_bits {
	PL_EN = 6,
	PL_CFG_DBCNTM	
};


enum PL_BF_ZCOMP_bits {
	ZLOCK0,
	ZLOCK1,
	ZLOCK2,
	BKFR0 = 6,
	BKFR1
};
	
enum FF_MT_CFG_bits {
	XEFE = 3,
	YEFE,
	ZEFE,
	OAE,
	FF_MT_CFG_ELE
};

enum FF_MT_SRC_bits {
	XHP,		//Polarity 1 means positive, 0 meas negativa
	XHE,		//Event flag, 1 means axis movement was detected
	YHP,
	YHE,
	ZHP,
	ZHE,
	FF_MT_SRC_EA = 7
};


enum FF_MT_THS_bits{
	FF_MT_THS_DBCNTM = 7
};
	
enum TRANS_CFG_bits {
	HPF_BYP,
	XTEFE,
	YTEFE,
	ZTEFE,
	TRANS_CFGELE	
};

enum TRANS_SRC_bits {
	X_TRANS_POL,
	XTRANSE,
	YTRANS_POL,
	YTRANSE,
	Z_TRANS_POL,
	ZTRANSE,
	TRAN_SRC_EA	
};

enum TRANS_THS_bits {
	DBCNTM = 7
};

enum PULSE_CGF_bits {
	XSPEFE,
	XDPEFE,
	YSPEFE,
	YDPEFE,
	ZSPEFE,
	ZDPEFE,
	ELE,
	DPA
};

enum PULSE_SRC_bits {
	POLX,
	POLY,
	POLZ,
	DPE,
	AxX,
	AxY,
	AxZ,
	PULSE_SRC_EA
};

enum CONTROL_REG1_bits{
	ACTIVE,
	F_READ,
	DR0 = 3,
	DR1,
	DR2,
	ASLP_RATE0,
	ASLP_RATE1	
};

enum CONTROL_REG2_bits{
	MODS0,
	MODS1,
	AUTO_SLEEP,
	SMODS0,
	SDMOS1,
	RST = 6,		//Software reset
	ST				//Self-test enab
};

enum CONTROL_REG3_bits{
	PP_OD,			//Interrupt line push-pull(LOW, default), or open drain(HIHG)
	IPOL,
	WAKE_FF_MT = 3,
	WAKE_PULSE,
	WAKE_LNDPRT,
	WAKE_TRANS,
	FIFO_GATE
};

enum CONTROL_REG4_bits{
	INT_EN_DRDY,
	INT_EN_FF_MT = 2,
	INT_EN_PULSE,
	INT_EN_LNDPRT,
	INT_EN_TRANS,
	INT_EN_FIFO,
	INT_EN_ASLP	
};


enum CONTROL_REG5_bits{
	INT_CFG_DRDY,
	INT_CFG_FF_MT = 2,
	INT_CGF_PULSE,
	INT_CFG_LNDPRT,
	INT_TRANS,
	INT_FIFO,
	INT_ASLP	
};

void MMA8652_Init(uint8_t range);
uint8_t MMA8652_readRegister(uint8_t offset);
void MMA8652_writeRegister(uint8_t offset, uint8_t dataToWrite);
uint8_t MMA8652_getID(void);
void MMA8652_printID(void);
void MMA8652_readAcc(int16_t *dataOut);
void MMA8652_readAccG(float *gAxis);

#endif /* MMA8652FC_H_ */