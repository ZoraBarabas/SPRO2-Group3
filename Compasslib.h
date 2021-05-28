/******************************************************************
 * Compasslib.h
 *
 * Created: 14/03/2021 16:47:43
 *  Author: Henrik Frederiksen
 *	Exam number: 493947
 *****************************************************************/


#ifndef COMPASSLIB_H_
#define COMPASSLIB_H_

//CPU clock frequency
 #define F_CPU 16000000UL
 
 #include <stdio.h>
 #include "stdint.h"
 #include <stdlib.h>
 #include <avr/io.h>
 #include <util/delay.h>
 #include "i2cmaster.h"
 #include "usart.h"
 #include "lcd.h"
 #include "std.h"

 #include "avr/interrupt.h"
 
 //Type definitions
 typedef struct
 {
	 double x;
	 double y;
	 double z;
 } vectorvar;

 
#define COMPASS2_RETVAL  uint8_t

//Compass definitions
#define COMPASS2_OK           0x00
#define COMPASS2_INIT_ERROR   0xFF
#define COMPASS2_REG_DEVICE_ID         0x00 
#define COMPASS2_REG_INFORMATION       0x01 
#define COMPASS2_REG_STATUS_1          0x02 
#define COMPASS2_REG_AXIS_X_LOW        0x03 
#define COMPASS2_REG_AXIS_X_HIGH       0x04 
#define COMPASS2_REG_AXIS_Y_LOW        0x05 
#define COMPASS2_REG_AXIS_Y_HIGH       0x06 
#define COMPASS2_REG_AXIS_Z_LOW        0x07 
#define COMPASS2_REG_AXIS_Z_HIGH       0x08 
#define COMPASS2_REG_STATUS_2          0x09 
#define COMPASS2_REG_CNTRL1            0x0A 
#define COMPASS2_REG_CNTRL2            0x0B 
#define COMPASS2_REG_SELF_TEST         0x0C 
#define COMPASS2_REG_TEST_1            0x0D 
#define COMPASS2_REG_TEST_2            0x0E 
#define COMPASS2_REG_I2C_DISABLE       0x0F 
#define COMPASS2_REG_X_AXIS_SENS       0x10 
#define COMPASS2_REG_Y_AXIS_SENS       0x11 
#define COMPASS2_REG_Z_AXIS_SENS       0x12 

#define COMPASS2_MODE_POWER_DOWN       0x00
#define COMPASS2_MODE_SINGL_MEASUR     0x01
#define COMPASS2_MODE_CONT_MEAS_1      0x02
#define COMPASS2_MODE_CONT_MEAS_2      0x06
#define COMPASS2_MODE_EXT_TRIG         0x04
#define COMPASS2_MODE_SELF_TEST        0x08
#define COMPASS2_MODE_FUSE_ROM         0x0F

#define COMPASS2_SET_RESOLUTION_14bit  0x00
#define COMPASS2_SET_RESOLUTION_16bit  0x10

#define COMPASS2_MODE_SPI              0x00
#define COMPASS2_MODE_I2C              0x01

#define COMPASS2_X_AXIS                0x00
#define COMPASS2_Y_AXIS                0x01
#define COMPASS2_Z_AXIS                0x02

#define COMPASS2_READ_ERROR			0x01

 #define COMPASS2ADDR 0x1E //1E 
 
 //function headings
 void i2c_write_reg(char, char, char);
 char i2c_read_reg(char device, char reg);
 void get_data_accel(int *x, int *y, int *z);
 int16_t compass2_getAxisData( uint8_t axis );
 void compass2_setMode( uint8_t mode );
 unsigned char i2c22_readNak(void);
 void getcompassadj (void);
 void compass2_init (void);
 vectorvar compass_adjust_reading (vectorvar *magraw, float pitch, float roll);
float compass2_adjval_calc(float magraw, int compass_calibration_value);
float compasshead(float pitch, float roll);
void compassoutput(float heading);
#endif /* COMPASSLIB_H_ */