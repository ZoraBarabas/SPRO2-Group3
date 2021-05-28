/******************************************************************
 * Compasslib.c
 *
 * Created: 14/03/2021 16:47:43
 *  Author: Henrik Frederiksen
 *	Exam number: 493947
 *****************************************************************/
#include "Compasslib.h"

 #define F_CPU 16000000UL

int _compass2_error=0;
vectorvar _compass_calibration_values;

/*****************************************************************/
// Function: compasshead
// Compass main function, reads magnetometer values and calculates
// heading.
// Parameters:
//	float pitch: calculated pitch from accelerometer
//	float roll: calculated roll from accelerometer
// Returns: Averaged heading in degrees
/****************************************************************/
float compasshead(float pitch, float roll)
{
	//used variables
	vectorvar magraw, magadjusted;
	int heading;
	
	//error handling incase of unsuccesful initialization
	if(_compass2_error==COMPASS2_READ_ERROR)
	{
		//printf("Exiting application");
		//program stops
		exit(EXIT_FAILURE);
	}
	while(1)
	 {
		 //get values from compass:
		compass2_setMode(COMPASS2_MODE_SINGL_MEASUR);
		_delay_ms(10); //very necessary delay no touchy, 
		//even just little less delay means total failure
		//Opens communication to the compass and reads control register 1
		i2c_write_reg(COMPASS2ADDR, COMPASS2_REG_CNTRL1, COMPASS2_MODE_FUSE_ROM );
		magraw.x=(float)compass2_getAxisData(COMPASS2_X_AXIS);
		if(_compass2_error==COMPASS2_READ_ERROR)
		{
			continue;
		}
		magraw.y=(float)compass2_getAxisData(COMPASS2_Y_AXIS);
		if(_compass2_error==COMPASS2_READ_ERROR)
		{
			continue;
		}
		magraw.z=(float)compass2_getAxisData(COMPASS2_Z_AXIS);
		if(_compass2_error==COMPASS2_READ_ERROR)
		{
			continue;
		}
		
		//Treat values from compass
		magadjusted=compass_adjust_reading(&magraw, pitch, roll);

		//calculate heading
		heading = atan2(magadjusted.y,magadjusted.x) * 180 / M_PI;

		if(heading<0)
		{
			heading+=360;
		}
		compassoutput(heading);
		return heading + 180;
	 }
	 
}

/*****************************************************************/
// Function: compassoutput
// Turns on appropriate LED according to heading
// Parameters:
//	float heading: calculated heading (0->360°) from compass
// Returns: -
/****************************************************************/
void compassoutput(float heading)
{
		if(heading>=338 || heading<=23)
		{
			PORTB = 0x02; //PB1 north
			PORTD = 0x0B; //turn off portd leds
		}
		else if (heading>=23 && heading<=68)
		{
			PORTD = ((1<<PORTD7) & ~(1<<PORTD6) & ~(1<<PORTD5)) | (1<<PORTD3);//make sure button not nullified
			PORTB = 0x00; //portb leds off
		}
		else if (heading>=68 && heading<=113)
		{
			PORTD = (~(1<<PORTD7) & ~(1<<PORTD6) & (1<<PORTD5)) | (1<<PORTD3);
			PORTB = 0x00;
		}
		else if (heading>=113 && heading<=158)
		{
			
			PORTD = (~(1<<PORTD7) & (1<<PORTD6) & ~(1<<PORTD5)) | (1<<PORTD3);
			PORTB = 0x00;
		}
		else if (heading>=158 && heading<=203)
		{
			PORTB = 0x01; //PB0
			PORTD = 0x0B;
		}
		else if (heading>=203 && heading<=248)
		{
			PORTB = 0x04; //PB2
			PORTD = 0x0B;
		}
			
		else if (heading>=248 && heading<=293)
		{
			PORTB = 0x10; //PB4
			PORTD = 0x0B;
		}
		else if (heading>=293 && heading<=338)
		{
			PORTB = 0x08; //PB3
			PORTD = 0x0B;
		}
}

/*****************************************************************/
//fetches data from compass according to desired axis 
// Function: compass2_getAxisData
// Parameters:
// uint8_t axis 
// Valid values:
//	COMPASS2_X_AXIS: Returns x values readings.
//	COMPASS2_Y_AXIS: Returns y values readings.
//	COMPASS2_Z_AXIS: Returns z values readings.
// Return reading.
/*****************************************************************/
int16_t compass2_getAxisData( uint8_t axis )
{
	uint8_t lsb  = 0;
	uint8_t msb  = 0;
	int16_t axis_data;

	//Depending on parameter axis, reads the contents of the register.
	if ( axis == COMPASS2_X_AXIS )
	{
		lsb = i2c_read_reg(COMPASS2ADDR, COMPASS2_REG_AXIS_X_LOW );
		msb = i2c_read_reg(COMPASS2ADDR, COMPASS2_REG_AXIS_X_HIGH );
	}
	
	else if (axis == COMPASS2_Y_AXIS)
	{
		lsb = i2c_read_reg(COMPASS2ADDR,COMPASS2_REG_AXIS_Y_LOW );
		msb = i2c_read_reg(COMPASS2ADDR,COMPASS2_REG_AXIS_Y_HIGH );
	}
	else
	{
		lsb = i2c_read_reg(COMPASS2ADDR,COMPASS2_REG_AXIS_Z_LOW );
		msb = i2c_read_reg(COMPASS2ADDR,COMPASS2_REG_AXIS_Z_HIGH );
	}
	axis_data = msb;
	axis_data <<= 8;
	axis_data |= lsb;
	return  axis_data;
}

/*****************************************************************/
// Function: compass2_setMode
// Parameters:
// uint8_t mode
// Valid values:
//	COMPASS2_MODE_SINGL_MEASUR: sets compass to only do 1 measurement
// COMPASS2_SET_RESOLUTION_16bit: sets output resolution to 16bit
// Returns: -
/****************************************************************/

void compass2_setMode( uint8_t mode )
{
	
	static uint16_t _compassres = 1;
	_compassres &= 0xF0;
	_compassres |= mode;
	//sets compass to desired mode, though only single measurement mode is used
	i2c_write_reg(COMPASS2ADDR, COMPASS2_REG_CNTRL1, _compassres );
}


/*****************************************************************/
// Function: getcompassadj
// Gets the magnetometers own calibration values
// Parameters: -
// Returns: -
/****************************************************************/
void getcompassadj (void)
{
	//Set compass to read mode from control register 1
	i2c_write_reg(COMPASS2ADDR, COMPASS2_REG_CNTRL1, COMPASS2_MODE_FUSE_ROM );
	//Read each axis of the calibration values, but only proceed if all succeed.
	//if fails, set error
	_compass_calibration_values.x=i2c_read_reg(COMPASS2ADDR, COMPASS2_REG_X_AXIS_SENS );
	if(_compass2_error!=COMPASS2_READ_ERROR)
	{
		_compass_calibration_values.y=i2c_read_reg(COMPASS2ADDR, COMPASS2_REG_Y_AXIS_SENS );
		if(_compass2_error!=COMPASS2_READ_ERROR)
		{
			_compass_calibration_values.z=i2c_read_reg(COMPASS2ADDR, COMPASS2_REG_Z_AXIS_SENS );
		}
	}
	i2c_write_reg(COMPASS2ADDR,COMPASS2_REG_CNTRL1, COMPASS2_MODE_POWER_DOWN );
}

/*****************************************************************/
// Function: compass2_init
// Starts compass and sets desired settings for measuring
// Parameters: -
// Returns: -
/****************************************************************/
void compass2_init (void)
{
	//Open communication to the compass and set modes
	i2c_start(COMPASS2ADDR);
	compass2_setMode(COMPASS2_MODE_SINGL_MEASUR | COMPASS2_SET_RESOLUTION_16bit);
	//get the calibration values
	getcompassadj();
}

/*****************************************************************/
// Function: compass_adjust_reading
// finds minimum and maximum values for all incoming readings,
// uses these and the calibration values to adjust measurements.
// compensates for pitch and roll.
// Parameters: 
//	vectorvar *magraw: the raw compass measurements
//	float pitch: calculated pitch from accelerometer readings
//	float roll: calculated roll from accelerometer readings
// Returns:
//	vectorvar struct with calibrated and compensated values for	 
//	compass x and y direction.
/****************************************************************/
vectorvar compass_adjust_reading (vectorvar *magraw, float pitch, float roll)
{
	static float xmax=0, xmin=0, xcal=0, ymax=0, ymin=0, ycal=0, zmax=0, zmin=0, zcal=0;
	vectorvar magadjusted;
	//Check if the new measurements are greater or smaller than all previous, if so save them
	if(magraw->x<xmin)
	{
		xmin=magraw->x;
	}
	if(magraw->x>xmax)
	{
		xmax=magraw->x;
	}
	if(magraw->y<ymin)
	{
		ymin=magraw->y;
	}
	if(magraw->y>ymax)
	{
		ymax=magraw->y;
	}
	if(magraw->z<zmin)
	{
		zmin=magraw->z;
	}
	if(magraw->z>zmax)
	{
		zmax=magraw->z;
	}
	//Take the average of minimum and maximum for each axis
	xcal=(xmax+xmin)/2;
	ycal=(ymax+ymin)/2;
	zcal=(zmax+zmin)/2;
	//As compass on startup can have a seemingly random maximum and minimum on all axis, values are centered on 0 by subtracting average
	//Values are also adjusted with the calibration values
	magraw->x=compass2_adjval_calc(magraw->x, _compass_calibration_values.x)-xcal;
	magraw->y=compass2_adjval_calc(magraw->y, _compass_calibration_values.y)-ycal;
	magraw->z=compass2_adjval_calc(magraw->z, _compass_calibration_values.z)-zcal; 
	//Compass values are compensated for pitch and roll
	magadjusted.y=magraw->y*cos(pitch)+magraw->x*sin(roll)*sin(pitch)-magraw->z*cos(roll)*sin(pitch);
	magadjusted.x=magraw->x*cos(roll)+magraw->y*sin(roll)*sin(pitch)-magraw->z*cos(pitch)*sin(roll);

	return magadjusted;
}

/*****************************************************************/
// Function: compass2_adjval_calc
// Adjusts the 3 axis compass readings with the calibration values
// Parameters:
//	float magraw: The measured compass value for one axis (x, y, z)
//	int compass_calibration_value: the calibration values (x, y, z)
// Returns:
//	float: the adjusted compass measurement
/****************************************************************/
float compass2_adjval_calc(float magraw, int compass_calibration_value)
{
	float asa2;
	//converts calibration value to float
	asa2=(float)compass_calibration_value;
	//uses the factory given calibration formula
	return (((magraw)*(asa2+128))/256.0);
}

/*****************************************************************/
// Function: i2c22_readNak
// Reads 1 byte via I2C, with built-in break.
// Parameters: -
// Returns: unsigned char
/****************************************************************/
unsigned char i2c22_readNak(void)
{
	int counter=0;
	_compass2_error=0;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)))
	{
		counter++;
		if (counter==1000)
		{
			_compass2_error=COMPASS2_READ_ERROR;
			_delay_ms(1);
			break;
		}
	}
	
	return TWDR;

}

/*****************************************************************/
// Function: i2c_write_reg
// Writes to a slave over I2C, interrupt proof.
// Parameters:
//	char device: I2C address of slave
//	char reg: Target register of slave
//	char data: Data to send to register
// Returns: -
/****************************************************************/
void i2c_write_reg(char device, char reg, char data)
{
	cli();
	i2c_start(device + I2C_WRITE);
	i2c_write(reg);
	i2c_write(data);
	i2c_stop();
	sei();
}

/*****************************************************************/
// Function: i2c_write_reg
// Read from a slave over I2C, interrupt proof.
// Parameters:
//	char device: I2C address of slave
//	char reg: Target register of slave
// Returns: char
/****************************************************************/
char i2c_read_reg(char device, char reg)
{
	char data;
	cli();
	i2c_start(device + I2C_WRITE);
	i2c_write(reg);
	i2c_start(device + I2C_READ);
	data=i2c22_readNak();
	i2c_stop();
	sei();
	return data;
}

