/*
 * group 3.c
 *
 * Created: 27/05/2021 8:35:28
 * Author : Zora
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>

#include "Compasslib.h"
#include "anacc.h"
#include "oled.h"
#include <avr/pgmspace.h>

#define ADC_PIN0 0
#define ADC_PIN1 1
#define ADC_PIN2 2
#define timeinterval 0.0512
#define DACCRES 16384
#define GRAVITY 9.8

//digital and compass stuff
long acceleration (void);

volatile float acc, vel, dis, acc1, pitch, roll;
volatile float heading, disx, disy;
float toimer;

volatile int five_sec = 0;
volatile float accx, accy, accz, avgx = 0, avgy = 0, avgz = 0, avgx_updated = 0, avgy_updated = 0, avgz_updated = 0;

//analogue stuff
linear_values linear; //all values calculated
float no_move_x, no_move_y; //threshold values
angles tilt;

//steps
volatile int steps, step_start = 0;

//display stuff
int five_or_six(float);
char avg[] = {'A','V','G'};
char x[] = {'X','='}, ax[] = {'A', 'X', '='},vx []= {'V', 'X', '='}, dx[] = {'D', 'X', '='};
char y[] = {'Y','='}, ay[] = {'A', 'Y', '='}, vy[] = {'V', 'Y', '='}, dy[] = {'D', 'Y', '='};
char z[] = {'Z','='}, az[] = {'A', 'Z', '='};
char dist[] = {'D', 'I', 'S', 'T', '='};
char disp[] = {'D', 'I', 'S', 'P', '=' };
char st[] = {'S', 'T', 'E', 'P', 'S'};
char trn[] = {'T', 'U', 'R', 'N'}, arnd[] = {'A', 'R', 'O', 'U', 'N', 'D'};


int main(void)
{
    int length, pushed = 0;
	
	DDRC=0x00; //make all PINC inputs
	DDRD=0xF7; //make all output except D3 for button
	DDRB = 0xFF; //make all PINB output for LEDs
	PORTD = 0x08; // Enable internal pull at pin D3 for button, leds off
	PORTB = 0x00; //LEDs off
	
	i2c_init();
	oled_init();
	compass2_init();
	oledPrintString(1, 4, 4, trn);
	oledPrintString(2, 3, 6, arnd);
	display();
	_delay_ms(5000);
	
	//select Vref=AVcc
	ADMUX = (1<<REFS0);
	//set prescaler to 128 and turn on ADC
	ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADEN);
	
	Calibrate();
	MMA8451_init_1();
	
	//normval normacc;
	float xcord, ycord, zcord;
	
	//calculate analog thresholds
	int sample = 0;
	do
	{
		no_move_x += calculate_acceleration(ADC_PIN0)*9.8;
		no_move_y += calculate_acceleration(ADC_PIN1)*9.8;
		sample++;
	} while (sample<5);
	if(no_move_x<0) no_move_x = no_move_x/(-5) + 0.1; //make sure its positive
	else no_move_x = no_move_x/5 + 0.1;
	if(no_move_y<0) no_move_y = no_move_y/(-5) + 0.1;
	else no_move_y = no_move_y/5 + 0.1;
	
	TCCR1B|=(1<<WGM12); //Timer mode CTC
	TIMSK1 |= (1 << OCIE1A); //enable interrupts on overflow of timer 1
	OCR1A=(F_CPU/1024)*timeinterval-1; // count to timeinterval- for more accuracy at higher frequencies,
	//division factor should be adjusted
	
	get_data_accel_1(&xcord, &ycord, &zcord); //get acceleration values
	_delay_ms(500); //init time for accel
	get_data_accel_1(&xcord, &ycord, &zcord); //get acceleration values
	xcord=((xcord)/DACCRES);
	ycord=((ycord)/DACCRES);
	zcord=((zcord)/DACCRES);
	acc1=1; //Gravity is 1G
	
	sei();
	TCCR1B|=(1<<CS10)|(1<<CS12); //prescaler 1024 & start timer
	
	get_data_accel_1(&xcord, &ycord, &zcord); //get acceleration values
	
    while (1) 
    {
		_delay_ms(300); //suitably long delay for there to be enough time for the interrupt to occur frequently enough to give accurate readings
		heading=compasshead(pitch, roll); //compass function- it requires that pitch and roll have been measured, or at least defined as 0
		
		if(PIND == (PIND & (~(1<<PIND3)))) {
			_delay_ms(200);
			pushed++;
			if(pushed == 7) pushed = 0;
		}
		
		//display
		if((pushed == 0)){
			clear();
			oledPrintString(1,2,3,ax);
			oledPrintString(2,2,3,vx);
			length = five_or_six(accx);
			oledPrintFloat(1,5,length,accx);
			length = five_or_six(linear.velx[1]);
			oledPrintFloat(2,5,length,linear.velx[1]);
			cli();
			display();
			sei();
		}
		else if(pushed == 1){
			clear();
			oledPrintString(1,2,3,ay);
			oledPrintString(2,2,3,vy);
			length = five_or_six(accy);
			oledPrintFloat(1,5,length,accy);
			length = five_or_six(linear.vely[1]);
			oledPrintFloat(2,5,length,linear.vely[1]);
			cli();
			display();
			sei();
		}
		else if(pushed == 2){
			clear();
			oledPrintString(1,3,3,az);
			oledPrintString(2,1,3,avg);
			oledPrintString(2,4,2,z);
			length = five_or_six(accz);
			oledPrintFloat(1,6,length,accz);
			length = five_or_six(avgz_updated);
			oledPrintFloat(2,6,length, avgz_updated);
			cli();
			display();
			sei();
		}		
		else if(pushed == 3){
		clear();
		oledPrintString(1,1,3,avg);
		oledPrintString(1,4,2,x);
		oledPrintString(2,1,3,avg);
		oledPrintString(2,4,2,y);
		length = five_or_six(avgx_updated);
		oledPrintFloat(1,6,length,avgx_updated);
		length = five_or_six(avgy_updated);
		oledPrintFloat(2,6,length,avgy_updated);
		cli();
		display();
		sei();
		}
		else if (pushed == 4){			
		clear();
		oledPrintString(1,3,3,dx);
		oledPrintString(2,3,3,dy);
		length = five_or_six(linear.posx[1]);
		oledPrintFloat(1,6,length,linear.posx[1]);
		length = five_or_six(linear.posy[1]);
		oledPrintFloat(2,6,length,linear.posy[1]);
		cli();
		display();
		sei();
		}	
		else if(pushed == 5){
		clear();
		oledPrintString(1,1,5,dist);
		oledPrintString(2,1,5,disp);
		length = five_or_six(dis);
		oledPrintFloat(1,6,length,dis);
		length = five_or_six(sqrt(disx*disx+disy*disy));
		oledPrintFloat(2,6,length,sqrt(disx*disx+disy*disy));
		cli();
		display();
		sei();
		}
		else if (pushed == 6){
		clear();
		oledPrintString(1,4,5,st);
		oledPrintFloat(2,3,5,steps);
		cli();
		display();
		sei();
		}
    }
}

int five_or_six(float number)
{
	//if number is less than 0 return 6 else return 5
	return (number<0)? 6 : 5;
}

ISR (TIMER1_COMPA_vect){
	
	static float a0=0, v0=0, dis0=0;
	float accchange;
	float accxnorm, accynorm, accmag;
	static int count=0;
	float xcord, ycord, zcord;

	
	get_data_accel_1(&xcord, &ycord, &zcord); //get acceleration values
	
	xcord=((xcord)/DACCRES); //convert values to unit g
	ycord=((ycord)/DACCRES);
	zcord=((zcord)/DACCRES);
	
	accx = xcord * GRAVITY;
	accy = ycord * GRAVITY;
	accz = zcord * GRAVITY;
	
	if(five_sec <= 196){
		avgx += accx;
		avgy += accy;
		avgz += accz;
	}
	
	if(five_sec == 196){
		avgx_updated = avgx/ five_sec;
		avgy_updated = avgy/ five_sec;
		avgz_updated = avgz/ five_sec;
		five_sec = 0;
		avgx = 0;
		avgy = 0;
		avgz = 0;
	}
	
	//steps
	if((xcord*xcord + ycord*ycord + zcord*zcord) >= 1.35) step_start = 1;
	if((step_start==1)&&((xcord*xcord + ycord*ycord + zcord*zcord) <= 0.9)) {steps++; step_start = 0;}	
	
	if(sqrt(xcord*xcord+ycord*ycord+zcord*zcord)<1.1) //if acceleration is less than slightly above 1G, save measurement as pitch and roll
	{
		accmag=sqrt(xcord*xcord+ycord*ycord+zcord*zcord);
		accxnorm=xcord/accmag;
		accynorm=ycord/accmag;
		pitch=asin(accxnorm);
		roll=-asin(accynorm/cos(pitch))+2*M_PI;
	}
	accchange=sqrt(xcord*xcord+ycord*ycord+zcord*zcord)-acc1; //find change in acceleration
	if (accchange<0.02) //if acceleration change is less than 0.02, assume noise
	{
		accchange=0;
	}
	if((accchange)<0.02) //if there is no acceleration for 50*0.025 Sec
	{
		count++;
		if(count>25)
		{
			accchange=0; //assume person is standing still
			a0=0;
			v0=0;
			count=0;
		}
	}
	
	acc=a0+(accchange)*GRAVITY; //convert from g to m/s^2
	vel=acc*timeinterval+v0; //get linear velocity
	v0=vel;
	dis=vel*timeinterval+dis0;
	disx=vel*timeinterval*cos(heading*M_PI/180)+disx; //sum displacement along north/south axis
	disy=vel*timeinterval*sin(heading*M_PI/180)+disy; //sum displacement along east/west axis
	
	dis0=dis;
	
	roll_and_pitch();
	if(!((tilt.roll <= -10) || (tilt.roll >= 10) || (tilt.pitch <= -10) || (tilt.pitch >= 10))){
	no_movement_check();
	calculate_all();
	}
	five_sec++;
}
