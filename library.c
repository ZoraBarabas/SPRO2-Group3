/*
 * anacc.c
 *
 * Created: 17/05/2021 12:16:10
 * Author : Zora
 */ 

#include <avr/io.h>
#include "anacc.h"
#include "oled.h"
int xRawMin = 330;
int xRawMax = 330;

int yRawMin = 330;
int yRawMax = 330;

int zRawMin = 330;
int zRawMax = 330;
int sampleSize = 10;

linear_values linear;
angles tilt;

float no_move_x;
float no_move_y;

char turn[] = {'T', 'U', 'R', 'N'};
char dvc[] = {'D', 'E', 'V', 'I', 'C', 'E'};

uint16_t adc_read (uint8_t adc_channel){
	ADMUX &= 0xf0; // clear any previously used channel, but keep internal reference
	ADMUX |= adc_channel; // set the desired channel
	//start a conversion
	ADCSRA |= (1<<ADSC);
	// now wait for the conversion to complete
	while ( (ADCSRA & (1<<ADSC)) );
	// now we have the result, so we return it to the calling function as a 16 bit unsigned int
	return ADC;
}

 void Compare(int xRaw, int yRaw, int zRaw){
	
	 if (xRaw < xRawMin)
	 {
		 xRawMin = xRaw;
	 }
	 if (xRaw > xRawMax)
	 {
		 xRawMax = xRaw;
	 }
	 
	 if (yRaw < yRawMin)
	 {
		 yRawMin = yRaw;
	 }
	 if (yRaw > yRawMax)
	 {
		 yRawMax = yRaw;
	 }
	 
	 if (zRaw < zRawMin)
	 {
		 zRawMin = zRaw;
	 }
	 if (zRaw > zRawMax)
	 {
		 zRawMax = zRaw;
	 }
 }

void Calibrate(){
	//calibrate
	int count=0;
	
	do{
		//printf("Calibrate\n");
		_delay_ms(100);
		int xRaw =0;
		int yRaw =0;
		int zRaw =0;
		
		for(int i =0; i<sampleSize;i++){
			xRaw += adc_read(ADC_PIN0);
		}
		xRaw=xRaw/sampleSize;
		for(int i =0; i<sampleSize;i++){
			yRaw += adc_read(ADC_PIN1);
		}
		yRaw=yRaw/sampleSize;
		for(int i =0; i<sampleSize;i++){
			zRaw += adc_read(ADC_PIN2);
		}
		zRaw=zRaw/sampleSize;
		Compare(xRaw, yRaw, zRaw);
		clear();
		oledPrintString(1, 4, 4, turn);
		oledPrintString(2, 3, 6, dvc);
		display();
		PORTD ^= (PORTD) | (1<<PORTD4);
		count++;
		_delay_ms(3000);
	}while(count<6);
	_delay_ms(1000);
	
}

float calculate_acceleration(uint8_t adc_channel){
	uint16_t adc_res;
	float acc=0;
	ADMUX &= 0xf0; // clear any previously used channel, but keep internal reference
	ADMUX |= adc_channel; // set the desired channel
	//start a conversion
	ADCSRA |= (1<<ADSC);
	// now wait for the conversion to complete
	while ( (ADCSRA & (1<<ADSC)) );
	adc_res = adc_read(adc_channel);
	if (adc_channel == ADC_PIN0) {//(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
		acc = (adc_res - xRawMin);
		acc = (acc * 2000 / (xRawMax - xRawMin) - 1000)/1000;
	}
	else if (adc_channel == ADC_PIN1) {
		acc = (adc_res - yRawMin);
		acc = (acc * 2000 / (yRawMax - yRawMin) - 1000)/1000;
	}
	else if (adc_channel == ADC_PIN2) {
		acc = (adc_res - zRawMin);
		acc = (acc * 2000 / (zRawMax - zRawMin) - 1000)/1000;
	}
	return acc;
}

void no_movement_check(){ //in m/s2
	unsigned char countx=0, county=0;
	
	linear.accx[1] = calculate_acceleration(ADC_PIN0)*9.8;
	linear.accy[1] = calculate_acceleration(ADC_PIN1)*9.8;
	
	for(int i = 0; i<3; i++){
		if ((linear.accx[1] <= no_move_x)&&(linear.accx[1] >= no_move_x*(-1))) //we count the number of acceleration samples that equals zero
		countx++;
		else countx = 0;
	}

	if (countx >= 2) //if this number exceeds 2, we can assume that velocity is zero
	{
		linear.velx[1] = 0;
		linear.velx[0] = 0;
		linear.accx[0] = no_move_x;//it cannot get lower then the threshold
	}
	//y axis
	for(int i = 0; i<3; i++){
		if ((linear.accy[1] <= no_move_y)&&(linear.accy[1] >= no_move_y*(-1))) //we count the number of acceleration samples that equals zero
		county++;
		else county = 0;
	}

	if (county >= 2) //if this number exceeds 25, we can assume that velocity is zero
	{
		linear.vely[1] = 0;
		linear.vely[0] = 0;
		linear.accy[0] = no_move_y;
	}

}

void calculate_all(){ //in m/s2
	unsigned char count=0;
	
	do{
		linear.accx[1] = linear.accx[1] + calculate_acceleration(ADC_PIN0)*9.8;
		linear.accy[1] = linear.accy[1] + calculate_acceleration(ADC_PIN1)*9.8;
		count++;
	}while(count!=64); //rolling avg for noise reduction
	
	linear.accx[1] = (linear.accx[1]/64);
	linear.accy[1] = (linear.accy[1]/64);
	
	
	//x axis
	if ((linear.accx[1] <= no_move_x) && (linear.accx[1] >= no_move_x*(-1))) {} //Discrimination window applied to the X axis acceleration variable
	else{
		linear.velx[1] = linear.accx[0] * timeinterval + (linear.accx[1] - linear.accx[0]) * timeinterval / 2; //first integration
		
		if ((linear.velx[1]<0)&&(linear.velx[0]<0)){			if ((linear.velx[1] * (-1)) < (linear.velx[0] * (-1))) linear.posx[1] = linear.posx[0] + linear.velx[0] * (-1) * timeinterval + ((linear.velx[1] * (-1) - linear.velx[0] * (-1)) * timeinterval/2)*(-1);			if ((linear.velx[1] * (-1)) >= (linear.velx[0] * (-1))) linear.posx[1] = linear.posx[0] + linear.velx[0] * (-1) * timeinterval + ((linear.velx[1] * (-1) - linear.velx[0] * (-1))* timeinterval /2);		}		if ((linear.velx[1]<0)&&(linear.velx[0]>=0)){			if ((linear.velx[1] * (-1)) < (linear.velx[0] * (-1))) linear.posx[1] = linear.posx[0] + linear.velx[0] * timeinterval + ((linear.velx[1] * (-1) - linear.velx[0]) * timeinterval/2)*(-1);			if ((linear.velx[1] * (-1)) >= (linear.velx[0] * (-1))) linear.posx[1] = linear.posx[0] + linear.velx[0] * timeinterval + ((linear.velx[1] * (-1) - linear.velx[0]) * timeinterval/2);		}
		if ((linear.velx[1]>=0)&&(linear.velx[0]<0)){			if ((linear.velx[1]) < (linear.velx[0] * (-1))) linear.posx[1] = linear.posx[0] + linear.velx[0] * (-1) * timeinterval + ((linear.velx[1] - linear.velx[0] * (-1)) * timeinterval/2)*(-1);			if ((linear.velx[1]) >= (linear.velx[0] * (-1))) linear.posx[1] = linear.posx[0] + linear.velx[0] * (-1) * timeinterval + ((linear.velx[1] - linear.velx[0] * (-1)) * timeinterval/2);		}
		if ((linear.velx[1]>=0)&&(linear.velx[0]>=0)){			if ((linear.velx[1]) < (linear.velx[0])) linear.posx[1] = linear.posx[0] + linear.velx[0] * timeinterval + ((linear.velx[1] - linear.velx[0]) * timeinterval/2)*(-1);			if ((linear.velx[1]) >= (linear.velx[0])) linear.posx[1] = linear.posx[0] + linear.velx[0] * timeinterval + ((linear.velx[1] - linear.velx[0]) * timeinterval/2);		}
	}
	
	//y axis
	if ((linear.accy[1] <= no_move_y) && (linear.accy[1] >= no_move_y*(-1))) {}//Discrimination window appliedto the Y axis acceleration variable
	else{
		linear.vely[1] = linear.accy[0] * timeinterval + (linear.accy[1] - linear.accy[0])/2; //first integration
		
		if ((linear.vely[1]<0)&&(linear.vely[0]<0)){			if ((linear.vely[1] * (-1)) < (linear.vely[0] * (-1))) linear.posy[1] = linear.posy[0] + linear.vely[0] * (-1) * timeinterval + ((linear.vely[1] * (-1) - linear.vely[0] * (-1)) * timeinterval/2)*(-1);			if ((linear.vely[1] * (-1)) >= (linear.vely[0] * (-1))) linear.posy[1] = linear.posy[0] + linear.vely[0] * (-1) * timeinterval + ((linear.vely[1] * (-1) - linear.vely[0] * (-1)) * timeinterval/2);		}		if ((linear.vely[1]<0)&&(linear.vely[0]>=0)){			if ((linear.vely[1] * (-1)) < (linear.vely[0] * (-1))) linear.posy[1] = linear.posy[0] + linear.vely[0] * timeinterval + ((linear.vely[1] * (-1) - linear.vely[0]) * timeinterval/2)*(-1);			if ((linear.vely[1] * (-1)) >= (linear.vely[0] * (-1))) linear.posy[1] = linear.posy[0] + linear.vely[0] * timeinterval + ((linear.vely[1] * (-1) - linear.vely[0]) * timeinterval/2);		}
		if ((linear.vely[1]>=0)&&(linear.vely[0]<0)){			if ((linear.vely[1]) < (linear.vely[0] * (-1))) linear.posy[1] = linear.posy[0] + linear.vely[0] * (-1) * timeinterval + ((linear.vely[1] - linear.vely[0] * (-1)) * timeinterval/2)*(-1);			if ((linear.vely[1]) >= (linear.vely[0] * (-1))) linear.posy[1] = linear.posy[0] + linear.vely[0] * (-1) * timeinterval + ((linear.vely[1] - linear.vely[0] * (-1)) * timeinterval/2);		}
		if ((linear.vely[1]>=0)&&(linear.vely[0]>=0)){			if ((linear.vely[1]) < (linear.vely[0])) linear.posy[1] = linear.posy[0] + linear.vely[0] * timeinterval + ((linear.vely[1] - linear.vely[0]) * timeinterval/2)*(-1);			if ((linear.vely[1]) >= (linear.vely[0])) linear.posy[1] = linear.posy[0] + linear.vely[0] * timeinterval + ((linear.vely[1] - linear.vely[0]) * timeinterval/2);		}
	}
	
	linear.accx[0] = linear.accx[1]; //new becomes previous
	linear.velx[0] = linear.velx[1];
	linear.posx[0] = linear.posx[1];
	
	linear.accy[0] = linear.accy[1]; //new becomes previous
	linear.vely[0] = linear.vely[1];
	linear.posy[0] = linear.posy[1];
	
}

 void roll_and_pitch(){
	 float accx, accy, accz;
	 accx = calculate_acceleration(ADC_PIN0);
	 accy = calculate_acceleration(ADC_PIN1);
	 accz = calculate_acceleration(ADC_PIN2);
	 
	 tilt.roll = atan2(accy, accz)*57.3;//atan2 gives rad
	 tilt.pitch = atan2((- accx) , sqrt(accy * accy + accz * accz))*57.3;
	 
	 if((tilt.roll <= -10) || (tilt.roll >= 10) || (tilt.pitch <= -10) || (tilt.pitch >= 10)) PORTD ^= (PORTD) | (1<<PORTD4);
	 else PORTD = PORTD & ~(1<<PORTD4);
 }