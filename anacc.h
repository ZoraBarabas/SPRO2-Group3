/*
 * anacc.h
 *
 * Created: 17/05/2021 12:16:45
 *  Author: Zora
 */ 


#ifndef ANACC_H_
#define ANACC_H_

#define F_CPU 16000000UL

#include <util/delay.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include "avr/interrupt.h"
#include "usart.h"

#define ADC_PIN0 0 //xout
#define ADC_PIN1 1 //yout
#define ADC_PIN2 2 //zout
#define timeinterval 0.0512

//fcts
uint16_t adc_read(uint8_t adc_channel);
void Calibrate();
void Compare (int, int, int);
float calculate_acceleration(uint8_t);
void calculate_all(void);
void no_movement_check(void);
void roll_and_pitch(void);

//variables
typedef struct{
	volatile float accx[2], accy[2], velx[2], vely[2], posx[2], posy[2];
} linear_values;

typedef struct{
	volatile double roll, pitch;
} angles;


#endif /* ANACC_H_ */