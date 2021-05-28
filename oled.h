#ifndef OLED_H
#define OLED_H

void oled_init(void);
void display(void);
void clear(void);
void fill(void);
void drawPixel(int, int);
void drawFromNumber(int);
void drawFromList(int []);
void drawFromXYList(int [],int,int);
void oledPrintString(int,int,int, char []);
void oledPrintFloat(int,int,int,float);

#endif