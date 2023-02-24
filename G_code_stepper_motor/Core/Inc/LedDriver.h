/*
 * LedDriver.h
 *
 *  Created on: 11-Nov-2022
 *      Author: Ajith Pinninti
 */

#ifndef INC_LEDDRIVER_H_
#define INC_LEDDRIVER_H_
//brightness values
// LEDs addresses
#define LED1 '1'
#define LED2 '2'
#define LED3 '4'
extern uint8_t count;


extern int ch1_br; //ch1 brightness
extern int ch2_br; //ch2 brightness
extern int ch3_br; //ch3 brightness


extern char x; //channel number

//sending bits are arr1 and arr2
extern int arr[16];
extern int arr1[8];
extern int arr2[8];

// 3 channel buffers for controlling
extern char ch1_buff1;
extern char ch1_buff2;
extern char ch2_buff1;
extern char ch2_buff2;
extern char ch3_buff1;
extern char ch3_buff2;

extern uint8_t stop_led;

//function definitions

void dectoint(char *y_1, char *y_2);
void split(char *y1, char *y2);
void dectobin(int y, char *y1, char *y2);
void send_function( char channel , char *y_1 , char *y_2);
void stop_function( char channel);




#endif /* INC_LEDDRIVER_H_ */
