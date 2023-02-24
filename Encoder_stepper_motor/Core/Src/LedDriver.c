/*
 * LedDriver.c
 *
 *  Created on: 11-Nov-2022
 *      Author: Ajith Pinninti
 */
#include "main.h"
#include "LedDriver.h"
#include <math.h>

extern SPI_HandleTypeDef hspi1;
uint8_t count = 0;

//brightness values

int ch1_br = 65535/4; //ch1 brightness
int ch2_br = 65535/4; //ch2 brightness
int ch3_br = 65535/4; //ch3 brightness


char x = '0'; //channel number

//sending bits are arr1 and arr2
int arr[16];
int arr1[8];
int arr2[8];

// 3 channel buffers for controlling
char ch1_buff1;
char ch1_buff2;
char ch2_buff1;
char ch2_buff2;
char ch3_buff1;
char ch3_buff2;

uint8_t stop_led = 0;

/*
 * Convert binary values contaning arry to decimal values
 * Params:
 * y_1 =
 */
void dectoint(char *y_1, char *y_2)
{
	*y_1 = ((arr1[0]*pow(2,7))+(arr1[1]*pow(2,6))+(arr1[2]*pow(2,5))+(arr1[3]*pow(2,4))+(arr1[4]*pow(2,3))+(arr1[5]*pow(2,2))+(arr1[6]*pow(2,1))+(arr1[7]*pow(2,0)));

	*y_2 = ((arr2[0]*pow(2,7))+(arr2[1]*pow(2,6))+(arr2[2]*pow(2,5))+(arr2[3]*pow(2,4))+(arr2[4]*pow(2,3))+(arr2[5]*pow(2,2))+(arr2[6]*pow(2,1))+(arr2[7]*pow(2,0)));

}

/*
 * Split the values in the 16bit value to two 8 bit values and stores that value as decimal in two variables
 * param:
 * y1 = empty char which going to carry left most 8 bits equivalent decimal value after the function execution.
 * y2 = empty char which going to carry right most 8 bits equivalent decimal value after the function execution.
 */

void split(char *y1, char *y2)
{
	int k=0;
	int l=0;
	for(int i = 0; i<16; i++)
	{
		if(i>=0 && i<=7)
		{

			arr1[k] = arr[i];
			k++;
		}

		else if(i>=8 && i <=15)
		{

			arr2[l] = arr[i];
			l++;
		}
	}
	dectoint(y1,y2);// stores the equivalent decimal of binary number in arr1 and arr2 into y1 and y2
}
/*
 * convert  decimal value to binary value and split that 16 bit value to two 8 bit values and stores in other variables in decimal format
 * param:
 * y = decimal that need to convert to binary
 * y1 = stores the left most 8 bits in decimal form from 16bit binary number
 * y2 = stores the right most 8 bits in decimal form from 16bit binary number
 *
 */
void dectobin(int y, char *y1, char *y2)
{
	int i = 0;
	int j = 0;

	int binaryNum[16]={0};
	while( y > 0)
	{
		binaryNum[i] = y % 2;
		y = y/2;
		i++;
	}

	int k=0;
	for(j= 15; j >= 0; j--)
	{
		arr[k] = binaryNum[j];
		k++;
	}
	split(y1,y2);
}
/*
 * Turn On the LED channel with desired brightness that we provided as parameter
 * param:
 * channel = char which equal to binary between 00110000 to 00111111
 * y1 : 8 bit value that splits from left part of 16bit brightness value
 * y2 :  8 bit value that splits from right part of 16bit brightness value
 */
void send_function( char channel , char *y_1 , char *y_2)
{

		HAL_GPIO_WritePin(SPI_Selection_GPIO_Port , SPI_Selection_Pin, GPIO_PIN_RESET);
		x = channel;

		HAL_SPI_Transmit(&hspi1,(uint8_t*) &x, sizeof(x), 100);

	    HAL_SPI_Transmit(&hspi1,(uint8_t*) y_1, sizeof(*y_1), 100);

	    HAL_SPI_Transmit(&hspi1,(uint8_t*) y_2, sizeof(*y_2), 100);


	    HAL_GPIO_WritePin(SPI_Selection_GPIO_Port , SPI_Selection_Pin, GPIO_PIN_SET);
}

/*
 * Turn Off the LED channel that we provided as parameter
 * param:
 * channel = char which equal to binary between 00110000 to 00111111
 */
void stop_function( char channel)
{

		HAL_GPIO_WritePin(SPI_Selection_GPIO_Port , SPI_Selection_Pin, GPIO_PIN_RESET);
		x = channel;

		HAL_SPI_Transmit(&hspi1,(uint8_t*) &x, sizeof(x), 100);

	    HAL_SPI_Transmit(&hspi1,&stop_led, sizeof(stop_led), 100);

	    HAL_SPI_Transmit(&hspi1,&stop_led, sizeof(stop_led), 100);

	    HAL_GPIO_WritePin(SPI_Selection_GPIO_Port , SPI_Selection_Pin, GPIO_PIN_SET);
}

