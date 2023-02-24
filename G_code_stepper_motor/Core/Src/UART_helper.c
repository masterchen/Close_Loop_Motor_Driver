/*
 * UART_helper.c
 *
 *  Created on: Jan 29, 2023
 *      Author: Ajith Pinninti
 */

#include "UART_helper.h"

void UART_Command(char *tokens[]){

		if(strcmp(tokens[0],"G90") == 0 ){
		//Absolute positioning
		Setup_Absolute_position(tokens);

		//Run the Motors
		Run_Motor();

		//Execute Command make zero and Reset the buffer
		Exec_command = 0;
		memset(RxBuffer,0,sizeof(RxBuffer));


		//sending the ready status
		memset(sending_data,0,sizeof(sending_data));
		sprintf(sending_data,"G90Completed \n");
		HAL_UART_Transmit(&huart2,(uint8_t*)sending_data,strlen(sending_data),HAL_MAX_DELAY);

		}

		else if(strcmp(tokens[0],"G91") == 0){
		//relative Positioning
		Setup_Relative_position(tokens);

		//RUN the MOTOR
		Run_Motor();

		//Execute Command make zero and Reset the buffer
		Exec_command = 0;
		memset(RxBuffer,0,sizeof(RxBuffer));

		//sending the ready status
		memset(sending_data,0,sizeof(sending_data));
		sprintf(sending_data,"G91Completed \n");
		HAL_UART_Transmit(&huart2,(uint8_t*)sending_data,strlen(sending_data),HAL_MAX_DELAY);


		}

		else if(strcmp(tokens[0],"HOME") == 0){

		//enabling IRQ for Endstop button
		HAL_NVIC_EnableIRQ(Z_END_STOP_EXTI_IRQn);
		HOMED = false;

		//homing
		Homing_motor(tokens);


		//sending response
//		memset(sending_data,0,sizeof(sending_data));
//		sprintf(sending_data,"HOMED \n");
//		HAL_UART_Transmit(&huart2,(uint8_t*)sending_data,strlen(sending_data),HAL_MAX_DELAY);

		//Excute_command make zero and Reset the buffer
		//Exec_command = 0;
		//Exec_command made zero by endstop
		}
		else if(strcmp(tokens[0],"MOTPOS") == 0){
			//sending motor current position
			memset(sending_data,0,sizeof(sending_data));
			sprintf(sending_data,"%0.2f\n",(float) currentPosition()/steps_per_millimeters );
			HAL_UART_Transmit(&huart2,(uint8_t*)sending_data,strlen(sending_data),HAL_MAX_DELAY);

			//Execute Command make zero and Reset the buffer
			Exec_command = 0;
			memset(RxBuffer,0,sizeof(RxBuffer));
		}

		else if(strcmp(tokens[0],"ENCZERO") == 0){
			//making encoder value zero

			//Set encoder counting to zero
			__HAL_TIM_SET_COUNTER(&htim2,0);

			//sending the ready status
			memset(sending_data,0,sizeof(sending_data));
			sprintf(sending_data,"Zeroed\n");
			HAL_UART_Transmit(&huart2,(uint8_t*)sending_data,strlen(sending_data),HAL_MAX_DELAY);

			//Execute Command make zero and Reset the buffer
			Exec_command = 0;
			memset(RxBuffer,0,sizeof(RxBuffer));
		}
		else if(strcmp(tokens[0],"ENCVAL") == 0){
			//sending encoder value through UART
			memset(sending_data,0,sizeof(sending_data));
			sprintf(sending_data,"%ld\n",__HAL_TIM_GET_COUNTER(&htim2) );
			HAL_UART_Transmit(&huart2,(uint8_t*)sending_data,strlen(sending_data),HAL_MAX_DELAY);

			//Execute Command make zero and Reset the buffer
			Exec_command = 0;
			memset(RxBuffer,0,sizeof(RxBuffer));
		}
		else{

			//Execute Command make zero and Reset the buffer
			Exec_command = 0;
			memset(RxBuffer,0,sizeof(RxBuffer));
		}


}
