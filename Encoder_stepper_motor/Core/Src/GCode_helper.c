/*
 * GCode_helper.c
 *
 *  Created on: Jan 28, 2023
 *      Author: Ajith Pinninti
 */

#include "GCode_helper.h"


void Setup_Absolute_position(char *tokens[] ){

	if((*tokens[1]) == 'Z'){
	// Z directional motor
//	char *p;
	float steps =strtod(tokens[1]+1,NULL);  // strtol(tokens[1]+1, &p, 10);  //number start from next position of string

	/* Z motor Positioning */
//	_targetPos = __HAL_TIM_GET_COUNTER(&htim2);
	_currentPos = __HAL_TIM_GET_COUNTER(&htim2);
	int32_t total_steps = (int32_t) (steps * steps_per_millimeters);
	moveTo(total_steps);



	if(*tokens[2] == 'F'){
	/* F speed controller */
	float speed = strtod(tokens[2]+1,NULL);
	speed = speed * steps_per_millimeters;
	/*Fan Speed controlling */
	setMaxSpeed(speed);

	}
	return;
	}


}


void Setup_Relative_position(char *tokens[] ){

	if((*tokens[1]) == 'Z'){
	// Z directional motor
//	char *p;
	float steps =strtod(tokens[1]+1,NULL);  // strtol(tokens[1]+1, &p, 10);  //number start from next position of string

	/* Z motor Positioning */
	int32_t total_steps = (int32_t) (steps * steps_per_millimeters);
	move(total_steps);

	if(*tokens[2] == 'F'){
	/* F speed controller */
	float speed = strtod(tokens[2]+1,NULL);
	speed = speed * steps_per_millimeters;

	/*Fan Speed controlling */
	setMaxSpeed(speed);

	}
}
	return;
}

void Homing_motor(char *tokens[] ){
	setMaxSpeed(2*home_speed); //MOVING IN ccw
	setSpeed(-1*home_speed); //MOVING IN ccw


	while(!HOMED){
	runSpeed();
	}
	setCurrentPosition(0);
	return;
}

void Homing_completion(void){
	//resetting position parameters
	setCurrentPosition(0);
	stop();
	//Resetting the encoder value to zero
	__HAL_TIM_SET_COUNTER(&htim2,0);
	return;
}

void Run_Motor(void){
	runToPosition();
	return;
}
