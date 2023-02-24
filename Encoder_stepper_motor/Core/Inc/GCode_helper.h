/*
 * GCode_helper.h
 *
 *  Created on: Jan 28, 2023
 *      Author: Ajith Pinninti
 */

#ifndef INC_GCODE_HELPER_H_
#define INC_GCODE_HELPER_H_


/*********************************** Macros definitons **********************/

#include "stepper.h"
#include "main.h"

/************************ Function Prototype ******************/

/*Setting up the absolute motor position,
 * @ params
 * tokens -> contains splitted receiver buffer
 *
 */

void Setup_Absolute_position(char *tokens[]);
/* Setting up the Relative motor position,
 * @ params
 * tokens -> contains splitted receiver buffer
 *
 */
void Setup_Relative_position(char *tokens[] );

/*
 * Running the Motor to the desired poistion
 * won't return until completion
 */

void Run_Motor(void);


/** Homing Functions **/

//moving motor in one direction in same speed
void Homing_motor(char *tokens[] );

//reset the position
void Homing_completion(void);

/*******************************************************************/

#endif /* INC_GCODE_HELPER_H_ */
