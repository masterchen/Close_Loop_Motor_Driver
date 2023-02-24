/*
 * stepper.c
 *
 *  Created on: Jan 24, 2023
 *      Author: Ajith Pinninti
 */
//#include "main.h"
#include "stepper.h"

//#define debug 1


#if debug
	uint32_t sending_var;
#endif


uint16_t counter;

GPIO_TypeDef*  step_port;
uint16_t step_pin;

GPIO_TypeDef*  dir_port;
uint16_t dir_pin;

_Bool _direction; // 1 == CW

uint32_t  _stepInterval;
uint8_t   _interface;          // 0, 1, 2, 4, 8, See MotorInterfaceType
uint8_t   _pin[4];
uint8_t   _pinInverted[4];
int32_t    _currentPos;    // Steps

/// The target position in steps. The AccelStepper library will move the
/// motor from the _currentPos to the _targetPos, taking into account the
/// max speed, acceleration and deceleration
int32_t  _targetPos;     // Steps

/// The current motos speed in steps per second
/// Positive is clockwise
float    _speed;         // Steps per second

/// The maximum permitted speed in steps per second. Must be > 0.
float          _maxSpeed;

/// The acceleration to use to accelerate or decelerate the motor in steps
/// per second per second. Must be > 0
float          _acceleration;
float          _sqrt_twoa; // Precomputed sqrt(2*_acceleration)

/// The last step time in microseconds
uint32_t  _lastStepTime;

/// The minimum allowed pulse width in microseconds
uint32_t   _minPulseWidth;

/// Is the direction pin inverted?
///bool           _dirInverted; /// Moved to _pinInverted[1]

/// Is the step pin inverted?
///bool           _stepInverted; /// Moved to _pinInverted[0]

/// Is the enable pin inverted?
_Bool           _enableInverted;

/// Enable pin for stepper driver, or 0xFF if unused.
uint8_t        _enablePin;

/// The pointer to a forward-step procedure
void (*_forward)();

/// The pointer to a backward-step procedure
void (*_backward)();

/// The step counter for speed calculations
int32_t _n;

/// Initial step size in microseconds
float _c0;

/// Last step size in microseconds
float _cn;

/// Min step size in microseconds based on maxSpeed
float _cmin; // at max speed




void moveTo(int32_t absolute)
{
	absolute  = (absolute<min_pos) ? min_pos :( (absolute<max_pos) ? absolute : max_pos );
    if (_targetPos != absolute)
    {
	_targetPos = absolute;
	computeNewSpeed();
	// compute new n?
    }
}

void move(int32_t relative)
{
    moveTo(_currentPos + relative);
}

void stepper_setup(GPIO_TypeDef*_step_port,uint16_t _step_pin, GPIO_TypeDef*  _dir_port,uint16_t _dir_pin ){


	step_port = _step_port;
	step_pin = _step_pin;
	dir_port = _dir_port;
	dir_pin = _dir_pin;


	_stepInterval = 0;
	_minPulseWidth = 1;
	_currentPos = 0;
	_targetPos = 0;
	_speed = 0.0;
	_maxSpeed = 30.0;
	_acceleration = 0.0;
	_sqrt_twoa = 1.0;
	_stepInterval = 0;
	_minPulseWidth = 1;
	_enablePin = 0xff;
	_lastStepTime = 0;

//	_enableInverted = false;




	// NEW
	_n = 0;
	_c0 = 0.0;
	_cn = 0.0;
	_cmin = 1.0;
	_direction = DIRECTION_CCW;

	int i;
	for (i = 0; i < 4; i++)
	_pinInverted[i] = 0;


	for (i = 0; i < 4; i++)
	_pinInverted[i] = 0;

}


_Bool runSpeed()
{
    // Dont do anything unless we actually have a step interval
    if (!_stepInterval)
	return false;

    uint32_t time =  __HAL_TIM_GET_COUNTER(&htim6);
    if (time  >= _stepInterval)
    {
//    	_currentPos = __HAL_TIM_GET_COUNTER(&htim2);
//	if (_direction == DIRECTION_CW)
//	{
//	    // Clockwise
//	    _currentPos += 1;
//	}
//	else
//	{
//	    // Anticlockwise
//	    _currentPos -= 1;
//	}

	step(_currentPos);
	_currentPos = __HAL_TIM_GET_COUNTER(&htim2);

	__HAL_TIM_SET_COUNTER(&htim6,0); //resetting the timer
	//_lastStepTime = time; // Caution: does not account for costs in step()

	return true;
    }
    else
    {
	return false;
    }
}

int32_t distanceToGo()
{
    return _targetPos - _currentPos;
}

int32_t targetPosition()
{
    return _targetPos;
}

int32_t currentPosition()
{
    return _currentPos;
}

void setCurrentPosition(int32_t position)
{
    _targetPos = _currentPos = position;
    _n = 0;
    _stepInterval = 0;
    _speed = 0.0;
}

uint32_t computeNewSpeed()
{
    int32_t distanceTo = distanceToGo(); // +ve is clockwise from curent location

    int32_t stepsToStop = (int32_t)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16

    if (distanceTo == 0 && stepsToStop <= 1)
    {
	// We are at the target and its time to stop
	_stepInterval = 0;
	_speed = 0.0;
	_n = 0;
	return _stepInterval;
    }

    if (distanceTo > 0)
    {
	// We are anti clockwise from the target
	// Need to go clockwise from here, maybe decelerate now
	if (_n > 0)
	{
	    // Currently accelerating, need to decel now? Or maybe going the wrong way?
	    if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
		_n = -stepsToStop; // Start deceleration
	}
	else if (_n < 0)
	{
	    // Currently decelerating, need to accel again?
	    if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
		_n = -_n; // Start accceleration
	}
    }
    else if (distanceTo < 0)
    {
	// We are clockwise from the target
	// Need to go anticlockwise from here, maybe decelerate
	if (_n > 0)
	{
	    // Currently accelerating, need to decel now? Or maybe going the wrong way?
	    if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
		_n = -stepsToStop; // Start deceleration
	}
	else if (_n < 0)
	{
	    // Currently decelerating, need to accel again?
	    if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
		_n = -_n; // Start accceleration
	}
    }

    // Need to accelerate or decelerate
    if (_n == 0)
    {
	// First step from stopped
	_cn = _c0;
	_direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    else
    {
	// Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
	_cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
	_cn = _cn >_cmin ? _cn:_cmin; // max(_cn,_cmin)
    }
    _n++;
    _stepInterval = _cn;
    _speed = 1000000.0 / _cn;
    if (_direction == DIRECTION_CCW)
	_speed = -_speed;

#if debug

    // transmit the encoder value and step interval value
//    sending_var = _speed;
    sending_var = *((uint32_t*)&_speed) ;

    HAL_UART_Transmit(&huart2, (uint8_t*)&sending_var , sizeof(sending_var), HAL_MAX_DELAY);

    sending_var = *((uint32_t*)&_currentPos) ;

    HAL_UART_Transmit(&huart2, (uint8_t*)&sending_var, sizeof(sending_var), HAL_MAX_DELAY);


#endif

#if 0
    Serial.println(_speed);
    Serial.println(_acceleration);
    Serial.println(_cn);
    Serial.println(_c0);
    Serial.println(_n);
    Serial.println(_stepInterval);
    Serial.println(distanceTo);
    Serial.println(stepsToStop);
    Serial.println("-----");
#endif
    return _stepInterval;
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if the motor is still running to the target position.
_Bool run()
{
    if (runSpeed())
	computeNewSpeed();
    return _speed != 0.0 || distanceToGo() != 0;
}


void setMaxSpeed(float speed)
{
    if (speed < 0.0)
       speed = -speed;
    if (_maxSpeed != speed)
    {
	_maxSpeed = speed;
	_cmin = 1000000.0 / speed;
	// Recompute _n from current speed and adjust speed if accelerating or cruising
	if (_n > 0)
	{
	    _n = (int32_t)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
	    computeNewSpeed();
	}
    }
}

float maxSpeed()
{
    return _maxSpeed;
}

void setAcceleration(float acceleration)
{
    if (acceleration == 0.0)
	return;
    if (acceleration < 0.0)
      acceleration = -acceleration;
    if (_acceleration != acceleration)
    {
	// Recompute _n per Equation 17
	_n = _n * (_acceleration / acceleration);
	// New c0 per Equation 7, with correction per Equation 15
	_c0 = 0.676 * sqrt(2.0 / acceleration) * 1000000.0; // Equation 15
	_acceleration = acceleration;
	computeNewSpeed();
    }
}

float  acceleration()
{
    return _acceleration;
}

void setSpeed(float speed)
{
    if (speed == _speed)
        return;
    speed = (speed) > (-_maxSpeed) ? ( (speed < _maxSpeed)?(speed):(_maxSpeed) ) : (-_maxSpeed) ; //constrain(speed, -_maxSpeed, _maxSpeed);

    if (speed == 0.0)
	_stepInterval = 0;
    else
    {
	_stepInterval = fabs(1000000.0 / speed);
	_direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    _speed = speed;
    //added
//    computeNewSpeed();
}

void setOutputPins(uint8_t mask)
{
    uint8_t numpins = 2;

    uint8_t i;
    for (i = 0; i < numpins; i++)
    	if(i==1){ //step pin
    		HAL_GPIO_WritePin( step_port, step_pin, (mask & (1 << i)) ? ( 1  ^ _pinInverted[i]) : (0 ^ _pinInverted[i]) );
    	}
    	else{
    		HAL_GPIO_WritePin( dir_port, dir_pin, (mask & (1 << i)) ? ( 1  ^ _pinInverted[i]) : (0 ^ _pinInverted[i]) );
    	}
}

float speed()
{
    return _speed;
}

void step(uint32_t step){

	//making step with driver
	step1(step);

}


// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 7)
// Subclasses can override
void step1(uint32_t step)
{
	(void)(step); // Unused

	// _pin[0] is step, _pin[1] is direction
	setOutputPins(_direction ? 0b10 : 0b00); // Set direction first else get rogue pulses
	setOutputPins(_direction ? 0b11 : 0b01); // step HIGH
	// Caution 200ns setup time
	// Delay the minimum allowed pulse width

	delay_us(_minPulseWidth);
	setOutputPins(_direction ? 0b10 : 0b00); // step LOW
}

//int32_t stepForward()
//{
//    // Clockwise
//    _currentPos += 1;
//	step(_currentPos);
////	_lastStepTime = micros();
//    return _currentPos;
//}

//int32_t stepBackward()
//{
//    // Counter-clockwise
//    _currentPos -= 1;
//	step(_currentPos);
////	_lastStepTime = micros();
//    return _currentPos;
//}

void setEnablePin(uint8_t enablePin)
{
    _enablePin = enablePin;

    // This happens after construction, so init pin now.
    if (_enablePin != 0xff)
    {
    	//need to write based on errors
//        pinMode(_enablePin, OUTPUT);
//        digitalWrite(_enablePin, HIGH ^ _enableInverted);
    }
}

void setMinPulseWidth(unsigned int minWidth)
{
    _minPulseWidth = minWidth;
}

void setPinsInverted(_Bool directionInvert, _Bool stepInvert, _Bool enableInvert)
{
    _pinInverted[0] = stepInvert;
    _pinInverted[1] = directionInvert;
    _enableInverted = enableInvert;
}

void runToPosition()
{
    while (run());
//	YIELD; // Let system housekeeping occur
}

_Bool runSpeedToPosition()
{
    if (_targetPos == _currentPos)
	return false;
    if (_targetPos >_currentPos)
	_direction = DIRECTION_CW;
    else
	_direction = DIRECTION_CCW;
    return runSpeed();
}

void runToNewPosition(int32_t position)
{
    moveTo(position);
    runToPosition();
}

void stop()
{
    if (_speed != 0.0)
    {
	int32_t stepsToStop = (int32_t)((_speed * _speed) / (2.0 * _acceleration)) + 1; // Equation 16 (+integer rounding)
	if (_speed > 0)
	    move(stepsToStop);
	else
	    move(-stepsToStop);
    }
}

_Bool isRunning()
{
    return !(_speed == 0.0 && _targetPos == _currentPos);
}



/*********** Helper Functions ************/
void delay_us (uint16_t us){
//{	__HAL_TIM_DISABLE
//	HAL_TIM_Base_Stop(&htim7);
	__HAL_TIM_SET_COUNTER(&htim7,0x0000);  // set the counter value a 0

#if debug
	counter = __HAL_TIM_GET_COUNTER(&htim7);
#endif
//	HAL_TIM_Base_Start(&htim7);

	while (__HAL_TIM_GET_COUNTER(&htim7) < us);  // wait for the counter to reach the us input in the parameter
#if debug
	counter = __HAL_TIM_GET_COUNTER(&htim7);
#endif
}




