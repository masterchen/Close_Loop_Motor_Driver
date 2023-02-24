/*
 * stepper.h
 *
 *  Created on: Jan 24, 2023
 *      Author: Ajith Pinninti
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

/*********************************** Macros definitons **********************/

#include "main.h"


//debugging mode
#define debug 1


//boolean
#define true 1
#define false 0

// \brief Direction indicator
// Symbolic names for the direction the motor is turning

#define DIRECTION_CCW  0  ///< Counter-Clockwise
#define DIRECTION_CW   1   ///< Clockwise

/******************************** Private Variable declaration *************************************/

	extern GPIO_TypeDef*  step_port;
	extern uint16_t step_pin;

	extern GPIO_TypeDef*  dir_port;
	extern uint16_t dir_pin;


	extern _Bool _direction; // 1 == CW

	/// The current interval between steps in microseconds.
	/// 0 means the motor is currently stopped with _speed == 0
	extern uint32_t  _stepInterval;

/************************************** Private Variable Declarations *********************************/
	/// Number of pins on the stepper motor. Permits 2 or 4. 2 pins is a
    /// bipolar, and 4 pins is a unipolar.
    extern uint8_t        _interface;          // 0, 1, 2, 4, 8, See MotorInterfaceType

    /// Arduino pin number assignments for the 2 or 4 pins required to interface to the
    /// stepper motor or driver
    extern uint8_t        _pin[4];

    /// Whether the _pins is inverted or not
    extern uint8_t        _pinInverted[4];

    /// The current absolution position in steps.
    extern int32_t           _currentPos;    // Steps

    /// The target position in steps. The AccelStepper library will move the
    /// motor from the _currentPos to the _targetPos, taking into account the
    /// max speed, acceleration and deceleration
    extern int32_t           _targetPos;     // Steps

    /// The current motos speed in steps per second
    /// Positive is clockwise
    extern float          _speed;         // Steps per second

    /// The maximum permitted speed in steps per second. Must be > 0.
    extern float          _maxSpeed;

    /// The acceleration to use to accelerate or decelerate the motor in steps
    /// per second per second. Must be > 0
    extern float          _acceleration;
    extern float          _sqrt_twoa; // Precomputed sqrt(2*_acceleration)

    /// The last step time in microseconds
    extern uint32_t  _lastStepTime;

    /// The minimum allowed pulse width in microseconds
    extern uint32_t   _minPulseWidth;

    /// Is the direction pin inverted?
    ///bool           _dirInverted; /// Moved to _pinInverted[1]

    /// Is the step pin inverted?
    ///bool           _stepInverted; /// Moved to _pinInverted[0]

    /// Is the enable pin inverted?
    extern _Bool           _enableInverted;

    /// Enable pin for stepper driver, or 0xFF if unused.
    extern uint8_t        _enablePin;

    /// The pointer to a forward-step procedure
    extern void (*_forward)();

    /// The pointer to a backward-step procedure
    extern void (*_backward)();

    /// The step counter for speed calculations
    extern int32_t _n;

    /// Initial step size in microseconds
    extern float _c0;

    /// Last step size in microseconds
    extern float _cn;

    /// Min step size in microseconds based on maxSpeed
    extern float _cmin; // at max speed



/***************************** private Function Definition **************************/

    	/*
    }
	 * @brief intialize the step and dir pin of the motor
	 *
	 * @params step_pin_port GPIO pin port of the direction pin
	 *
	 */
	void stepper_setup(GPIO_TypeDef*  _step_port,uint16_t _step_pin, GPIO_TypeDef*  _dir_port,uint16_t _dir_pin );
	/*
	 *
	 * Set the target position. The run() function will try to move the motor (at most one step per call)
	 * from the current position to the target position set by the most
	 * recent call to this function. Caution: moveTo() also recalculates the speed for the next step.
	 * If you are trying to use constant speed movements, you should call setSpeed() after calling moveTo().
	 * @param[in] absolute The desired absolute position. Negative is
	 * anticlockwise from the 0 position.
	  */
	void   moveTo(int32_t absolute);

	/// Set the target position relative to the current position.
    /// \param[in] relative The desired position relative to the current position. Negative is
    /// anticlockwise from the current position.
    void    move(int32_t relative);

    /// Poll the motor and step it if a step is due, implementing
    /// accelerations and decelerations to achieve the target position. You must call this as
    /// frequently as possible, but at least once per minimum step time interval,
    /// preferably in your main loop. Note that each call to run() will make at most one step, and then only when a step is due,
    /// based on the current speed and the time since the last step.
    /// \return true if the motor is still running to the target position.
    _Bool run();

    /// Poll the motor and step it if a step is due, implementing a constant
    /// speed as set by the most recent call to setSpeed(). You must call this as
    /// frequently as possible, but at least once per step interval,
    /// \return true if the motor was stepped.
    _Bool runSpeed();

    /// Sets the maximum permitted speed. The run() function will accelerate
    /// up to the speed set by this function.
    /// Caution: the maximum speed achievable depends on your processor and clock speed.
    /// The default maxSpeed is 1.0 steps per second.
    /// \param[in] speed The desired maximum speed in steps per second. Must
    /// be > 0. Caution: Speeds that exceed the maximum speed supported by the processor may
    /// Result in non-linear accelerations and decelerations.
    void    setMaxSpeed(float speed);

    /// Returns the maximum speed configured for this stepper
    /// that was previously set by setMaxSpeed();
    /// \return The currently configured maximum speed
    float   maxSpeed();

    /// Sets the acceleration/deceleration rate.
    /// \param[in] acceleration The desired acceleration in steps per second
    /// per second. Must be > 0.0. This is an expensive call since it requires a square
    /// root to be calculated. Dont call more ofthen than needed
    void    setAcceleration(float acceleration);

    /// Returns the acceleration/deceleration rate configured for this stepper
    /// that was previously set by setAcceleration();
    /// \return The currently configured acceleration/deceleration
    float   acceleration();

    /// Sets the desired constant speed for use with runSpeed().
    /// \param[in] speed The desired constant speed in steps per
    /// second. Positive is clockwise. Speeds of more than 1000 steps per
    /// second are unreliable. Very slow speeds may be set (eg 0.00027777 for
    /// once per hour, approximately. Speed accuracy depends on the Arduino
    /// crystal. Jitter depends on how frequently you call the runSpeed() function.
    /// The speed will be limited by the current value of setMaxSpeed()
    void    setSpeed(float speed);

    /// The most recently set speed.
    /// \return the most recent speed in steps per second
    float   speed();

    /// The distance from the current position to the target position.
    /// \return the distance from the current position to the target position
    /// in steps. Positive is clockwise from the current position.
    int32_t    distanceToGo();

    /// The most recently set target position.
    /// \return the target position
    /// in steps. Positive is clockwise from the 0 position.
    int32_t    targetPosition();

    /// The current motor position.
    /// \return the current motor position
    /// in steps. Positive is clockwise from the 0 position.
    int32_t    currentPosition();

    /// Resets the current position of the motor, so that wherever the motor
    /// happens to be right now is considered to be the new 0 position. Useful
    /// for setting a zero position on a stepper after an initial hardware
    /// positioning move.
    /// Has the side effect of setting the current motor speed to 0.
    /// \param[in] position The position in steps of wherever the motor
    /// happens to be right now.
    void    setCurrentPosition(int32_t position);

    /// Moves the motor (with acceleration/deceleration)
    /// to the target position and blocks until it is at
    /// position. Dont use this in event loops, since it blocks.
    void    runToPosition();

    /// Executes runSpeed() unless the targetPosition is reached.
    /// This function needs to be called often just like runSpeed() or run().
    /// Will step the motor if a step is required at the currently selected
    /// speed unless the target position has been reached.
    /// Does not implement accelerations.
    /// \return true if it stepped
    _Bool runSpeedToPosition();

    /// Moves the motor (with acceleration/deceleration)
    /// to the new target position and blocks until it is at
    /// position. Dont use this in event loops, since it blocks.
    /// \param[in] position The new target position.
    void    runToNewPosition(int32_t position);

    /// Sets a new target position that causes the stepper
    /// to stop as quickly as possible, using the current speed and acceleration parameters.
    void stop();

    /// Disable motor pin outputs by setting them all LOW
    /// Depending on the design of your electronics this may turn off
    /// the power to the motor coils, saving power.
    /// This is useful to support Arduino low power modes: disable the outputs
    /// during sleep and then reenable with enableOutputs() before stepping
    /// again.
    /// If the enable Pin is defined, sets it to OUTPUT mode and clears the pin to disabled.

/*******************************************Protected definitions**************************************************/


    /// Forces the library to compute a new instantaneous speed and set that as
	/// the current speed. It is called by
	/// the library:
	/// \li  after each step
	/// \li  after change to maxSpeed through setMaxSpeed()
	/// \li  after change to acceleration through setAcceleration()
	/// \li  after change to target position (relative or absolute) through
	/// move() or moveTo()
	/// \return the new step interval
	uint32_t  computeNewSpeed();


	/// Called to execute a step. Only called when a new step is
	/// required. Subclasses may override to implement new stepping
	/// interfaces. The default calls step1(), step2(), step4() or step8() depending on the
	/// number of pins defined for the stepper.
	/// \param[in] step The current step phase number (0 to 7)
    void   step(uint32_t step);

	/// Called to execute a step on a stepper driver (ie where pins == 1). Only called when a new step is
	/// required. Subclasses may override to implement new stepping
	/// interfaces. The default sets or clears the outputs of Step pin1 to step,
	/// and sets the output of _pin2 to the desired direction. The Step pin (_pin1) is pulsed for 1 microsecond
	/// which is the minimum STEP pulse width for the 3967 driver.
	/// \param[in] step The current step phase number (0 to 7)
	 void  step1(uint32_t step);

	 /// Low level function to set the motor output pins
	 /// bit 0 of the mask corresponds to _pin[0]
	 /// bit 1 of the mask corresponds to _pin[1]
	 /// You can override this to impment, for example serial chip output insted of using the
	 /// output pins directly
	 void   setOutputPins(uint8_t mask);

	 /// function to create delay in us
	 // it'll use TIMER 7 peripheral for this operation
	 void delay_us(uint16_t delay);


/*
 * Variables
 */


#endif /* INC_STEPPER_H_ */
