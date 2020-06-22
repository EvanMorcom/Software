#pragma once

#include "firmware/shared/circular_buffer.h"

// wheel_controller_difference_equation.h
typedef struct WheelController
{
    CircularBuffer_t* previous_command_buffer;
    CircularBuffer_t* previous_output_sample_buffer;
    float* command_coefficients;
    float* output_sample_coefficients;
    unsigned int num_command_coefficients;
    unsigned int num_output_sample_coefficients;
} WheelController_t;

/**
 * Function creates a WheelController_t with the specified parameters and returns a
 pointer to it. The wheel_controller_destroy function MUST be called before the
 WheelController_t* goes out of scope.
 * The WheelController_t allows previously sampled wheel states to be added to it's state,
 and allows for new wheel commands to be requested.
 *
 * The coefficients parameters are in order of decreasing recentness - where
 coefficient_array[0] is the most recent value.
 *
 * The convention is as follows:
 * coeff[0] -> z^0
 * coeff[1] -> z^-1
 * coeff[2] -> z^-2
 * ...
 * coeff[N] -> z^-N
 *
 * NOTE: All coefficients MUST be specified. That is if the difference equation is A*z^-3
 + B*z^-1 the z^-2 coefficient MUST be included. Ex, A*z^-3 + 0*z^-2 + B*z^-1.
 *
 * @param command_coefficients [in] The coefficients for the controller command inputs.
 These are the coefficients relating to how previous input effects the controller output.
 *
 * @param num_command_coefficients [in] The number of elements in the command_coefficients
 array
 *
 * @param output_sample_coefficients [in] The coefficients for the controller sampled
 output. These are the coefficients relating to how the sampled output(wheel state)
 effects the controller output.

 * @param num_sample_coefficient [in] The number of elements in the
 sampled_output)coefficients array.
 *
 * @return Pointer to the newly created WheelController_t
 */
WheelController_t* app_wheel_controller_create(
    float* command_coefficients, unsigned int num_command_coefficients,
    float* output_sample_coefficients, unsigned int num_output_sample_coefficients);
/**
 * Function pushes a new command value to the specified WheelController.
 *
 * @param wheel_controller [in] The WheelController that is the target of the new command
 * value.
 *
 * @param command [in] The value of the commanded state requested.
 */
void app_wheel_controller_pushNewCommand(WheelController_t* wheel_controller,
                                         float command);

/**
 * Function pushes a newly sampled output(wheel state) to the WheelController.
 *
 * @param wheel_controller [in] The WheelController that will receive the new output
 * sample.
 *
 * @param output_sample [in] Measured value of the current wheel state.
 */
void app_wheel_controller_pushNewSampleOutput(WheelController_t* wheel_controller,
                                              float output_sample);

/**
 * Function returns a new wheel voltage to apply to the wheel motor (V) based on previous
 * command input and previous sampled wheel states.
 *
 * @param wheel_controller [in] The WheelController to calculate the new applied voltage
 * for.
 *
 * @return Wheel voltage to apply to achieve the desired state.
 */
float app_wheel_controller_getWheelVoltage(WheelController_t* wheel_controller);

/**
 * Function destroys the specified instance of WheelController, de-allocating all memory
 * reserved for it.
 *
 * @param wheel_controller [in] WheelController_t to destroy.
 */
void app_wheel_controller_destroy(WheelController_t* wheel_controller);