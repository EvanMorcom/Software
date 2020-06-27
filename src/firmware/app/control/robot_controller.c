#include "firmware_new/boards/frankie_v1/io/drivetrain.h"
#include "firmware/app/control/trajectory_planner.h"

#include <assert.h>
#include <stdbool.h>

static DrivetrainUnit_t *_front_left_drive_unit;
static DrivetrainUnit_t *_front_right_drive_unit;
static DrivetrainUnit_t *_back_left_drive_unit;
static DrivetrainUnit_t *_back_right_drive_unit;
static VelocityTrajectory_t* _trajectory;
static size_t _trajectory_index = 0;
static size_t _num_trajectory_elements = 0;

static bool initialized = false;

void io_wheel_controller_init(DrivetrainUnit_t *front_left_drive_unit,
                              DrivetrainUnit_t *front_right_drive_unit,
                              DrivetrainUnit_t *back_left_drive_unit,
                              DrivetrainUnit_t *back_right_drive_unit)
{
    _front_left_drive_unit  = front_left_drive_unit;
    _front_right_drive_unit = front_right_drive_unit;
    _back_left_drive_unit   = back_left_drive_unit;
    _back_right_drive_unit  = back_right_drive_unit;

    initialized = true;
}

void io_robot_controller_updateTrajectory(VelocityTrajectory_t* trajectory, size_t num_elements){

    // Update the local values to the new ones
    _num_trajectory_elements = num_elements;
    _trajectory = trajectory;

    // Now that there is a new trajectory, reset the current tracking index
    _trajectory_index = 0;
}


