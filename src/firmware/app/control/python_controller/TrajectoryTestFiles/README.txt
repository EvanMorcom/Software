Trajectories are generated using the following parameters:

    // The path parameterization value indicating the beginning of the
    // considered path
    t_start = 0

    // The path parameterization value indicating the end of the considered
    // path
    float t_end = 1

    // The number of segments to discretize the trajectory into.
    //  Must be greater than 2 segments.
    //  THE NUMBER OF SEGMENTS MUST BE UNDER
    //  TRAJECTORY_PLANNER_MAX_NUMBER_SEGMENTS
    unsigned int num_segments = 2000

    // The maximum acceleration allowed at any
    //  point along the trajectory. This factor limits the maximum delta-velocity
    // and
    //  also the max speed around curves due to centripetal acceleration [m/s^2]
    float max_allowable_acceleration = 3

    // The maximum speed allowable at any point along the trajectory```
    float max_allowable_speed = 3

    // The initial speed at the start of the trajectory [m/s]
    float initial_speed = 0

    // The final speed at the end of the trajectory [m/s]
    float final_speed = 0
