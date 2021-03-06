#pragma once

#include <cinttypes>
#include <memory>

#include "software/primitive/primitive.h"
#include "software/simulation/physics/physics_ball.h"
#include "software/simulation/physics/physics_robot.h"

extern "C"
{
#include "firmware/app/primitives/primitive_manager.h"
}

/**
 * Because the PrimitiveManager_t struct is defined in the .c file (rather than the .h
 * file), C++ considers it an incomplete type and is unable to use it with smart pointers
 * because it doesn't know the size of the object. Therefore we need to create our own
 * "Deleter" class we can provide to the smart pointers to handle that instead.
 *
 * See https://en.cppreference.com/w/cpp/memory/unique_ptr/unique_ptr for more info and
 * examples
 */
struct PrimitiveManagerDeleter
{
    void operator()(PrimitiveManager_t* primitive_manager) const
    {
        app_primitive_manager_destroy(primitive_manager);
    };
};

/**
 * The SimulatorRobot class acts as a wrapper for a PhysicsRobot that deals with more
 * logic-focused elements for simulation, such as whether or not autokick is enabled.
 *
 * All members of this class are intentionally protected or private to force this class
 * to only be controlled by the SimulatorRobotSingleton.
 */
class SimulatorRobot
{
    friend class SimulatorRobotSingleton;

   public:
    /**
     * Create a new SimulatorRobot given a PhysicsRobot
     *
     * @param physics_robot the PhysicsRobot to simulate and control
     */
    explicit SimulatorRobot(std::weak_ptr<PhysicsRobot> physics_robot);
    explicit SimulatorRobot() = delete;

    /**
     * Returns the ID of this robot
     *
     * @return the ID of this robot
     */
    unsigned int getRobotId();

   protected:
    /**
     * Returns the x-position of the robot, in global field coordinates, in meters
     *
     * @return the x-position of the robot, in global field coordinates, in meters
     */
    float getPositionX();

    /**
     * Returns the y-position of the robot, in global field coordinates, in meters
     *
     * @return the y-position of the robot, in global field coordinates, in meters
     */
    float getPositionY();

    /**
     * Returns the orientation of the robot, in global field coordinates, in radians
     *
     * @return the orientation of the robot, in global field coordinates, in radians
     */
    float getOrientation();

    /**
     * Returns the x-velocity of the robot, in global field coordinates, in m/s
     *
     * @return the x-velocity of the robot, in global field coordinates, in m/s
     */
    float getVelocityX();

    /**
     * Returns the y-velocity of the robot, in global field coordinates, in m/s
     *
     * @return the y-velocity of the robot, in global field coordinates, in m/s
     */
    float getVelocityY();

    /**
     * Returns the angular velocity of the robot, in rad/s
     *
     * @return the angular of the robot, in rad/s
     */
    float getVelocityAngular();

    /**
     * Returns the battery voltage, in volts
     *
     * @return the battery voltage, in volts
     */
    float getBatteryVoltage();

    /**
     * Fires the kicker, kicking the ball in the direction the robot is facing
     * at the given speed if the ball is very close to the kicker
     *
     * @param speed_m_per_s How fast to kick the ball, in meters per second
     */
    void kick(float speed_m_per_s);

    /**
     * Fires the chipper, chipping the ball in the direction the robot is facing
     * for the given distance if the ball is very close to the chipper
     *
     * @param speed_m_per_s How far to chip the ball (the distance to the first bounce)
     * in meters
     */
    void chip(float distance_m);

    /**
     * Enables autokick on the robot. If the ball touches the kicker, the robot will
     * kick the ball with the given speed.
     *
     * @param speed_m_per_s How fast to kick the ball in meters per second when
     * the kicker is fired
     */
    void enableAutokick(float speed_m_per_s);

    /**
     * Enables autochip on the robot. If the ball touches the chipper, the robot will
     * chip the ball the given distance.
     *
     * @param speed_m_per_s How far to chip the ball (distance to the first bounce)
     * when the chipper is fired
     */
    void enableAutochip(float distance_m);

    /**
     * Disables autokick
     */
    void disableAutokick();

    /**
     * Disables autochip
     */
    void disableAutochip();

    /**
     * Returns true if autokick is enabled and false otherwise
     *
     * @return true if autokick is enabled and false otherwise
     */
    bool isAutokickEnabled();

    /**
     * Returns true if autochip is enabled and false otherwise
     *
     * @return true if autochip is enabled and false otherwise
     */
    bool isAutochipEnabled();

    /**
     * Sets the speed of the dribbler
     *
     * @param rpm The rpm to set for the dribbler
     */
    void setDribblerSpeed(uint32_t rpm);

    /**
     * Makes the dribbler coast until another operation is applied to it
     */
    void dribblerCoast();

    /**
     * Returns the temperature of the dribbler, in degrees C
     *
     * @return the temperature of the dribbler, in degrees C
     */
    unsigned int getDribblerTemperatureDegC();

    /**
     * Applies the given force to the wheel
     *
     * @param force_in_newtons the force to apply to the wheel
     */
    void applyWheelForceFrontLeft(float force_in_newtons);
    void applyWheelForceBackLeft(float force_in_newtons);
    void applyWheelForceBackRight(float force_in_newtons);
    void applyWheelForceFrontRight(float force_in_newtons);

    /**
     * Gets the motor speed for the wheel, in RPM
     */
    float getMotorSpeedFrontLeft();
    float getMotorSpeedBackLeft();
    float getMotorSpeedBackRight();
    float getMotorSpeedFrontRight();

    /**
     * Sets the motor to coast (spin freely)
     */
    void coastMotorBackLeft();
    void coastMotorBackRight();
    void coastMotorFrontLeft();
    void coastMotorFrontRight();

    /**
     * Sets the motor to brake (act against the current direction of rotation)
     */
    void brakeMotorBackLeft();
    void brakeMotorBackRight();
    void brakeMotorFrontLeft();
    void brakeMotorFrontRight();

    /**
     * Sets the current primitive this robot is running to a new one
     *
     * @param firmware_world The world to run the primitive in
     * @param primitive_index The index of the primitive to run
     * @param params The parameters for the primitive
     */
    void startNewPrimitive(std::shared_ptr<FirmwareWorld_t> firmware_world,
                           unsigned int primitive_index,
                           const primitive_params_t& params);

    /**
     * Runs the current primitive
     *
     * @param world The world to run the primitive in
     */
    void runCurrentPrimitive(std::shared_ptr<FirmwareWorld_t> firmware_world);

   private:
    /**
     * A function that is called during every physics step for as long as the ball
     * is touching this robot's chicker
     *
     * @param physics_robot The robot involved in the contact
     * @param physics_ball The ball invovled in the contact
     */
    void onChickerBallContact(PhysicsRobot* physics_robot, PhysicsBall* physics_ball);

    /**
     * A function that is called during every physics step for as long as the ball
     * is touching this robot's dribbler
     *
     * @param physics_robot The robot involved in the contact
     * @param physics_ball The ball invovled in the contact
     */
    void onDribblerBallContact(PhysicsRobot* physics_robot, PhysicsBall* physics_ball);

    /**
     * A function that is called during once when the ball starts touching
     * this robot's dribbler.
     *
     * @param physics_robot The robot involved in the contact
     * @param physics_ball The ball invovled in the contact
     */
    void onDribblerBallStartContact(PhysicsRobot* physics_robot,
                                    PhysicsBall* physics_ball);

    /**
     * A function that is called during once when the ball stops touching
     * this robot's dribbler.
     *
     * @param physics_robot The robot involved in the contact
     * @param physics_ball The ball invovled in the contact
     */
    void onDribblerBallEndContact(PhysicsRobot* physics_robot, PhysicsBall* physics_ball);

    /**
     * Helper functions that check if the current pointer to the physics_robot is valid
     * before calling the given function. If the physics_robot is invalid, a warning is
     * logged and a default value is returned.
     *
     * @param func The function to perform on the physics robot
     */
    void checkValidAndExecuteVoid(
        std::function<void(std::shared_ptr<PhysicsRobot>)> func);
    float checkValidAndReturnFloat(
        std::function<float(std::shared_ptr<PhysicsRobot>)> func);
    unsigned int checkValidAndReturnUint(
        std::function<unsigned int(std::shared_ptr<PhysicsRobot>)> func);

    std::weak_ptr<PhysicsRobot> physics_robot;
    std::optional<double> autokick_speed_m_per_s;
    std::optional<double> autochip_distance_m;
    uint32_t dribbler_rpm;

    // A pointer to all the balls currently being dribbled
    std::vector<PhysicsBall*> balls_in_dribbler_area;

    std::unique_ptr<PrimitiveManager, PrimitiveManagerDeleter> primitive_manager;
    std::optional<unsigned int> current_primitive_index;
    std::optional<primitive_params_t> current_primitive_params;
};
