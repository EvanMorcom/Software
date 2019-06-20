#pragma once

#include "ai/hl/stp/tactic/tactic.h"
#include "ai/passing/pass.h"

/**
 * This tactic is for a robot performing a penalty kick.
 */
class PenaltyKickTactic : public Tactic
{
public:
    /**
     * Creates a new PenaltyKickTactic
     *
     * @param pass The pass this tactic should try to execute
     * @param ball The ball that we're trying to pass
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit PenaltyKickTactic(const Ball& ball, bool loop_forever = false);

    std::string getName() const override;

    /**
     * Updates the parameters for this PenaltyKickTactic.
     *
     * @param updated_ball The ball we're passing
     */
    void updateParams(const Ball& updated_ball, const Robot& enemy_goalie);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the block destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) override;

private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Tactic parameters
    Robot enemy_goalie;
    Ball ball;
};
