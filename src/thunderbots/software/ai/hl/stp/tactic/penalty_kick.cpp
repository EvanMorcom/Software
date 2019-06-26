/**
 * Implementation of the PenaltyKickTactic
 */
#include "penalty_kick.h"

#include "ai/hl/stp/action/kick_action.h"
#include "ai/hl/stp/action/move_action.h"
#include "geom/util.h"
#include "shared/constants.h"
#include "util/logger/init.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"


PenaltyKickTactic::PenaltyKickTactic(const Ball& ball, const Field& field, const Robot& enemy_goalie, bool loop_forever)
        : ball(ball), field(field), enemy_goalie(enemy_goalie), Tactic(loop_forever)
{
}

std::string PenaltyKickTactic::getName() const
{
    return "Penalty Kick Tactic";
}

void PenaltyKickTactic::updateParams(const Ball& updated_ball, const Robot& updated_enemy_goalie, const Field& updated_field)
{
    this->enemy_goalie = updated_enemy_goalie;
    this->ball = updated_ball;
    this->field = updated_field;
}

double PenaltyKickTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    // Prefer robots closer to the pass start position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost =
            (robot.position() - world.ball().position()).len() / world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}

Angle PenaltyKickTactic::evaluate_penalty_shot() {

    // The value of a penalty shot is proportional to how far away the enemy goalie is from the current shot of the robot
    return robot.value().orientation().minDiff( (robot.value().position() - enemy_goalie.position()).orientation() );

}

void PenaltyKickTactic::calculateNextIntent(IntentCoroutine::push_type& yield) {

    // Keep track if a shot has been taken
    bool shot_taken = false;

    // We will need to keep track of time so we don't break the rules by taking too long
    Timestamp penalty_kick_start = robot->getMostRecentTimestamp();


    //TODO: First we want to move to the ball (Wherever the referee put it)
    MoveAction approach_ball_move_act = MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, false);


    // Step 1: We want to get our shooter close to the ball as an initial position
    while (!approach_ball_move_act.done()) {

        // We actually want to line up based on the direction to the net
        const Vector direction_net_to_ball = (field.enemyGoal() - ball.position()).norm();

        // Calculate a starting position aiming at the center of the net
        const Point shot_start_pos =
                ball.position() + direction_net_to_ball.norm(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS);

        yield(approach_ball_move_act.updateStateAndGetNextIntent(*robot, shot_start_pos,
                                                                 (-1 * direction_net_to_ball).orientation(), 0));
    }

    //TODO: GET CLOSE AND RUN THE DRIBBLER (BETTER DEAKING)
    MoveAction grab_ball_move_act = MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD / 2.0, false);

    // Step 2: Get close to the net with the dribber ON (Make sure not to dibble further than 1m from the point of contact with the ball)
    while (!grab_ball_move_act.done()) {

        // We want to line up based on the direction to the net
        const Vector direction_net_to_ball = (field.enemyGoal() - ball.position()).norm();

        // Calculate a starting position aiming at the center of the net
        const Point shot_start_pos =
                ball.position() + direction_net_to_ball.norm(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS);

        yield(grab_ball_move_act.updateStateAndGetNextIntent(*robot, shot_start_pos,
                                                             (-1 * direction_net_to_ball).orientation(), 0,
                                                             ENABLE_DRIBBLER));
    }


    // Grab the angle between the robot and the left/right enemy goal posts to shoot between
    Angle robot_to_neg_post = (field.enemyGoalpostNeg() - robot.value().position()).orientation();
    Angle robot_to_pos_post = (field.enemyGoalpostPos() - robot.value().position()).orientation();

    // We want to alternate between the positive and negative goal posts to juke out the enemy goalie
    bool is_facing_negative_post = false;

    // Step 3: Now we want to 'juke' out the enemy goalie with movement
    while (((penalty_kick_start - robot->getMostRecentTimestamp()) < penalty_shot_timeout) && !shot_taken) {

        // Evaluate shots using only the enemy goalie and shooter
        Angle shot_angle = evaluate_penalty_shot();

        // If we have a solid shot on net TODO: Make this work better for goal shots (we want to shot as FAR away from enemy goalie as possible (Make my own function?
        if ((shot_angle > Angle::ofDegrees(25.0))) {

            KickAction kick_action = KickAction();
            Point shot_location = is_facing_negative_post ? field.enemyGoalpostNeg() + Point(0, 0.02) :
                                  field.enemyGoalpostPos() + Point(0, 0.02);

            // Wait for kick to execute
            while (!kick_action.done()) {

                yield(kick_action.updateStateAndGetNextIntent(
                        *robot, ball, ball.position(), shot_location, 5.0));
            }
            // Change shot flag to leave the loop
            shot_taken = true;
        }
//
//        // If we didn't get a good shot, keep looking
//        // We want to face the opposite side of the enemy net than what we are currently facing
//        Angle angle_to_face = is_facing_negative_post ? robot_to_pos_post : robot_to_neg_post;
//
//        //while( !dribble_action.done()) {
//        //dribble_action.updateStateAndGetNextIntent(*robot, robot.value().position(), angle_to_face, 5000, false);
//        //
//        // Flip the value of the state boolean
//        is_facing_negative_post = !is_facing_negative_post;
    }
}

