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

void PenaltyKickTactic::calculateNextIntent(IntentCoroutine::push_type& yield)
{

    // We will need to keep track of time so we don't break the rules by taking too long
    Timestamp penalty_kick_start = robot->getMostRecentTimestamp();


    //TODO: First we want to move to the ball (Wherever the referee put it)
    MoveAction approach_ball_move_act = MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, false);


    // Step 1: We want to get our shooter close to the ball as an initial position
    while( !approach_ball_move_act.done() ) {

        // We actually want to line up based on the direction to the net
        const Vector direction_net_to_ball = (field.enemyGoal() - ball.position()).norm();

        // Calculate a starting position aiming at the center of the net
        const Point shot_start_pos =
                ball.position() + direction_net_to_ball.norm(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS);

        yield(approach_ball_move_act.updateStateAndGetNextIntent(*robot, shot_start_pos,
                                                      (-1 * direction_net_to_ball).orientation(), 0));
    }

    //TODO: GET CLOSE AND RUN THE DRIBBLER (BETTER DEAKING)
    MoveAction grab_ball_move_act = MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD/2.0, false);

    // Step 2: Get close to the net with the dribber ON (Make sure not to dibble further than 1m from the point of contact with the ball)
    while( !grab_ball_move_act.done() ) {

        // We want to line up based on the direction to the net
        const Vector direction_net_to_ball = (field.enemyGoal() - ball.position()).norm();

        // Calculate a starting position aiming at the center of the net
        const Point shot_start_pos =
                ball.position() + direction_net_to_ball.norm(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS);

        yield(grab_ball_move_act.updateStateAndGetNextIntent(*robot, shot_start_pos,
                                                      (-1 * direction_net_to_ball).orientation(), 0, ENABLE_DRIBBLER));
    }

    // Step 3: Now we want to 'juke' out the enemy goalie with movement
    while( (penalty_kick_start - robot->getMostRecentTimestamp()) < penalty_shot_timeout ) {

        // Evaluate shots using only the enemy goalie and shooter
        std::optional<std::pair<Point, Angle>> best_shot_data = Evaluation::calcBestShotOnEnemyGoal(field, Team(Duration::fromSeconds(20), { *robot }), Team(Duration::fromSeconds(20), {enemy_goalie}), *robot);

        // If we have a solid shot on net TODO: Make this work better for goal shots (we want to shot as FAR away from enemy goalie as possible (Make my own function?
        if( best_shot_data.has_value() && (best_shot_data.value().second > Angle::ofDegrees(30.0)) ) {

        }

    }
    // WHERE DO WE WANT TO GO? We want to get as close to the net was possible without losing the ball
    // If we see a good shot we want to take it
    // Approach 1 corner of the net
    // Aim between goalposts looking for an opening (we should have the highest chance of scoring in these positions)

    //move_action.

    //TODO: We need to be running the dribbler (perhaps make slight deak-y moves)

    //TODO: End conditions: 1. We run out of time. 2. We have a good shot on net


//    MoveAction move_action = MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, true);
//
//    // Move to a position just behind the ball (in the direction of the pass)
//    // until it's time to perform the pass
//    while (ball.lastUpdateTimestamp() < pass.startTime())
//    {
//        // We want to wait just behind where the pass is supposed to start, so that the
//        // ball is *almost* touching the kicker
//        Vector ball_offset =
//                Vector::createFromAngle(pass.passerOrientation())
//                        .norm(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS * 2);
//        Point wait_position = pass.passerPoint() - ball_offset;
//
//        yield(move_action.updateStateAndGetNextIntent(*robot, wait_position,
//                                                      pass.passerOrientation(), 0));
//    }
//
//    // The angle between the ball velocity vector and a vector from the passer
//    // point to the receiver point
//    Angle ball_velocity_to_pass_orientation;
//
//    KickAction kick_action = KickAction();
//    do
//    {
//        // We want the robot to move to the starting position for the shot and also
//        // rotate to the correct orientation to face the shot
//        yield(kick_action.updateStateAndGetNextIntent(
//                *robot, ball, ball.position(), pass.receiverPoint(), pass.speed()));
//
//        // We want to keep trying to kick until the ball is moving along the pass
//        // vector with sufficient velocity
//        Angle passer_to_receiver_angle =
//                (pass.receiverPoint() - pass.passerPoint()).orientation();
//        ball_velocity_to_pass_orientation =
//                ball.velocity().orientation().minDiff(passer_to_receiver_angle);
//    } while (ball_velocity_to_pass_orientation.abs() > Angle::ofDegrees(20) ||
//             ball.velocity().len() < 0.5);
}
