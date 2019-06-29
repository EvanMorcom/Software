/**
 * Implementation of the PenaltyKickTactic
 */
#include "penalty_kick.h"

#include "ai/hl/stp/action/kick_action.h"
#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/action/dribble_action.h"
#include "geom/util.h"
#include "shared/constants.h"
#include "util/logger/init.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"


PenaltyKickTactic::PenaltyKickTactic(const Ball& ball, const Field& field, const std::optional<Robot>& enemy_goalie, bool loop_forever)
        : ball(ball), field(field), enemy_goalie(enemy_goalie), Tactic(loop_forever)
{
}

std::string PenaltyKickTactic::getName() const
{
    return "Penalty Kick Tactic";
}

void PenaltyKickTactic::updateParams(const Ball& updated_ball, const std::optional<Robot>& updated_enemy_goalie, const Field& updated_field)
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


    // We will make a penalty shot if the enemy goalie cannot accelerate in time to block it
    Segment goal_line = Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg());
   // Ray shot_ray = Ray(robot.value().position(), robot.value().orientation().)

    if(enemy_goalie.has_value()) {
        return robot.value().orientation().minDiff( (robot.value().position() - enemy_goalie.value().position()).orientation() );
    }
    else {
        // If there is no enemy goalie, the whole net is open
        return ((robot.value().position() - field.enemyGoalpostPos()).orientation().minDiff( (robot.value().position() - field.enemyGoalpostNeg()).orientation()));
    }
}

void PenaltyKickTactic::calculateNextIntent(IntentCoroutine::push_type& yield) {

    // Keep track if a shot has been taken
    bool shot_taken = false;

    // We will need to keep track of time so we don't break the rules by taking too long
    Timestamp penalty_kick_start = robot->getMostRecentTimestamp();


    MoveAction approach_ball_move_act = MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, false);
    KickAction kick_action = KickAction();
    DribbleAction dribble_action = DribbleAction();

    do {

        Vector behind_ball_vector = (ball.position() - field.enemyGoal());
        // A point behind the ball that leaves 5cm between the ball and kicker of the
        // robot
        Point behind_ball =
                ball.position() +
                behind_ball_vector.norm(BALL_MAX_RADIUS_METERS +
                                        DIST_TO_FRONT_OF_ROBOT_METERS + 0.04);

        // If we haven't approached the ball yet, get close

        if((robot.value().position() - behind_ball).len() <= MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD && (robot.value().orientation().minDiff((-behind_ball_vector).orientation()).toDegrees() < 5.0  )) {
            printf("\nMade it to ball!");

            // TODO This set angle value doesn't work well
            if( evaluate_penalty_shot() > Angle::ofDegrees(20)) {

                    printf("\nWaiting for kick action");
                    yield(kick_action.updateStateAndGetNextIntent(*robot, ball, ball.position(), field.enemyGoalpostPos(), PENALTY_KICK_SHOT_SPEED));

            }


        }
        else if( (robot.value().position() - ball.position()).len() > MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD || (robot.value().orientation().minDiff((-behind_ball_vector).orientation()).toDegrees() < 3.0 )) {

            // The default behaviour is to move behind the ball and face the net
            yield(approach_ball_move_act.updateStateAndGetNextIntent(
                    *robot, behind_ball, (-behind_ball_vector).orientation(), 0));
            printf("\nWaiting for move action");
        }
        else {
            printf("\nDoing nothing");
        }

    } while( ! (kick_action.done() || (penalty_kick_start - robot->getMostRecentTimestamp()) < penalty_shot_timeout ) );

}
//
//    //TODO: GET CLOSE AND RUN THE DRIBBLER (BETTER DEAKING)
//    MoveAction grab_ball_move_act = MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD / 2.0, false);
//
//    // Step 2: Get close to the net with the dribber ON (Make sure not to dibble further than 1m from the point of contact with the ball)
//    while (!grab_ball_move_act.done()) {
//
//        // We want to line up based on the direction to the net
//        const Vector direction_net_to_ball = (field.enemyGoal() - ball.position()).norm();
//
//        // Calculate a starting position aiming at the center of the net
//        const Point shot_start_pos =
//                ball.position() + direction_net_to_ball.norm(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS);
//
//        yield(grab_ball_move_act.updateStateAndGetNextIntent(*robot, shot_start_pos,
//                                                             (-1 * direction_net_to_ball).orientation(), 0,
//                                                             ENABLE_DRIBBLER));
//    }
//
//
//    // Grab the angle between the robot and the left/right enemy goal posts to shoot between
//    Angle robot_to_neg_post = (field.enemyGoalpostNeg() - robot.value().position()).orientation();
//    Angle robot_to_pos_post = (field.enemyGoalpostPos() - robot.value().position()).orientation();
//
//    // We want to alternate between the positive and negative goal posts to juke out the enemy goalie
//    bool is_facing_negative_post = false;
//
//    // Step 3: Now we want to 'juke' out the enemy goalie with movement
//    while (((penalty_kick_start - robot->getMostRecentTimestamp()) < penalty_shot_timeout) && !shot_taken) {
//
//        // Evaluate shots using only the enemy goalie and shooter
//        Angle shot_angle = evaluate_penalty_shot();
//
//        // If we have a solid shot on net TODO: Make this work better for goal shots (we want to shot as FAR away from enemy goalie as possible (Make my own function?
//        if ((shot_angle > Angle::ofDegrees(25.0))) {
//
//            KickAction kick_action = KickAction();
//            Point shot_location = is_facing_negative_post ? field.enemyGoalpostNeg() + Point(0, 0.02) :
//                                  field.enemyGoalpostPos() + Point(0, 0.02);
//
//            // Wait for kick to execute
//            while (!kick_action.done()) {
//
//                yield(kick_action.updateStateAndGetNextIntent(
//                        *robot, ball, ball.position(), shot_location, 5.0));
//            }
//            // Change shot flag to leave the loop
//            shot_taken = true;
//        }
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
//    }
//}

