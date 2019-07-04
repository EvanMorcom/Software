#include "ai/hl/stp/tactic/penalty_goalie.h"

#include "ai/hl/stp/action/chip_action.h"
#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "ai/hl/stp/evaluation/possession.h"
#include "geom/point.h"
#include "geom/ray.h"
#include "geom/segment.h"
#include "geom/util.h"
#include "shared/constants.h"
#include "util/parameter/dynamic_parameters.h"


PenaltyGoalieTactic::PenaltyGoalieTactic(const Ball &ball, const Field &field,
                                         const Team &friendly_team,
                                         const Team &enemy_team)
    : ball(ball),
      field(field),
      friendly_team(friendly_team),
      enemy_team(enemy_team),
      Tactic(true)
{
    addWhitelistedAvoidArea(AvoidArea::FRIENDLY_DEFENSE_AREA);
    addWhitelistedAvoidArea(AvoidArea::BALL);
    addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
}

std::string PenaltyGoalieTactic::getName() const
{
    return "Penalty Goalie Tactic";
}

void PenaltyGoalieTactic::updateParams(const Ball &ball, const Field &field,
                                       const Team &friendly_team, const Team &enemy_team)
{
    // Update the parameters stored by this Tactic
    this->ball          = ball;
    this->field         = field;
    this->friendly_team = friendly_team;
    this->enemy_team    = enemy_team;
}

double PenaltyGoalieTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Strongly prefer the robot assigned to be the goalie.
    // TODO: This is a hack to "ensure" the right robot will be assigned. We should
    // normally return values in the range [0, 1]
    if (world.friendlyTeam().getGoalieID() &&
        robot.id() == world.friendlyTeam().getGoalieID().value())
    {
        return 0.0;
    }
    else
    {
        return 1000000;
    }
}

void PenaltyGoalieTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    MoveAction move_action = MoveAction();
    ChipAction chip_action = ChipAction();
    do
    {
        // Goalie Tactic
        //
        // The goalie tactic is responsible for blocking as many shots as it can, to the
        // best of its ability. The tactic consists of 3 cases
        //
        // Case 1: The ball is moving towards the goal and has a speed that is concerning
        //      Goalie moves onto the closest point on the oncoming line to stop the ball
        //
        // Case 2: The ball is moving at a slow speed and is inside the defense area
        //      Goalie moves to the ball and chips it out of the defense area
        //
        // Case 3: Any other case
        //      Goalie blocks the cone to the net. (Cone being from the ball to either
        //      goal post) The goalie also snaps to a semicircle inside the defense area,
        //      to avoid leaving the defense area
        //
        std::unique_ptr<Intent> next_intent;

        // compute intersection points from ball position and velocity
        Ray ball_ray = Ray(ball.position(), ball.velocity());
        Segment full_goal_segment =
            Segment(field.friendlyGoalpostNeg(), field.friendlyGoalpostPos());

        auto [intersection1, intersection2] =
            raySegmentIntersection(ball_ray, full_goal_segment);

        // Case 1
        if (intersection1.has_value() &&
            ball.velocity().len() > BALL_SLOW_SPEED_THRESHOLD)
        {
            // the ball is heading towards the net. move to block the shot
            Point goalie_pos = intersection1.value();

            next_intent = move_action.updateStateAndGetNextIntent(
                *robot, goalie_pos, robot->orientation(),
                Util::DynamicParameters::GoalieTactic::goalie_final_speed.value(), false,
                AUTOCHIP);
        }
        // Case 2
        else if (ball.velocity().len() < BALL_SLOW_SPEED_THRESHOLD &&
                 field.pointInFriendlyDefenseArea(ball.position()))
        {
            // if the ball is slow or stationary inside our defense area, chip it out
            next_intent = chip_action.updateStateAndGetNextIntent(
                *robot, ball, ball.position(),
                (ball.position() - field.friendlyGoal()).orientation(), 2);
        }
        // Case 3
        else
        {
            // Calculate the Robot that is shoting
            std::optional<Robot> enemy_with_possesion =
                Evaluation::getRobotWithEffectiveBallPossession(enemy_team, ball, field);

            if (enemy_with_possesion.has_value())
            {
                const Ray enemy_shot_projection =
                    Ray(enemy_with_possesion->position(),
                        Point(enemy_with_possesion->orientation().cos(),
                              enemy_with_possesion->orientation().sin()));

                auto [goal_intersection, second_intersection] =
                    raySegmentIntersection(enemy_shot_projection, full_goal_segment);
                Point goal_intersection_adjusted = Point(field.friendlyGoal());

                // If the enemies gaze ends up in the net, position to block
                if (goal_intersection.has_value())
                {
                    if (goal_intersection.value().y() > field.enemyGoalpostPos().y())
                    {
                        goal_intersection_adjusted =
                            field.enemyGoalpostPos() + Point(0, -ROBOT_MAX_RADIUS_METERS);
                    }
                    else if (goal_intersection.value().y() < field.enemyGoalpostNeg().y())
                    {
                        goal_intersection_adjusted =
                            field.enemyGoalpostPos() + Point(0, +ROBOT_MAX_RADIUS_METERS);
                    }
                    else
                    {
                        goal_intersection_adjusted = goal_intersection.value();
                    }
                }

                else
                {
                    if (enemy_with_possesion->orientation().sin() > 0)
                    {
                        goal_intersection_adjusted =
                            field.enemyGoalpostPos() - Point(0, ROBOT_MAX_RADIUS_METERS);
                    }
                    else
                    {
                        goal_intersection_adjusted =
                            field.enemyGoalpostNeg() + Point(0, ROBOT_MAX_RADIUS_METERS);
                    }
                }

                next_intent = move_action.updateStateAndGetNextIntent(
                    *robot, goal_intersection_adjusted, robot->orientation(), 0.0, false,
                    AUTOCHIP);
            }
            else
            {
                // No enemt has possession, just hang in the middle of net
                next_intent = move_action.updateStateAndGetNextIntent(
                    *robot,
                    Point(full_goal_segment.getSegStart().x(),
                          (full_goal_segment.getSegStart().y() +
                           full_goal_segment.getEnd().y()) /
                              2),
                    robot->orientation(), 0.0, false, AUTOCHIP);
            }
        }



        yield(std::move(next_intent));

    } while (!move_action.done());
}
