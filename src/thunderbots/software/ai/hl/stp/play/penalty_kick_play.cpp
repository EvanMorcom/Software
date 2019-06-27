#include "ai/hl/stp/play/penalty_kick_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/penalty_kick.h"

const std::string PenaltyKickPlay::name = "Penalty Kick Play";

std::string PenaltyKickPlay::getName() const
{
    return PenaltyKickPlay::name;
}

bool PenaltyKickPlay::isApplicable(const World &world) const
{
    return true;
}

bool PenaltyKickPlay::invariantHolds(const World &world) const
{
    return true;
}

void PenaltyKickPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    // Create MoveTactics that will loop forever
    auto penalty_shot_tactic = std::make_shared<PenaltyKickTactic>(world.ball(), world.field(), world.enemyTeam().goalie().value());

    auto move_tactic_2 = std::make_shared<MoveTactic>(true);
    auto move_tactic_3 = std::make_shared<MoveTactic>(true);
    auto move_tactic_4 = std::make_shared<MoveTactic>(true);
    auto move_tactic_5 = std::make_shared<MoveTactic>(true);
    auto move_tactic_6 = std::make_shared<MoveTactic>(true);

    do
    {
        // The angle between each robot spaced out in a circle around the ball
        Angle angle_between_robots = Angle::full() / world.friendlyTeam().numRobots();

        // Move the robots in a circle around the ball, facing the ball
        penalty_shot_tactic->updateParams(world.ball(), world.enemyTeam().goalie().value(), world.field());

        move_tactic_2->updateParams(
                world.ball().position() + Point::createFromAngle(angle_between_robots * 2),
                (angle_between_robots * 2) + Angle::half(), 0);
        move_tactic_3->updateParams(
                world.ball().position() + Point::createFromAngle(angle_between_robots * 3),
                (angle_between_robots * 3) + Angle::half(), 0);
        move_tactic_4->updateParams(
                world.ball().position() + Point::createFromAngle(angle_between_robots * 4),
                (angle_between_robots * 4) + Angle::half(), 0);
        move_tactic_5->updateParams(
                world.ball().position() + Point::createFromAngle(angle_between_robots * 5),
                (angle_between_robots * 5) + Angle::half(), 0);
        move_tactic_6->updateParams(
                world.ball().position() + Point::createFromAngle(angle_between_robots * 6),
                (angle_between_robots * 6) + Angle::half(), 0);

        // yield the Tactics this Play wants to run, in order of priority
        yield({penalty_shot_tactic, move_tactic_2, move_tactic_3, move_tactic_4, move_tactic_5,
               move_tactic_6});
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<PenaltyKickPlay> factory;
