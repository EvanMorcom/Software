#include "software/proto/message_translation/tbots_protobuf.h"

#include <gtest/gtest.h>
#include <string.h>

#include "shared/proto/geometry.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "shared/proto/vision.pb.h"
#include "software/primitive/primitive.h"
#include "software/test_util/test_util.h"
#include "software/world/world.h"

class TbotsProtobufTest : public ::testing::Test
{
   public:
    static void assertPointMessageEqual(const Point& point, const PointMsg& point_msg)
    {
        ASSERT_TRUE(point_msg.x() == point.x());
        ASSERT_TRUE(point_msg.y() == point.y());
    }

    static void assertAngleMessageEqual(const Angle& angle, const AngleMsg& angle_msg)
    {
        ASSERT_TRUE(angle_msg.radians() == angle.toRadians());
    }

    static void assertVectorMessageEqual(const Vector& vector,
                                         const VectorMsg& vector_msg)
    {
        ASSERT_TRUE(vector_msg.x_component() == vector.x());
        ASSERT_TRUE(vector_msg.y_component() == vector.y());
    }

    static void assertBallStateMessageFromBall(const Ball& ball,
                                               const BallStateMsg& ball_state_msg)
    {
        assertPointMessageEqual(ball.position(), ball_state_msg.global_position_meters());
        assertVectorMessageEqual(ball.velocity(),
                                 ball_state_msg.global_velocity_meters_per_sec());
    }

    static void assertRobotStateMessageFromRobot(const Robot& robot,
                                                 const RobotStateMsg& robot_state_msg)
    {
        assertPointMessageEqual(robot.position(),
                                robot_state_msg.global_position_meters());
        assertAngleMessageEqual(robot.orientation(),
                                robot_state_msg.global_orientation_radians());
        assertVectorMessageEqual(robot.velocity(),
                                 robot_state_msg.global_velocity_meters_per_sec());
        assertAngleMessageEqual(
            robot.angularVelocity(),
            robot_state_msg.global_angular_velocity_radians_per_sec());
    }

    static void assertSaneTimestamp(const TimestampMsg& timestamp)
    {
        // time will only move forward
        // we make sure the number that the timestamp is from the past
        const auto clock_time = std::chrono::system_clock::now();
        double time_in_seconds =
            static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                    clock_time.time_since_epoch())
                                    .count()) /
            MICROSECONDS_PER_SECOND;
        ASSERT_TRUE(timestamp.epoch_timestamp_seconds() <= time_in_seconds);
    }
};

TEST(TbotsProtobufTest, point_msg_test)
{
    auto point     = Point(420, 420);
    auto point_msg = createPointMsg(point);

    TbotsProtobufTest::assertPointMessageEqual(point, *point_msg);
}

TEST(TbotsProtobufTest, angle_msg_test)
{
    auto angle     = Angle::fromRadians(420);
    auto angle_msg = createAngleMsg(angle);

    TbotsProtobufTest::assertAngleMessageEqual(angle, *angle_msg);
}

TEST(TbotsProtobufTest, vector_msg_test)
{
    auto vector     = Vector(420, 420);
    auto vector_msg = createVectorMsg(vector);

    TbotsProtobufTest::assertVectorMessageEqual(vector, *vector_msg);
}

TEST(TbotsProtobufTest, timestamp_msg_test)
{
    auto timestamp_msg = createCurrentTimestampMsg();
    TbotsProtobufTest::assertSaneTimestamp(*timestamp_msg);
}

TEST(TbotsProtobufTest, robot_state_msg_test)
{
    auto position         = Point(420, 420);
    auto velocity         = Vector(420, 420);
    auto orientation      = Angle::fromRadians(420);
    auto angular_velocity = Angle::fromRadians(420);

    Robot robot(0, position, velocity, orientation, angular_velocity,
                Timestamp::fromSeconds(0));
    auto robot_state_msg = createRobotStateMsg(robot);

    TbotsProtobufTest::assertRobotStateMessageFromRobot(robot, *robot_state_msg);
}

TEST(TbotsProtobufTest, ball_state_msg_test)
{
    auto position = Point(420, 420);
    auto velocity = Vector(420, 420);

    Ball ball(position, velocity, Timestamp::fromSeconds(0));
    auto ball_state_msg = createBallStateMsg(ball);

    TbotsProtobufTest::assertBallStateMessageFromBall(ball, *ball_state_msg);
}

TEST(TbotsProtobufTest, vision_msg_test)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setFriendlyRobotPositions(
        world, {Point(420, 420), Point(420, 420), Point(420, 420), Point(420, 420)},
        Timestamp::fromSeconds(0));

    auto vision_msg = createVisionMsg(world);

    TbotsProtobufTest::assertBallStateMessageFromBall(world.ball(),
                                                      vision_msg->ball_state());
    TbotsProtobufTest::assertSaneTimestamp(vision_msg->time_sent());

    auto friendly_robots   = world.friendlyTeam().getAllRobots();
    auto& robot_states_map = *vision_msg->mutable_robot_states();

    std::for_each(friendly_robots.begin(), friendly_robots.end(),
                  [&](const Robot& robot) {
                      ASSERT_TRUE(robot_states_map.count(robot.id()));
                      TbotsProtobufTest::assertRobotStateMessageFromRobot(
                          robot, robot_states_map[robot.id()]);
                  });
}
