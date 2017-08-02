#include "ros/ros.h"
#include "ros/console.h"

#include "amdc.h"
#include <Eigen/Dense>
#include "controller.h"
#include "debris_thresholds.h"

#include <string>

using namespace Eigen;
using namespace controller;

// squared distance in which we consider pose is near goal
static const float dist_eps = .5;

static const float full_pwm = 230;
static const float open_door_pwm = 0;
static const float close_door_pwm = 100;

// arbitrary number of random goals while finding for debris
static const int NUM_OF_RANDOM_GOALS = 5;
// spread of the distance between random goals and pose
static const float SPREAD = .5;

static ros::Time timer;
static ros::Duration close_door_duration(1);
static ros::Duration open_door_duration(1);
static ros::Duration close_door_delay(2);

enum sm_states 
{
    sm_idle,
    sm_go2goal,
    sm_find_debris,
    sm_go2debris,
    sm_opening_door,
    sm_opened_door,
    sm_delay_before_closing_door,
    sm_closing_door,
    sm_invalid
};

enum sm_states idle();
enum sm_states go2goal();
enum sm_states find_debris();
enum sm_states go2debris();
enum sm_states opening_door();
enum sm_states opened_door();
enum sm_states delay_before_closing_door();
enum sm_states closing_door();

enum sm_states (*sm_func[8])() =
{
    idle,
    go2goal,
    find_debris,
    go2debris,
    opening_door,
    opened_door,
    delay_before_closing_door,
    closing_door
};

static enum sm_states state = sm_idle;

enum sm_states idle()
{
    if (amdc_s.goals.size() != 0)
        return sm_go2goal;
    return sm_idle;
}

static void update_propeller()
{
    VectorXf cmd, cmd_pwm;
    cmd = amdc_s.controller_nav.compute_u(amdc_s.state, 
            amdc_s.goals.front(), amdc_s.range);
    cmd_pwm = amdc_s.controller_nav.u2pwm(cmd);

    amdc_s.propeller_cmd.left_spd = cmd_pwm(0);
    amdc_s.propeller_cmd.right_spd = cmd_pwm(1);
    amdc_s.propeller_cmd.update = true;
}

static float dist_between(const VectorXf& x, const VectorXf& y)
{
    return (x - y).norm();
}

enum sm_states go2goal()
{
    float dist;

    update_propeller();

    dist = dist_between(amdc_s.state.head(2), amdc_s.goals.front());
    if (dist < dist_eps)
    {
        amdc_s.propeller_cmd.left_spd = 0;
        amdc_s.propeller_cmd.right_spd = 0;
        amdc_s.propeller_cmd.update = true;
        amdc_s.goals.pop();
        for (int i = 0; i < NUM_OF_RANDOM_GOALS; ++i)
            amdc_s.goals.push(VectorXf::Random(2)*SPREAD + 
                    amdc_s.state.head(2));

        return sm_find_debris;
    }
    return sm_go2goal;
}

enum sm_states find_debris()
{
    float dist;

    update_propeller();

    if (amdc_s.debris_coord(0) >= 0 &&
        amdc_s.controller_nav.getCState() != oa_state)
    {
        return sm_go2debris;
    }

    dist = dist_between(amdc_s.state.head(2), amdc_s.goals.front());
    if (dist < dist_eps)
        amdc_s.goals.pop();

    if (amdc_s.goals.size() == 0)
        return sm_idle;

    return sm_find_debris;
}

enum sm_states go2debris()
{
    if (amdc_s.debris_coord(0) < 0)
        return find_debris();

    if (in_open_door_box(amdc_s.debris_coord))
    {
        amdc_s.servo_cmd.open = true;
        amdc_s.servo_cmd.update = true;
        timer = ros::Time::now() + open_door_duration;
        return sm_opening_door;
    }

    VectorXf cmd, cmd_pwm;
    cmd = amdc_s.controller_vs.compute_u(amdc_s.state, 
            amdc_s.debris_coord, amdc_s.range);
    cmd_pwm = amdc_s.controller_vs.u2pwm(cmd);

    amdc_s.propeller_cmd.left_spd = cmd_pwm(0);
    amdc_s.propeller_cmd.right_spd = cmd_pwm(1);
    amdc_s.propeller_cmd.update = true;
    return sm_go2debris;
}

enum sm_states opening_door()
{
    if (ros::Time::now() > timer)
        return sm_opened_door;

    amdc_s.propeller_cmd.left_spd = open_door_pwm;
    amdc_s.propeller_cmd.right_spd = open_door_pwm;
    amdc_s.propeller_cmd.update = true;
    return sm_opening_door;
}

enum sm_states opened_door()
{
    VectorXf cmd, cmd_pwm;
    cmd = amdc_s.controller_vs.collect(amdc_s.state, amdc_s.range);
    cmd_pwm = amdc_s.controller_vs.u2pwm(cmd);

    bool decelerating = (cmd_pwm(0) + cmd_pwm(1)) < 0;

    // !in_stay_opened_box includes -1,-1 (i.e. no debris)
    if (in_close_door_box(amdc_s.debris_coord) ||
            !in_stay_opened_box(amdc_s.debris_coord) ||
            decelerating)
    {
        timer = ros::Time::now() + close_door_delay;
        return sm_delay_before_closing_door;
    }

    amdc_s.propeller_cmd.left_spd = cmd_pwm(0);
    amdc_s.propeller_cmd.right_spd = cmd_pwm(1);
    amdc_s.propeller_cmd.update = true;
    return sm_opened_door;
}

enum sm_states delay_before_closing_door()
{
    VectorXf cmd, cmd_pwm;
    cmd = amdc_s.controller_vs.collect(amdc_s.state, amdc_s.range);
    cmd_pwm = amdc_s.controller_vs.u2pwm(cmd);

    if (ros::Time::now() > timer)
    {
        amdc_s.servo_cmd.open = false;
        amdc_s.servo_cmd.update = true;
        timer = ros::Time::now() + close_door_duration;
        return sm_closing_door;
    }

    amdc_s.propeller_cmd.left_spd = cmd_pwm(0);
    amdc_s.propeller_cmd.right_spd = cmd_pwm(1);
    amdc_s.propeller_cmd.update = true;
    return sm_delay_before_closing_door;
}

enum sm_states closing_door()
{
    VectorXf cmd, cmd_pwm;
    cmd = amdc_s.controller_vs.collect(amdc_s.state, amdc_s.range);
    cmd_pwm = amdc_s.controller_vs.u2pwm(cmd);

    if (ros::Time::now() > timer)
        return sm_find_debris;

    amdc_s.propeller_cmd.left_spd = cmd_pwm(0);
    amdc_s.propeller_cmd.right_spd = cmd_pwm(1);
    amdc_s.propeller_cmd.update = true;
    return sm_closing_door;
}

std::string state2name[] =
{
    "sm_idle",
    "sm_go2goal",
    "sm_find_debris",
    "sm_go2debris",
    "sm_opening_door",
    "sm_opened_door",
    "sm_closing_door",
    "sm_invalid"
};

void update_state_machine()
{
    ROS_INFO_STREAM("sm " << state2name[state]);
    ROS_INFO_STREAM("state\n" << amdc_s.state);
    ROS_INFO_STREAM("range\n"
                    "     " << amdc_s.range(3) << "\n" <<
                    amdc_s.range(2) << "\t\t" << amdc_s.range(4) << "\n\n" <<
                    amdc_s.range(1) << "\t\t" << amdc_s.range(5) << "\n\n" <<
                    amdc_s.range(0) << "\t\t" << amdc_s.range(6));
    if (amdc_s.goals.size() > 0)
        ROS_INFO_STREAM("ref\n" << amdc_s.goals.front());

    state = sm_func[state]();
}



