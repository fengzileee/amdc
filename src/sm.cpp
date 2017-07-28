#include "ros/ros.h"
#include "ros/console.h"

#include "amdc.h"
#include <Eigen/Dense>
#include "controller.h"

#include <string>

using namespace Eigen;
using namespace controller;

// squared distance in which we consider pose is near goal
static const float dist_eps = .5;

// conversion factor from force to pwm
static const float force2pwm = 230;

// arbitrary number of random goals while finding for debris
static const int NUM_OF_RANDOM_GOALS = 5;
// spread of the distance between random goals and pose
static const int SPREAD = 3;

enum sm_states 
{
    sm_idle,
    sm_go2goal,
    sm_find_debris,
    sm_go2debris,
    sm_invalid
};

enum sm_states idle();
enum sm_states go2goal();
enum sm_states find_debris();
enum sm_states go2debris();

enum sm_states (*sm_func[4])() =
{
    idle,
    go2goal,
    find_debris,
    go2debris
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

    if (amdc_s.debris_coord.size() > 0)
        return sm_go2debris;

    dist = dist_between(amdc_s.state.head(2), amdc_s.goals.front());
    if (dist < dist_eps)
        amdc_s.goals.pop();

    return sm_find_debris;
}

enum sm_states go2debris()
{
    return sm_go2debris;
}

std::string state2name[] =
{
    "sm_idle",
    "sm_go2goal",
    "sm_find_debris",
    "sm_go2debris",
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



