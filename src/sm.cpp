#include "ros/ros.h"
#include "ros/console.h"

#include "amdc.h"
#include <Eigen/Dense>
#include "controller.h"

using namespace Eigen;
using namespace controller;

// squared distance in which we consider pose is near goal
static const float dist_eps = 0.2;

enum sm_states 
{
    sm_idle,
    sm_go2goal,
    sm_find_debris,
    sm_invalid
};

enum sm_states idle();
enum sm_states go2goal();

enum sm_states (*sm_func[2])() =
{
    idle,
    go2goal
};

static enum sm_states state = sm_idle;

enum sm_states idle()
{
    if (amdc.goals.size() != 0)
        return sm_go2goal;
    return sm_idle;
}

enum sm_states go2goal()
{
    VectorXf cmd;
    cmd = nav_controller(amdc.state, amdc.goals.front(), amdc.range);

    ROS_INFO_STREAM("state\n" << amdc.state);
    ROS_INFO_STREAM("range\n" << amdc.range);
    ROS_INFO_STREAM("ref\n" << amdc.goals.front());
    ROS_INFO_STREAM("propeller cmd\n" << cmd);

    amdc.propeller_cmd.left_spd = cmd(0);
    amdc.propeller_cmd.right_spd = cmd(1);
    amdc.propeller_cmd.update = true;

    float dist = (amdc.goals.front() - amdc.state.head(2)).squaredNorm();
    if (dist < dist_eps)
    {
        amdc.goals.pop();
        return sm_idle;
    }
    return sm_go2goal;
}

void update_state_machine()
{
    state = sm_func[state]();
}
