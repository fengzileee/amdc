#ifndef amdc__H
#define amdc__H

#include <queue>
#include <Eigen/Dense>

struct PropellerCommand
{
    int16_t left_spd;
    int16_t right_spd;
    bool update;
};

struct ServoCommand
{
    int16_t left_angle;
    int16_t right_angle;
    bool update;
};

class Amdc
{
public:
    std::queue<Eigen::VectorXf> goals;
    Eigen::VectorXf state;
    Eigen::VectorXf range;
    Eigen::VectorXf debris_coord;
    PropellerCommand propeller_cmd;
    ServoCommand servo_cmd;
    bool remote_controlled;

    Amdc()
    {
        state.resize(6);
        range.resize(7);
        debris_coord.resize(2);
    }
};

extern Amdc amdc;

void update_state_machine();

#endif
