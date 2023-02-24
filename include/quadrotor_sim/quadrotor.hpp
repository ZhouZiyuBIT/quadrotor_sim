#pragma once

#include<mutex>

extern "C"
{
    int ddyn(const double** arg, double** res, long long int* iw, double* w, int mem);
}

class PID;

//
//   T1    T3
//     \  /
//      \/
//      /\
//     /  \
//   T4    T2
//
class Quadrotor
{
public:
    double time_factor = 1.0;
    double T_min[4] = {0, 0, 0, 0};
    double T_max[4] = {4.179, 4.179, 4.179, 4.179};
    bool has_ground = true;

    Quadrotor();
    void set_position(double pos[3]);
    int start();
    void send_ctrl(double c[4]);
    void set_states_cb(void (*states_callback)(double states[13]));

private:
    double X[13];
    std::mutex X_m;
    double U[4];

    double thrust_sp = 0;
    double angular_rate_sp[3] = {0};
    std::mutex ctrl_m;

    PID* wx_pid;
    PID* wy_pid;
    PID* wz_pid;

    void low_ctrl();

    void (*states_callback)(double states[13]);

    void step_1ms();
    void step_10ms();

    static void simulation(Quadrotor* th);


};
