#include"quadrotor_sim/quadrotor.hpp"

#include<thread>
#include<unistd.h>

#include<iostream>
#include<cmath>

class PID
{
public:
    PID(): _init(0), _e_last(0){};
    void setParameters(float P, float I, float D, float I_lim, float Out_lim)
    {
        _P = P;
        _I = I;
        _D = D;
        _I_lim = I_lim;
        _Out_lim = Out_lim;

    }

    float update(float e)
    {
        _init += e;
        if(_init>_I_lim) _init = _I_lim;
        else if(_init< -_I_lim) _init = -_I_lim;

        float out = _P*e + _I*_init + _D*(e-_e_last);
        _e_last = e;
        if(out > _Out_lim) out = _Out_lim;
        else if(out < -_Out_lim) out = -_Out_lim;

        return out;
    }

private:
    float _P, _I, _D;
    float _I_lim, _Out_lim;
    float _e_last;
    float _init;
};

double inline constrain(double x, double lb, double ub)
{
    if(x<lb) x = lb;
    if(x>ub) x = ub;
    return x;
}

Quadrotor::Quadrotor()
{
    for(int i=0; i<13; i++)
    {
        X[i] = 0;
    }
    X[6] = 1; //qw
    for(int i=4; i<4; i++)
    {
        U[i] = 0;
    }
    states_callback = nullptr;

    wx_pid = new PID();
    wy_pid = new PID();
    wz_pid = new PID();

    wx_pid->setParameters(15, 0.2, 0, 3, 5);
    wy_pid->setParameters(15, 0.2, 0, 3, 5);
    wz_pid->setParameters(20, 0.3, 0, 3, 5);
}

int Quadrotor::start()
{
    std::thread t(Quadrotor::simulation, this);
    t.detach();
    return 1;
}

void Quadrotor::set_states_cb(void (*states_cb)(double states[13]))
{
    states_callback = states_cb;
}

void Quadrotor::send_ctrl(double c[4])
{
    std::lock_guard<std::mutex> U_lg(ctrl_m);
    thrust_sp = c[0];
    angular_rate_sp[0] = c[1];
    angular_rate_sp[1] = c[2];
    angular_rate_sp[2] = c[3];
}

void Quadrotor::low_ctrl()
{
    double Tx, Ty, Tz;
    double thrust_sp_, angular_rate_sp_[3];
    {
        std::lock_guard<std::mutex> ctrl_lg(ctrl_m);
        thrust_sp_ = thrust_sp;
        angular_rate_sp_[0] = angular_rate_sp[0];
        angular_rate_sp_[1] = angular_rate_sp[1];
        angular_rate_sp_[2] = angular_rate_sp[2];
    }

    double angular_rate_[3] = {X[10], X[11], X[12]};
    
    Tx = wx_pid->update(angular_rate_sp_[0]-angular_rate_[0]);
    Ty = wy_pid->update(angular_rate_sp_[1]-angular_rate_[1]);
    Tz = wz_pid->update(angular_rate_sp_[2]-angular_rate_[2]);

    double T1 = thrust_sp_ + Tx + Ty - Tz;
    double T2 = thrust_sp_ - Tx - Ty - Tz;
    double T3 = thrust_sp_ - Tx + Ty + Tz;
    double T4 = thrust_sp_ + Tx - Ty + Tz;

    U[0] = constrain(T1, T_min[0], T_max[0]);
    U[1] = constrain(T2, T_min[1], T_max[1]);
    U[2] = constrain(T3, T_min[2], T_max[2]);
    U[3] = constrain(T4, T_min[3], T_max[3]);
}

void Quadrotor::step_1ms()
{
    double X1[13];
    
    {
        std::lock_guard<std::mutex> X_lg(X_m);
        const double* arg[2] = {X, U};
        double* res[1] = {X1};

        F(arg, res, nullptr, nullptr, 0);
        
        //quaternion normalization
        double q_norm = sqrt( X1[6]*X1[6]+X1[7]*X1[7]+X1[8]*X1[8]+X1[9]*X1[9] );
        X1[6] = X1[6]/q_norm;
        X1[7] = X1[7]/q_norm;
        X1[8] = X1[8]/q_norm;
        X1[9] = X1[9]/q_norm;

        if(has_ground)
        {
            if(X1[2]>0) X1[2]=0;
        }

        for(int i=0; i<13; i++)
        {
            X[i] = X1[i];
        }
    }

}

void Quadrotor::step_10ms()
{
    if(states_callback != nullptr)
    {
        (*states_callback)(X);
    }
}

void Quadrotor::simulation(Quadrotor* th)
{
    long long int cnt_1ms = 0;
    while(1)
    {
        th->step_1ms();
        th->low_ctrl();

        if(cnt_1ms%10==0)
        {
            th->step_10ms();
        }

        usleep((1000/th->time_factor));
        cnt_1ms++;
        // std::cout << "h" << std::endl;
    }
}
