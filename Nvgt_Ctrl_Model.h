/************  Nvgt_Ctrl_Model.h  ******************************
     Navigation and Control Model
**********************************************************/

#ifndef  _NVGT_CTRL_MODEL_H_
#define  _NVGT_CTRL_MODEL_H_

#include "Data_Parameter.h"
#include "MotiModel.h"

class CMassModel;
class CDynaModel;
class CEngiModel;
class Data_Parameter;
class CMotiModel;


double LAQL1(int n, double* x, double* y, double u);
double Sign1(double x);
double Limit(double Xi, double Xmax);
double Get_Atmo_pp(double h);
double Get_Atmo_tt(double h);
double Get_Atmo_aa(double h);
double Get_Atmo_rr(double pp, double tt);
double Get_Atmo_windx(double h);
double Get_Atmo_windz(double h);

class CNCModel
{
public:
    int Num_Ctrl = 100;	
    double data_t_NC[100], data_alfa_NC[100];	// Time and control angle arrays
    double t, alfa_NC, t_NC_begin, t_NC_end;
    // Trajectory pitch angle control variables
    double theta_target;   // Target trajectory pitch angle (Theta, not THETA)
    bool is_theta_control; // Control mode flag: true for trajectory pitch angle control, false for angle of attack control
    
    // PID controller gains and state for trajectory pitch control
    double Kp_theta;        // Proportional gain
    double Ki_theta;        // Integral gain
    double Kd_theta;        // Derivative gain
    double theta_error_sum; // Error integral
    double prev_theta_error; // Previous error for derivative calculation
    
    // Flight phase enumeration
    enum FlightPhase {
        TAKEOFF = 0,    // Takeoff phase (30° pitch)
        CLIMB = 1,      // Climbing phase (6° pitch)
        CRUISE = 2,     // Cruising phase (0° pitch)
        DESCENT = 3,    // Descent phase (-2° pitch)
        TERMINAL = 4    // Terminal phase (0° pitch)
    };

    // Current flight phase
    FlightPhase current_phase;

    // Phase target parameters
    double climb_target_height;    // Climbing target height (m)
    double descent_target_height;  // Descent target height (m)
    double climb_rate;             // Target climb rate (m/s)
    double target_speed;           // Target speed (m/s)
    
    // Terminal cruise mode variables
    bool terminal_cruise_enabled;     // Whether to enable terminal cruise mode
    double terminal_cruise_duration;  // Terminal cruise duration counter
    double terminal_phase_start_time; // Terminal phase start time

public:
    CNCModel();
    ~CNCModel();
    void InputData_NC();
    void Get_Alfa_Ctrl(Data_Parameter* Gdata, CMotiModel* mo);
    // Trajectory pitch angle control function (controls Theta, not THETA)
    void Get_Theta_Ctrl(Data_Parameter* Gdata, CMotiModel* mo);
    
    // Flight phase management functions
    void UpdateFlightPhase(CMotiModel* mo);
    void AdjustControlForPhase(Data_Parameter* Gdata, CMotiModel* mo);
};
#endif