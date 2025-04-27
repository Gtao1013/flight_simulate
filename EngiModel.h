/************  EngModel.h  ******************************
     Engine Model
**********************************************************/

#ifndef  _C_ENG_MAS_DYN_MODEL_H_
#define  _C_ENG_MAS_DYN_MODEL_H_

#include "stdio.h"
#include "stdlib.h"
#include "MotiModel.h"
#include "Global_Constant.h"
#include "Data_Parameter.h"


class Data_Parameter;
class CMotiModel;

class CEngiModel
{
public:
    double Fp;             // Thrust magnitude
    double dm_dt_current;  // Current mass consumption rate (g/s)
    double rpm_current;    // Current RPM
    
    // Compatible with original time sequence thrust data
    int Num_Fp;
    double data_t_Fp[100], data_Fp[100];	// Time and thrust arrays
    double t, t_Fp_begin, t_Fp_end;
    
    // New 3D thrust data table
    int Num_H_Fp;         // Number of altitude points
    int Num_V_Fp;         // Number of velocity points
    int Num_RPM_Fp;       // Number of RPM points
    
    double* data_H_Fp;    // Altitude array
    double* data_V_Fp;    // Velocity array
    double* data_RPM_Fp;  // RPM array
    double* data_Fp_3d;   // Thrust array (3D)
    double* data_dm_dt;   // Mass consumption rate array (3D)
    
    // Temporary variables for reading 2D thrust table
    int Num_Engi;         // Actual number of data points read
    double* data_h;       // Height array
    double* data_v;       // Velocity array
    double* data_rpm;     // RPM array
    double* data_f;       // Thrust array
    double* data_dm;      // Mass consumption rate array

    int use_rpm_model;    // Flag for using variable RPM thrust model: 0-use time sequence, 1-use 3D interpolation

public:
    CEngiModel();
    ~CEngiModel();

    void InputData_En();
    void Get_Fp(Data_Parameter* Gdata, CMotiModel* mo);
    
    // 3D interpolation function
    double LAQL3_Fp(int l, int m, int n, double* data_l, double* data_m, double* data_n, 
                   double* data_C, double x, double y, double z);
                   
    // Set current RPM
    void SetRPM(double rpm);
    
    // Get current mass consumption rate (kg/s)
    double GetMassRate() const { return dm_dt_current / 1000.0; } // Convert to kg/s
    
    // Read engine data from file
    bool readEngineDataFromFile();
};


#endif





























