#define _CRT_SECURE_NO_WARNINGS
#include  "math.h"
#include  <stdio.h>
#include  <stdlib.h>
#include  <direct.h>
#include <iostream>
#include <iomanip>
#include <limits> // Added this header
using namespace std;

#include  "MotiModel.h"
#include  "MassModel.h"
#include  "Global_Constant.h"
#include  "Data_Parameter.h"
#include  "EngiModel.h"
#include  "Data_Out.h"
#include  "AeroModel.h"
#include "Nvgt_Ctrl_Model.h"
#include <windows.h>



CMassModel ma;
CDynaModel dy;
CMotiModel mo;
CDataout   out;
CEngiModel en;
CNCModel nc;


void InputData_Atmo();
void Integral_motion(CMassModel* ma, CDynaModel* dy, CEngiModel* en, CMotiModel* mo, Data_Parameter* Gdata, CNCModel* nc, double hStep);
void Integral_ode45(CMassModel* ma, CDynaModel* dy, CEngiModel* en, CMotiModel* mo, Data_Parameter* Gdata, CNCModel* nc, double hStep);
//##############################################################################################//Main Program

Data_Parameter* Gdata = new Data_Parameter;

// Check if a float value is valid
bool isValidFloat(double value) {
    return !isnan(value) && !isinf(value) &&
        value > -1e100 && value < 1e100; // Set reasonable range
}

// Reset abnormal values
void resetNumericalErrors(CMotiModel* mo, CMassModel* ma) {
    // Check and fix mass
    if (!isValidFloat(ma->m) || ma->m <= 0) {
        cout << "Abnormal mass value detected: " << ma->m << ", reset to initial mass" << endl;
        ma->m = Gdata->G_m0;
    }

    // Check and fix position and velocity
    if (!isValidFloat(mo->X) || !isValidFloat(mo->Y) || !isValidFloat(mo->Z)) {
        cout << "Abnormal position values detected, reset to initial position" << endl;
        mo->X = Gdata->X0;
        mo->Y = Gdata->Y0;
        mo->Z = Gdata->Z0;
    }

    if (!isValidFloat(mo->V) || mo->V < 0 || mo->V > 10000) {
        cout << "Abnormal velocity value detected: " << mo->V << ", reset to reasonable value" << endl;
        mo->V = Gdata->V0 > 0 ? Gdata->V0 : 100.0; // Use initial velocity or default value
    }

    // Check and fix angles and angular velocities
    if (!isValidFloat(mo->Theta) || !isValidFloat(mo->THETA) || !isValidFloat(mo->Psi)) {
        cout << "Abnormal angle values detected, reset to initial angles" << endl;
        mo->Theta = Gdata->Theta0;
        mo->THETA = Gdata->THETA0;
        mo->Psi = Gdata->Psi0;
    }
}

// Detect and report aerodynamic issues
void checkAerodynamicIssues(CMotiModel* mo, CDynaModel* dy) {
    // Check for zero or unreasonable aerodynamic coefficients
    if (fabs(dy->CD) < 1e-6 && fabs(dy->CL) < 1e-6 && fabs(dy->CZ) < 1e-6) {
        cout << "WARNING: All aerodynamic coefficients near zero at time " << mo->t << "s" << endl;
        cout << "         altitude=" << mo->Y << "m, Mach=" << mo->Ma << endl;
        cout << "         alfa=" << mo->alfa * RtD << "°, beta=" << mo->beta * RtD << "°" << endl;
    }
    
    // Check for excessive angles
    if (fabs(mo->alfa) > 30.0 * DtR) {
        cout << "WARNING: Excessive angle of attack " << mo->alfa * RtD << "° at time " << mo->t << "s" << endl;
    }
    
    if (fabs(mo->beta) > 20.0 * DtR) {
        cout << "WARNING: Excessive sideslip angle " << mo->beta * RtD << "° at time " << mo->t << "s" << endl;
    }
    
    // Check for dangerous flight conditions
    if (mo->Y < 200.0 && mo->Vy < 0) {
        cout << "DANGER: Low altitude " << mo->Y << "m with negative vertical speed " << mo->Vy << "m/s at time " << mo->t << "s" << endl;
    }
}

// 新增函数：询问用户是否继续迭代
bool askUserToContinue() {
    char input;
    cout << "\nContinue iterating? (y/n): ";
    cin >> input;
    return (input == 'y' || input == 'Y');
}

int main()  // Change void main() to int main()
{
    SetConsoleOutputCP(65001);  // 设置控制台输出为UTF-8编码
    int nnn;	 // Parameter used to determine print frequency
    double h;	//  Time step
    double h_original; // Save initial time step
    int errorFrames = 0; // Count of consecutive error frames
    const int MAX_ERROR_FRAMES = 10; // Maximum allowed consecutive error frames

    _mkdir("Output");         // Create output folder
    Gdata->InputData();      // System parameter input
    h = Gdata->h;			//  Time step
    h_original = h;          // Save initial value

    cout << "Initial time step h = " << h << endl;

    // Ensure initial time step is valid
    if (h <= 0 || !isfinite(h)) {
        cout << "Warning: Invalid initial time step, set to default value 0.01" << endl;
        h = 0.01;
        h_original = h;
        Gdata->h = h;
    }

    out.Initial(Gdata);												// Initialize input data
    out.OpenOutFiles(Gdata);										// Output data file headers

    InputData_Atmo();												// Read atmospheric data
    dy.InputData();												    // Aerodynamic parameter input
    nc.InputData_NC();											    // Navigation control parameter input
    en.InputData_En();											    // Engine parameter input

    Gdata->time = 0.0;
    //########################// System initialization
    ma.IniData(Gdata, &en, &mo);	// Mass model initialization
    mo.IniData(&ma, Gdata);	        // Initialize motion angle equations

    // Backup initial state values for recovery in case of numerical problems
    double initialMass = ma.m;
    double initialX = mo.X;
    double initialY = mo.Y;
    double initialZ = mo.Z;
    double initialV = mo.V;
    double initialTheta = mo.Theta;

    // Add thrust data log file
    FILE* thrustLog = nullptr;
    fopen_s(&thrustLog, "Output\\thrust_data.csv", "w");
    if (thrustLog) {
        fprintf(thrustLog, "Time,Phase,Height,Velocity,RPM,Thrust,Fuel_Rate,Mass\n");
    }

    nnn = 0;
    int errorCount = 0; // Track consecutive error count
    const int MAX_ERRORS = 5; // Maximum allowed consecutive errors

    //########################// Main loop
    while (Gdata->Kfly < 2 && errorFrames < MAX_ERROR_FRAMES)  // Add error frame limit
    {
        // Check h validity to prevent abnormal values
        if (h <= 0 || !isfinite(h) || fabs(h) > 1.0) {
            errorCount++;
            cout << "Abnormal time step detected h = " << h << ", reset to initial value" << endl;
            h = h_original;  // Reset to initial saved value

            // If errors occur repeatedly, stronger measures may be needed
            if (errorCount > MAX_ERRORS) {
                cout << "Multiple abnormal time steps detected, enhancing monitoring..." << endl;
                // Additional monitoring methods could be added here, but not exiting directly
            }
        }
        else {
            errorCount = 0; // Reset error count
        }

        // Save simulation state for rollback in case of problems
        double prevTime = Gdata->time;
        double prevMass = ma.m;
        double prevX = mo.X;
        double prevY = mo.Y;
        double prevZ = mo.Z;
        double prevV = mo.V;

        try {
            // Use fixed step size, not the potentially modified h
            Integral_ode45(&ma, &dy, &en, &mo, Gdata, &nc, h_original);
            mo.Motion(&ma, &en, &dy, Gdata, &nc, h_original);

            // Check if calculation results are valid
            if (!isValidFloat(mo.V) || !isValidFloat(mo.X) || !isValidFloat(mo.Y) ||
                !isValidFloat(mo.Z) || !isValidFloat(ma.m) || ma.m <= 0) {

                // Invalid results found
                errorFrames++;
                
                cout << "Abnormal values detected: " << endl;
                cout << "Position X=" << mo.X << ", Y=" << mo.Y << ", Z=" << mo.Z << endl;
                cout << "Velocity V=" << mo.V << endl;
                cout << "Mass m=" << ma.m << endl;
                
                // 当检测到异常值时询问用户是否继续
                if (!askUserToContinue()) {
                    cout << "User chose to stop iteration, program exits" << endl;
                    break; // Exit main loop
                }
                
                // 继续现有的错误处理流程...
                // Reset abnormal values
                resetNumericalErrors(&mo, &ma);

                // If errors occur repeatedly, consider rolling back to previous stable state or initial state
                if (errorFrames > MAX_ERROR_FRAMES / 2) {
                    cout << "Repeated calculation errors, rolling back to initial state" << endl;
                    ma.m = initialMass;
                    mo.X = initialX;
                    mo.Y = initialY;
                    mo.Z = initialZ;
                    mo.V = initialV;
                    mo.Theta = initialTheta;
                }
            }
            else {
                // Results valid, reset error frame count
                errorFrames = 0;
            }
        }
        catch (const std::exception& e) {
            cout << "Exception caught during calculation: " << e.what() << endl;
            errorFrames++;

            // Restore to previous valid state
            Gdata->time = prevTime;
            ma.m = prevMass;
            mo.X = prevX;
            mo.Y = prevY;
            mo.Z = prevZ;
            mo.V = prevV;
        }

        if (nnn++ % 10000 == 0)
            nnn = 1;

        // Use fixed output frequency, not dependent on potentially unstable h value
        if (nnn % 10 == 0 && errorFrames == 0) { // Output only when no errors
            out.Outputdefined(Gdata, &dy, &ma, &en, &mo);
        }

        if (nnn % 100 == 0)	// Print information every 100 steps
        {
            cout << "Time = " << fixed << setprecision(3) << Gdata->time << ",  V  = " << fixed << setprecision(6) << mo.V << ",  Theta =" << mo.Theta * RtD;
            cout << " ,  X = " << fixed << setprecision(6) << mo.X << ",  Y = " << mo.Y << ",  Z = " << mo.Z << endl;
            cout << "Mass = " << ma.m << " kg" << endl; // Add mass output for debugging
        }

        // Record thrust data only when values are valid
        if (thrustLog && Gdata->flag_Fp == 1 && errorFrames == 0 &&
            isValidFloat(mo.Y) && isValidFloat(mo.V) && isValidFloat(ma.m)) {
            fprintf(thrustLog, "%.2f,%d,%.2f,%.2f,%.1f,%.2f,%.4f,%.2f\n",
                mo.t, nc.current_phase, mo.Y, mo.V,
                en.rpm_current, en.Fp, en.dm_dt_current, ma.m);
        }
    }

    // If exiting loop due to too many errors
    if (errorFrames >= MAX_ERROR_FRAMES) {
        cout << "\nProgram terminated early due to persistent numerical calculation errors. Please check model parameters and calculation step size.\n" << endl;
    }

    // Close thrust data log file
    if (thrustLog) {
        fclose(thrustLog);
        cout << "Thrust data saved to: Output\\thrust_data.csv" << endl;
    }

    getchar();				// Prevent CMD window from closing after program ends
    return 0;
}

// void Integral_motion(CMassModel* ma, CDynaModel* dy, CEngiModel* en, CMotiModel* mo, Data_Parameter* Gdata, CNCModel* nc, double hStep)
// {
//     // Existing code
//     // ...
//     
//     // Add checks for numerical stability
//     resetNumericalErrors(mo, ma);
//     checkAerodynamicIssues(mo, dy);
//     
//     // Every 10 steps, print detailed diagnostic information
//     static int step_counter = 0;
//     if (++step_counter % 10 == 0) {
//         cout << "===== DIAGNOSTIC AT t=" << mo->t << "s =====" << endl;
//         cout << "Position: X=" << mo->X << "m, Y=" << mo->Y << "m, Z=" << mo->Z << "m" << endl;
//         cout << "Velocity: V=" << mo->V << "m/s, Vx=" << mo->Vx << "m/s, Vy=" << mo->Vy << "m/s, Vz=" << mo->Vz << "m/s" << endl;
//         cout << "Angles: Theta=" << mo->Theta * RtD << "°, THETA=" << mo->THETA * RtD << "°, alfa=" << mo->alfa * RtD << "°" << endl;
//         cout << "Aero: CL=" << mo->dy_CL << ", CD=" << mo->dy_CD << ", CZ=" << mo->dy_CZ << endl;
//         cout << "Forces: Lift=" << mo->Ry3 << "N, Drag=" << -mo->Rx3 << "N, Thrust=" << en->Fp << "N" << endl;
//         cout << "===========================" << endl;
//     }
//     
//     // Continue with the rest of the function
//     // ...
// }
