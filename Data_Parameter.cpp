#define _CRT_SECURE_NO_WARNINGS
#include "Data_Parameter.h"
#include <string.h>  // For string operations
#include <stdlib.h>  // For stdlib functions
#include <direct.h>  // For directory operations
#include <iostream>
#include <cmath>     // For isfinite function
using namespace std;

// Define _countof if not already defined
#ifndef _countof
#define _countof(array) (sizeof(array)/sizeof(array[0]))
#endif

// Function to try opening a file with different path combinations
static FILE* tryOpenFile(const char* fileName) {
    FILE* file = NULL;
    char pathBuffer[512];
    
    cout << "Trying to open file: " << fileName << endl;
    
    // Method 1: Direct filename
    if ((file = fopen(fileName, "r")) != NULL) {
        cout << "Successfully opened file: " << fileName << endl;
        return file;
    }
    cout << "  Path 1 failed: " << fileName << endl;
    
    // Method 2: Using Input/format
    sprintf(pathBuffer, "Input/%s", fileName);
    if ((file = fopen(pathBuffer, "r")) != NULL) {
        cout << "Successfully opened file: " << pathBuffer << endl;
        return file;
    }
    cout << "  Path 2 failed: " << pathBuffer << endl;
    
    // Method 3: Using ./Input/format
    sprintf(pathBuffer, "./Input/%s", fileName);
    if ((file = fopen(pathBuffer, "r")) != NULL) {
        cout << "Successfully opened file: " << pathBuffer << endl;
        return file;
    }
    cout << "  Path 3 failed: " << pathBuffer << endl;
    
    // Method 4: Using ../Input/format
    sprintf(pathBuffer, "../Input/%s", fileName);
    if ((file = fopen(pathBuffer, "r")) != NULL) {
        cout << "Successfully opened file: " << pathBuffer << endl;
        return file;
    }
    cout << "  Path 4 failed: " << pathBuffer << endl;
    
    // Method 5: Using ../../Input/format (for x64/Debug subdirectory)
    sprintf(pathBuffer, "../../Input/%s", fileName);
    if ((file = fopen(pathBuffer, "r")) != NULL) {
        cout << "Successfully opened file: " << pathBuffer << endl;
        return file;
    }
    cout << "  Path 5 failed: " << pathBuffer << endl;
    
    // Method 6: Current working directory
    char currentDir[512];
    if (_getcwd(currentDir, sizeof(currentDir))) {
        cout << "Current working directory: " << currentDir << endl;
        sprintf(pathBuffer, "%s/Input/%s", currentDir, fileName);
        if ((file = fopen(pathBuffer, "r")) != NULL) {
            cout << "Successfully opened file: " << pathBuffer << endl;
            return file;
        }
        cout << "  Path 6 failed: " << pathBuffer << endl;
        
        // Method 7: Parent directory
        char* lastSlash = strrchr(currentDir, '\\');
        if (lastSlash) {
            *lastSlash = '\0';  // Truncate string to get parent directory
            sprintf(pathBuffer, "%s/Input/%s", currentDir, fileName);
            if ((file = fopen(pathBuffer, "r")) != NULL) {
                cout << "Successfully opened file: " << pathBuffer << endl;
                return file;
            }
            cout << "  Path 7 failed: " << pathBuffer << endl;
        }
    }
    
    cout << "Could not open file: " << fileName << ", please check if it exists and has read permissions" << endl;
    cout << "Press Enter to continue..." << endl;
    getchar();
    return NULL;
}

// Initialize with sensible defaults
void Data_Parameter::InitializeDefaultValues() {
    // Set default values for all parameters
    Katmo = 1;           // Use Atmosphere.dat by default
    G_m0 = 190.0;        // Default mass
    J_x = 0.066;         // Default moments of inertia
    J_y = 0.308;
    J_z = 0.359;
    J_xy = -0.016;
    Sm = 2.265;          // Default reference area
    Lk = 0.333;          // Default reference length
    x_cg = 0.58;         // Default CG position
    
    flag_Fp = 1;         // Use engine model
    dm_dt = 0.01;        // Default fuel consumption rate
    
    flag_NC = 1;         // Default control mode
    
    X0 = 0.0;            // Initial position
    Y0 = 450.0;
    Z0 = 0.0;
    
    Ma = 0.118;          // Initial Mach number
    THETA0 = 30.0 * DtR; // Convert degrees to radians
    Psi0 = 0.0;
    Gama0 = 0.0;
    Wx0 = 0.0;
    Wy0 = 0.0;
    Wz0 = 0.0;
    alpha0 = 0.0;
    beta0 = 0.0;
    
    Theta0 = THETA0 - alpha0;
    Psi_v0 = Psi0 - beta0;
    
    h = 0.01;            // Default time step
    h_out = 1.0;         // Default output interval
    
    flag_1 = 1;
    flag_2 = 2;          // Default output format (Tecplot)
    flag_dy = 1;         // Default output content
    
    flag_3 = 4;          // Default termination variable (time)
    flag_4 = 1;          // Default termination logic (greater than)
    H_end = 28800.0;     // Default end value (8 hours in seconds)
    
    V0 = 40.0;           // Default initial velocity
}

Data_Parameter::Data_Parameter() {
    // Initialize all variables with sensible defaults
    InitializeDefaultValues();
}

Data_Parameter::~Data_Parameter() {
    // Nothing to clean up
}

int Data_Parameter::InputData() {
    FILE* input;
    char str[256]; // Larger buffer for reading strings
    
    // Initialize with defaults before reading file
    InitializeDefaultValues();

    // Try to open the data file
    if ((input = tryOpenFile("Data.dat")) == NULL) {
        cout << "Cannot open input file Data.dat, using default values" << endl;
        return -1;
    }

    try {
        // Read atmosphere parameters section
        fscanf(input, "%255s", str);  // AtmosphereParameters
        fscanf(input, "%255s", str);  // katmo description
        if (fscanf(input, "%d", &Katmo) != 1) {
            cout << "Warning: Failed to read Katmo, using default value: " << Katmo << endl;
        }

        // Read structure parameters section
        fscanf(input, "%255s", str);  // StructureParameters
        fscanf(input, "%255s", str);  // InitialMass
        if (fscanf(input, "%lf", &G_m0) != 1 || G_m0 <= 0 || !isfinite(G_m0)) {
            cout << "Warning: Invalid G_m0 value, using default: " << G_m0 << endl;
        }
        fscanf(input, "%255s", str);  // UnitMassInertialMomentJx
        if (fscanf(input, "%lf", &J_x) != 1 || !isfinite(J_x)) {
            cout << "Warning: Invalid J_x value, using default: " << J_x << endl;
        }
        fscanf(input, "%255s", str);  // UnitMassInertialMomentJy
        if (fscanf(input, "%lf", &J_y) != 1 || !isfinite(J_y)) {
            cout << "Warning: Invalid J_y value, using default: " << J_y << endl;
        }
        fscanf(input, "%255s", str);  // UnitMassInertialMomentJz
        if (fscanf(input, "%lf", &J_z) != 1 || !isfinite(J_z)) {
            cout << "Warning: Invalid J_z value, using default: " << J_z << endl;
        }
        fscanf(input, "%255s", str);  // UnitMassInertialMomentJxy
        if (fscanf(input, "%lf", &J_xy) != 1 || !isfinite(J_xy)) {
            cout << "Warning: Invalid J_xy value, using default: " << J_xy << endl;
        }
        fscanf(input, "%255s", str);  // ReferenceSurfaceArea
        if (fscanf(input, "%lf", &Sm) != 1 || Sm <= 0 || !isfinite(Sm)) {
            cout << "Warning: Invalid Sm value, using default: " << Sm << endl;
        }
        fscanf(input, "%255s", str);  // ReferenceLength
        if (fscanf(input, "%lf", &Lk) != 1 || Lk <= 0 || !isfinite(Lk)) {
            cout << "Warning: Invalid Lk value, using default: " << Lk << endl;
        }
        fscanf(input, "%255s", str);  // CGRelativePosition
        if (fscanf(input, "%lf", &x_cg) != 1 || !isfinite(x_cg)) {
            cout << "Warning: Invalid x_cg value, using default: " << x_cg << endl;
        }

        // Read engine parameters section
        fscanf(input, "%255s", str);  // EngineParameters
        fscanf(input, "%255s", str);  // ConsiderThrust
        if (fscanf(input, "%lf", &flag_Fp) != 1 || (flag_Fp != 0 && flag_Fp != 1) || !isfinite(flag_Fp)) {
            cout << "Warning: Invalid flag_Fp value, using default: " << flag_Fp << endl;
        }
        fscanf(input, "%255s", str);  // FuelConsumptionRate
        if (fscanf(input, "%lf", &dm_dt) != 1 || dm_dt < 0 || !isfinite(dm_dt)) {
            cout << "Warning: Invalid dm_dt value, using default: " << dm_dt << endl;
        }

        // Read guidance control parameters section
        fscanf(input, "%255s", str);  // GuidanceControlParameters
        fscanf(input, "%255s", str);  // ControlMode
        if (fscanf(input, "%d", &flag_NC) != 1 || (flag_NC != 0 && flag_NC != 1) || !isfinite(flag_NC)) {
            cout << "Warning: Invalid flag_NC value, using default: " << flag_NC << endl;
        }

        // Read launch parameters section
        fscanf(input, "%255s", str);  // LaunchParameters
        fscanf(input, "%255s", str);  // InitialPositionX
        if (fscanf(input, "%lf", &X0) != 1 || !isfinite(X0)) {
            cout << "Warning: Invalid X0 value, using default: " << X0 << endl;
        }
        fscanf(input, "%255s", str);  // InitialPositionY
        if (fscanf(input, "%lf", &Y0) != 1 || !isfinite(Y0)) {
            cout << "Warning: Invalid Y0 value, using default: " << Y0 << endl;
        }
        fscanf(input, "%255s", str);  // InitialPositionZ
        if (fscanf(input, "%lf", &Z0) != 1 || !isfinite(Z0)) {
            cout << "Warning: Invalid Z0 value, using default: " << Z0 << endl;
        }
        fscanf(input, "%255s", str);  // MachNumber
        if (fscanf(input, "%lf", &Ma) != 1 || Ma < 0 || !isfinite(Ma)) {
            cout << "Warning: Invalid Ma value, using default: " << Ma << endl;
        }
        fscanf(input, "%255s", str);  // PitchAngle
        if (fscanf(input, "%lf", &THETA0) != 1 || !isfinite(THETA0)) {
            cout << "Warning: Invalid THETA0 value, using default: " << THETA0 << endl;
        }
        fscanf(input, "%255s", str);  // YawAngle
        if (fscanf(input, "%lf", &Psi0) != 1 || !isfinite(Psi0)) {
            cout << "Warning: Invalid Psi0 value, using default: " << Psi0 << endl;
        }
        fscanf(input, "%255s", str);  // RollAngle
        if (fscanf(input, "%lf", &Gama0) != 1 || !isfinite(Gama0)) {
            cout << "Warning: Invalid Gama0 value, using default: " << Gama0 << endl;
        }
        fscanf(input, "%255s", str);  // RollAngularRate
        if (fscanf(input, "%lf", &Wx0) != 1 || !isfinite(Wx0)) {
            cout << "Warning: Invalid Wx0 value, using default: " << Wx0 << endl;
        }
        fscanf(input, "%255s", str);  // YawAngularRate
        if (fscanf(input, "%lf", &Wy0) != 1 || !isfinite(Wy0)) {
            cout << "Warning: Invalid Wy0 value, using default: " << Wy0 << endl;
        }
        fscanf(input, "%255s", str);  // PitchAngularRate
        if (fscanf(input, "%lf", &Wz0) != 1 || !isfinite(Wz0)) {
            cout << "Warning: Invalid Wz0 value, using default: " << Wz0 << endl;
        }
        fscanf(input, "%255s", str);  // AttackAngle
        if (fscanf(input, "%lf", &alpha0) != 1 || !isfinite(alpha0)) {
            cout << "Warning: Invalid alpha0 value, using default: " << alpha0 << endl;
        }
        fscanf(input, "%255s", str);  // SideslipAngle
        if (fscanf(input, "%lf", &beta0) != 1 || !isfinite(beta0)) {
            cout << "Warning: Invalid beta0 value, using default: " << beta0 << endl;
        }

        // Convert angles from degrees to radians
        THETA0 = THETA0 * DtR;
        Psi0 = Psi0 * DtR;
        Gama0 = Gama0 * DtR;
        Wx0 = Wx0 * DtR;
        Wy0 = Wy0 * DtR;
        Wz0 = Wz0 * DtR;
        alpha0 = alpha0 * DtR;
        beta0 = beta0 * DtR;

        Theta0 = THETA0 - alpha0;
        Psi_v0 = Psi0 - beta0;

        // Read calculation parameters section
        fscanf(input, "%255s", str);  // CalculationParameters
        fscanf(input, "%255s", str);  // TimeStep
        if (fscanf(input, "%lf", &h) != 1 || h <= 0 || !isfinite(h)) {
            cout << "Warning: Invalid time step h, using default: 0.01" << endl;
            h = 0.01;
        }
        fscanf(input, "%255s", str);  // OutputTimeInterval
        if (fscanf(input, "%lf", &h_out) != 1 || h_out <= 0 || !isfinite(h_out)) {
            cout << "Warning: Invalid output time interval h_out, using default: 1.0" << endl;
            h_out = 1.0;
        }
        fscanf(input, "%255s", str);  // AerodynamicDataCoordinateSystem
        if (fscanf(input, "%d", &flag_2) != 1 || (flag_2 != 1 && flag_2 != 2) || !isfinite(flag_2)) {
            cout << "Warning: Invalid flag_2 value, using default: " << flag_2 << endl;
        }
        fscanf(input, "%255s", str);  // OutputFileFormat
        if (fscanf(input, "%d", &flag_dy) != 1 || flag_dy < 1 || flag_dy > 3 || !isfinite(flag_dy)) {
            cout << "Warning: Invalid flag_dy value, using default: " << flag_dy << endl;
        }

        // Read termination conditions section
        fscanf(input, "%255s", str);  // TerminationConditions
        fscanf(input, "%255s", str);  // Example line
        fscanf(input, "%255s", str);  // TerminationVariable
        if (fscanf(input, "%d", &flag_3) != 1 || flag_3 < 1 || flag_3 > 4 || !isfinite(flag_3)) {
            cout << "Warning: Invalid flag_3 value, using default: " << flag_3 << endl;
        }
        fscanf(input, "%255s", str);  // CompareLogic
        if (fscanf(input, "%d", &flag_4) != 1 || flag_4 < 1 || flag_4 > 3 || !isfinite(flag_4)) {
            cout << "Warning: Invalid flag_4 value, using default: " << flag_4 << endl;
        }
        fscanf(input, "%255s", str);  // Value
        if (fscanf(input, "%lf", &H_end) != 1 || !isfinite(H_end)) {
            cout << "Warning: Invalid H_end value, using default: " << H_end << endl;
        }

        fclose(input);
        cout << "Successfully read parameters from Data.dat" << endl;
        
        // Final validation
        if (!isfinite(G_m0) || G_m0 <= 0) {
            cout << "Error: Invalid mass value, reset to default" << endl;
            G_m0 = 190.0;
        }
        
        if (!isfinite(h) || h <= 0) {
            cout << "Warning: Initial time step invalid, set to default value 0.01" << endl;
            h = 0.01;
        }

        return 2;
    }
    catch (const std::exception& e) {
        if (input) fclose(input);
        cout << "Exception during parameters reading: " << e.what() << endl;
        cout << "Using default parameter values" << endl;
        return -1;
    }
}
