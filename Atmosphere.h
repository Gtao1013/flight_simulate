/************  Atmosphere.h  ******************************
	 Atmospheric Model Related Definitions
**********************************************************/
#ifndef  _ATMOSPHERE_H_
#define  _ATMOSPHERE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Define _countof if not defined
#ifndef _countof
#define _countof(array) (sizeof(array)/sizeof(array[0]))
#endif

// Atmospheric Constants
#define pp0 101325.0    // Standard Atmospheric Pressure (Pa)
#define rho0 1.225      // Standard Atmospheric Density (kg/m³)
#define g00 9.80665     // Gravitational Acceleration (m/s²)

// Atmospheric Model Function Declarations
void InputData_Atmo();
double GetRho_H(double h);
double GetP_H(double h);
double GetT_H(double h);
double GetSpeedofSound(double t);

#endif