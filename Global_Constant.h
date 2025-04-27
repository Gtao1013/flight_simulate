/************  Global_Constant.h  ******************************
	 Global Constants Definition
**********************************************************/
#ifndef  _Global_Constant_H_ 
#define  _Global_Constant_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>  // Include header file needed for string operations
#include <math.h>
#include <iostream>
#include <cmath>    // Include header file needed for math functions

// Define _countof if not defined
#ifndef _countof
#define _countof(array) (sizeof(array)/sizeof(array[0]))
#endif

// Common mathematical constants
#define PI 3.14159265358979323846
#define DtR PI/180.0        // Degree to Radian conversion
#define RtD 180.0/PI        // Radian to Degree conversion
#define G0 9.80665          // Gravity acceleration (m/s^2)
#define Re 6371000.0        // Earth radius (m) 
#define Omega_e 7.292e-5    // Earth rotation angular velocity (rad/s)

using namespace std;

#endif
