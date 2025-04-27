#define _CRT_SECURE_NO_WARNINGS
/************  Atmospher.cpp  ******************************
	Atmosphere Model
**********************************************************/
#include "Atmosphere.h"
#include "FileUtils.h"  // Added FileUtils.h header
#include <string.h>
#include <iostream>
#include <stdio.h>

// Define _countof if not defined
#ifndef _countof
#define _countof(array) (sizeof(array)/sizeof(array[0]))
#endif

using namespace std;

// Removed static tryOpenFile function implementation, using public function instead

int Num_Atmo1;											// Number of interpolation points
double data_H1[66], data_P1[66], data_T1[66];			// Height, pressure, temperature
double LAQL1(int n, double* x, double* y, double u);	// One-dimensional interpolation function

void InputData_Atmo()
{
	FILE* input;
	char str[80];
	
	cout << "Preparing to read atmospheric data..." << endl;
	
	// Use new file open function to try multiple paths
	if ((input = tryOpenFile("Atmosphere.dat")) == NULL)
	{
		printf("Could not open input file Atmosphere.dat\n");
		return;
	}

	// Use standard fscanf instead of fscanf_s
	try {
		cout << "Starting to read atmospheric data content..." << endl;
		
		fscanf(input, "%s", str);     // "Pressure_Temperature_At_Different_Altitudes"
		cout << "Reading atmospheric data title: " << str << endl;

		fscanf(input, "%d", &Num_Atmo1);							// Number of interpolation points
		cout << "Read atmospheric data points: " << Num_Atmo1 << endl;

		fscanf(input, "%s", str);
		fscanf(input, "%s", str);
		fscanf(input, "%s", str);
		cout << "Reading atmospheric data header complete" << endl;

		for (int i = 0;i < Num_Atmo1;i++)
			fscanf(input, "%lf%lf%lf", &data_H1[i], &data_P1[i], &data_T1[i]);
		
		cout << "Atmospheric data reading complete, successfully read " << Num_Atmo1 << " sets of data" << endl;
		fclose(input);
		cout << "Atmospheric data file closed" << endl;
	}
	catch (const std::exception& e) {
		cout << "Exception occurred while reading atmospheric data: " << e.what() << endl;
		if (input) fclose(input);
		return;
	}
	catch (...) {
		cout << "Unknown exception occurred while reading atmospheric data" << endl;
		if (input) fclose(input);
		return;
	}

	return;
}


double Get_Atmo_pp(double h)							// Get atmospheric pressure at this height
{
	double pp;

	pp = LAQL1(Num_Atmo1, data_H1, data_P1, h);			// Interpolation calculation

	return (pp);
}

double Get_Atmo_tt(double h)
{
	double tt;
	tt = LAQL1(Num_Atmo1, data_H1, data_T1, h);			// Get atmospheric temperature at this height
	return (tt);
}

double Get_Atmo_rr(double pp, double tt)
{
	double rr;
	rr = pp / 287.053 / tt;								// Get atmospheric density at this height
	return (rr);
}

double Get_Atmo_aa(double tt)
{
	double aa;
	aa = sqrt(1.4 * 287.053 * tt);						// Get speed of sound at this height
	return (aa);
}
