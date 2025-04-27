#include "math.h"
#include <stdio.h>   // For error messages
#include <iostream>  // For error messages

using namespace std;

/************************************************
              One-dimensional Interpolation Subroutine
************************************************/
double LAQL1(int m, double* data_m, double* data_C, double x)
{
	// Check for null pointers or invalid array size
	if (data_m == nullptr || data_C == nullptr || m <= 1) {
		cout << "Error in LAQL1: Invalid input parameters" << endl;
		return 0.0;
	}

	int i;
	long double v, vary;
	
	// Value limiting
	if (x > data_m[m - 1])
		x = data_m[m - 1];
	else if (x < data_m[0])
		x = data_m[0];
		
	// Search for range
	int found = 0;
	for (i = 0; i < m - 1; i++) {
		if ((x - data_m[i] >= 0) && (x - data_m[i + 1] <= 0)) {
			found = 1;
			break;
		}
	}
	
	// If somehow we didn't find the interval, use the first interval
	if (!found) {
		i = 0;
	}
	
	// Check for division by zero
	double denominator = data_m[i + 1] - data_m[i];
	if (fabs(denominator) < 1.0e-10) {
		// If nearly zero, use the value at i
		return data_C[i];
	}
	
	vary = (x - data_m[i]) / denominator; // Position parameter of x between mi and mi+1
	
	// Clamp vary to [0,1] to ensure interpolation stays within bounds
	if (vary < 0.0) vary = 0.0;
	if (vary > 1.0) vary = 1.0;
	
	// One-dimensional interpolation of x
	v = (1 - vary) * data_C[i] + vary * data_C[i + 1];
	return v;
}

/************************************************
              Two-dimensional Linear Interpolation Subroutine
************************************************/
double LAQL2(int m, int n, double* data_m, double* data_n, double* data_C, double x, double y)
{
	// Check for null pointers or invalid array sizes
	if (data_m == nullptr || data_n == nullptr || data_C == nullptr || m <= 1 || n <= 1) {
		cout << "Error in LAQL2: Invalid input parameters" << endl;
		return 0.0;
	}

	int i, j;
	long double v, vary1, vary2, v1, v2;
	
	// Value limiting
	if (x > data_m[m - 1])
		x = data_m[m - 1];
	else if (x < data_m[0])
		x = data_m[0];

	if (y > data_n[n - 1])
		y = data_n[n - 1];
	else if (y < data_n[0])
		y = data_n[0];

	// Search for range with bounds checking
	int found_j = 0;
	for (j = 0; j < n - 1; j++) {
		if ((y - data_n[j] >= 0) && (y - data_n[j + 1] <= 0)) {
			found_j = 1;
			break;
		}
	}
	
	// If somehow we didn't find the interval, use the first interval
	if (!found_j) {
		j = 0;
	}
	
	int found_i = 0;
	for (i = 0; i < m - 1; i++) {
		if ((x - data_m[i] >= 0) && (x - data_m[i + 1] <= 0)) {
			found_i = 1;
			break;
		}
	}
	
	// If somehow we didn't find the interval, use the first interval
	if (!found_i) {
		i = 0;
	}

	// Check for division by zero
	double denominator1 = data_m[i + 1] - data_m[i];
	double denominator2 = data_n[j + 1] - data_n[j];
	
	if (fabs(denominator1) < 1.0e-10) {
		// If x dimension nearly zero, use simple linear interpolation in y
		if (fabs(denominator2) < 1.0e-10) {
			// Both dimensions near zero, just return the value at (i,j)
			return data_C[j*m + i];
		}
		vary2 = (y - data_n[j]) / denominator2;
		vary2 = fmax(0.0, fmin(1.0, vary2)); // Clamp to [0,1]
		return (1 - vary2) * data_C[j*m + i] + vary2 * data_C[(j + 1)*m + i];
	}
	
	if (fabs(denominator2) < 1.0e-10) {
		// If y dimension nearly zero, use simple linear interpolation in x
		vary1 = (x - data_m[i]) / denominator1;
		vary1 = fmax(0.0, fmin(1.0, vary1)); // Clamp to [0,1]
		return (1 - vary1) * data_C[j*m + i] + vary1 * data_C[j*m + i + 1];
	}

	// Normal case - both dimensions ok
	vary1 = (x - data_m[i]) / denominator1; // Position parameter of x between mi and mi+1
	vary2 = (y - data_n[j]) / denominator2; // Position parameter of y between nj and nj+1
	
	// Clamp vary values to [0,1] to ensure interpolation stays within bounds
	vary1 = fmax(0.0, fmin(1.0, vary1));
	vary2 = fmax(0.0, fmin(1.0, vary2));

	// Two-dimensional interpolation with bounds checking
	// Directly calculate indices to avoid overflow
	int idx00 = j*m + i;
	int idx10 = j*m + (i + 1);
	int idx01 = (j + 1)*m + i;
	int idx11 = (j + 1)*m + (i + 1);
	
	// Check bounds before accessing
	if (idx00 >= 0 && idx10 >= 0 && idx01 >= 0 && idx11 >= 0 && 
	    idx00 < m*n && idx10 < m*n && idx01 < m*n && idx11 < m*n) {
		v1 = (1 - vary1) * data_C[idx00] + vary1 * data_C[idx10];
		v2 = (1 - vary1) * data_C[idx01] + vary1 * data_C[idx11];
		v = (1 - vary2) * v1 + vary2 * v2;
	} else {
		// Out of bounds access detected
		cout << "Warning: Out of bounds index in LAQL2 [m=" << m << ", n=" << n 
		     << ", i=" << i << ", j=" << j << "]" << endl;
		v = 0.0;
	}

	return v;
}

/************************************************
              Three-dimensional Linear Interpolation Subroutine
************************************************/
double LAQL3(int m, int n, int l, double* data_m, double* data_n, double* data_l, double* data_C, double x, double y, double z)
{
	// Check for null pointers or invalid array sizes
	if (data_m == nullptr || data_n == nullptr || data_l == nullptr || data_C == nullptr || 
	    m <= 1 || n <= 1 || l <= 1) {
		cout << "Error in LAQL3: Invalid input parameters" << endl;
		return 0.0;
	}

	int i, j, k;
	long double v, vary1, vary2, vary3, v1, v2, v3, v4, v5, v6;
	
	// Value limiting
	if (x > data_m[m - 1])
		x = data_m[m - 1];
	else if (x < data_m[0])
		x = data_m[0];

	if (y > data_n[n - 1])
		y = data_n[n - 1];
	else if (y < data_n[0])
		y = data_n[0];
		
	if (z > data_l[l - 1])
		z = data_l[l - 1];
	else if (z < data_l[0])
		z = data_l[0];

	// Search for range with bounds checking
	int found_k = 0;
	for (k = 0; k < l - 1; k++) {
		if ((z - data_l[k] >= 0) && (z - data_l[k + 1] <= 0)) {
			found_k = 1;
			break;
		}
	}
	
	// If somehow we didn't find the interval, use the first interval
	if (!found_k) {
		k = 0;
	}
	
	int found_j = 0;
	for (j = 0; j < n - 1; j++) {
		if ((y - data_n[j] >= 0) && (y - data_n[j + 1] <= 0)) {
			found_j = 1;
			break;
		}
	}
	
	// If somehow we didn't find the interval, use the first interval
	if (!found_j) {
		j = 0;
	}
	
	int found_i = 0;
	for (i = 0; i < m - 1; i++) {
		if ((x - data_m[i] >= 0) && (x - data_m[i + 1] <= 0)) {
			found_i = 1;
			break;
		}
	}
	
	// If somehow we didn't find the interval, use the first interval
	if (!found_i) {
		i = 0;
	}

	// Check for division by zero
	double denominator1 = data_m[i + 1] - data_m[i];
	double denominator2 = data_n[j + 1] - data_n[j];
	double denominator3 = data_l[k + 1] - data_l[k];
	
	if (fabs(denominator1) < 1.0e-10 || fabs(denominator2) < 1.0e-10 || fabs(denominator3) < 1.0e-10) {
		// If any dimension has zero width, fall back to simpler interpolation
		return LAQL2(m, n, data_m, data_n, data_C + k*m*n, x, y);
	}

	vary1 = (x - data_m[i]) / denominator1; // Position parameter of x
	vary2 = (y - data_n[j]) / denominator2; // Position parameter of y
	vary3 = (z - data_l[k]) / denominator3; // Position parameter of z
	
	// Clamp vary values to [0,1] to ensure interpolation stays within bounds
	vary1 = fmax(0.0, fmin(1.0, vary1));
	vary2 = fmax(0.0, fmin(1.0, vary2));
	vary3 = fmax(0.0, fmin(1.0, vary3));

	// Three-dimensional interpolation with bounds checking
	// Calculate indices with safety checks
	int total_size = m * n * l;
	int idx000, idx100, idx010, idx110, idx001, idx101, idx011, idx111;
	
	try {
		idx000 = (k * n + j) * m + i;
		idx100 = (k * n + j) * m + (i + 1);
		idx010 = (k * n + (j + 1)) * m + i;
		idx110 = (k * n + (j + 1)) * m + (i + 1);
		idx001 = ((k + 1) * n + j) * m + i;
		idx101 = ((k + 1) * n + j) * m + (i + 1);
		idx011 = ((k + 1) * n + (j + 1)) * m + i;
		idx111 = ((k + 1) * n + (j + 1)) * m + (i + 1);
		
		// Check if any index is out of bounds
		if (idx000 < 0 || idx100 < 0 || idx010 < 0 || idx110 < 0 ||
			idx001 < 0 || idx101 < 0 || idx011 < 0 || idx111 < 0 ||
			idx000 >= total_size || idx100 >= total_size || idx010 >= total_size || idx110 >= total_size ||
			idx001 >= total_size || idx101 >= total_size || idx011 >= total_size || idx111 >= total_size) {
			
			cout << "Warning: Out of bounds index in LAQL3 [m=" << m << ", n=" << n << ", l=" << l
				 << ", i=" << i << ", j=" << j << ", k=" << k << "]" << endl;
			
			// Use a safe fallback
			return 0.0;
		}
		
		// Interpolate the z=0 plane in the x and y directions
		v1 = (1 - vary1) * data_C[idx000] + vary1 * data_C[idx100];
		v2 = (1 - vary1) * data_C[idx010] + vary1 * data_C[idx110];
		v3 = (1 - vary2) * v1 + vary2 * v2;

		// Interpolate the z=1 plane in the x and y directions
		v4 = (1 - vary1) * data_C[idx001] + vary1 * data_C[idx101];
		v5 = (1 - vary1) * data_C[idx011] + vary1 * data_C[idx111];
		v6 = (1 - vary2) * v4 + vary2 * v5;

		// Interpolate in the z direction
		v = (1 - vary3) * v3 + vary3 * v6;
	}
	catch (const std::exception& e) {
		cout << "Exception in LAQL3: " << e.what() << endl;
		return 0.0;
	}
	catch (...) {
		cout << "Unknown exception in LAQL3" << endl;
		return 0.0;
	}

	return v;
}