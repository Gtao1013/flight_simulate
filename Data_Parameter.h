/*################################################
	This file contains essential parameter data definitions

#################################################*/
#ifndef _Data_Parameter_H
#define _Data_Parameter_H
#include "Global_Constant.h"

class Data_Parameter
{
public:

	int Kfly, Katmo;			//Integer indicators
	double h, h_out, H_end;	//Integration step size and flight termination height	
	double time;
	double tl, ml, Vl, Vlx, Vly, Vlz, Xl, Yl, Zl, Thetal, Posil, Gamal, Hll, Qv_tl, Ma_tl, alfa_tl, beta_tl, Ny_tkl, Nz_tkl;											//Previous moment velocity, position and shear standard	
	//Initial parameters
	double V0, Theta0, Gama0, X0, Y0, Z0, H0, p00, Psi_v0, THETA0, Psi0, Wx0, Wz0, Wy0, Ma, RR_0, alpha0, beta0;
	//Engine parameters
	double flag_Fp;				  //Indicator - whether with engine						
	//Aerodynamic parameters
	double Sm, Lk;				                      //Aerodynamic parameters: reference area/aerodynamic reference length
	double G_m0, dm_dt;	                  //Mass
	double J_x, J_y, J_z, J_xy, x_cg;	                      //Moments of inertia
	int flag_1, flag_2, flag_3, flag_4;			//Integer indicators
	//Navigation guidance parameters
	int flag_NC;		//Indicator - control mode
	int flag_dy;		//Indicator - output mode

	Data_Parameter();
	~Data_Parameter();
	int InputData();
	
	// Declaration of function to initialize default values
	void InitializeDefaultValues();
};

#endif