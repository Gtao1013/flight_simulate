/************  AeroModel.h  ******************************
	 Aerodynamic Model
**********************************************************/

#ifndef  _C_AER_MODEL_H_
#define  _C_AER_MODEL_H_

#include "MotiModel.h"
#include "MassModel.h"
#include "Data_Parameter.h"


class CMotiModel;
class CMassModel;
class Data_Parameter;

/*********************Aerodynamic Model*********************/
double LAQL1(int m, double* data_m, double* data_C, double x);
double LAQL2(int m, int n, double* data_m, double* data_n, double* data_C, double x, double y);
double LAQL3(int m, int n, int p, double* data_m, double* data_n, double* data_p, double* data_C, double x, double y, double z);

class CDynaModel
{
public:
	int Num_H, Num_Ma, Num_Alfa, Num_beta;			//Height, Mach number, angle of attack, sideslip angle counts
	double* data_H, * data_Ma, * data_Alfa, * data_beta;		//Dynamic arrays for height, Mach number, angle of attack, sideslip angle
	double* data_Cxa, * data_Cya, * data_Mza, * data_Cxb, * data_Cyb, * data_Cz, * data_Mx, * data_My, * data_Mzb;		//Dynamic arrays for steady aerodynamic coefficients
	double* data_Mx_wx, * data_Mx_wy, * data_Mx_wz, * data_My_wx, * data_My_wy, * data_My_wz, * data_Mz_wx, * data_Mz_wy, * data_Mz_wz;		//Dynamic arrays for derivatives
	double Cx_a, Cx_b, Cx_0, Cx;		//Axial force in body system
	double Cy_a, Cy_b, Cy_0, Cy;		//Normal force in body system
	double Cz_b, Cz;					//Side force in body system
	double Mx_b, Mx;					//Roll moment
	double My_b, My;					//Yaw moment
	double Mz_a, Mz_b, Mz_0, Mz;		//Pitch moment
	double Mx_wx, Mx_wy, Mx_wz, My_wx, My_wy, My_wz, Mz_wx, Mz_wy, Mz_wz;	//Dynamic derivatives
	double t;
	double CD, CL, CZ;					//Drag, lift, side force in velocity system


public:
	CDynaModel();
	~CDynaModel();
	void InputData();										//Function to read aerodynamic data parameters
	void FreeData();										//Release dynamic arrays
	void GetForce_Moment_Motion(CMassModel* ma, CMotiModel* mo, Data_Parameter* Gdata); //Calculate aerodynamic forces
};

#endif





























