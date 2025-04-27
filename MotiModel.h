/************  MotiModel.h  ******************************
	 Missile Dynamics Model
**********************************************************/

#ifndef  _MOTI_MODEL_H_
#define  _MOTI_MODEL_H_

#include "MassModel.h"
#include "AeroModel.h"
#include "EngiModel.h"
#include "Data_Parameter.h"
#include "Nvgt_Ctrl_Model.h"

class CMassModel;
class CDynaModel;
class CEngiModel;
class Data_Parameter;
class CNCModel;


double LAQL1(int n, double* x, double* y, double u);
double Sign1(double x);
double Limit(double Xi, double Xmax);
double Get_Atmo_pp(double h);
double Get_Atmo_tt(double h);
double Get_Atmo_aa(double h);
double Get_Atmo_rr(double pp, double tt);
double Get_Atmo_windx(double h);
double Get_Atmo_windz(double h);

class CMotiModel
{
public:
	int VariNum, kmass;
	double y[15], d[15];										//Values of dynamic differential equations
	double dy_X, dy_Y, dy_Z, dy_Mz, dy_My, dy_Mx, dy_Xcp, dy_P_P0, dy_R_R0, dy_Fc;
	double Fpx2, Fpy2, Fpz2, Gx2, Gy2, Gz2, Rxx2, Ryx2, Rzx2, Rxy2, Ryy2, Rzy2, Rxz2, Ryz2, Rzz2, Fcx2, Fcy2, Fcz2;
	double V, t, Psi_v, Wx, Wy, Wz, Fai, THETA, Psi, X, Y, Z, Gama, Theta, Vx, Vy, Vz;
	double Xa, Ya, Za, Fe, R, RR, H, g, Rc, m, m_t;
	double Deltr, Delt1, Delt2, Alfab, Vi, Theta0, Ma;
	double nx, ny, nz, nx2, ny2, nz2;							 //Overload in missile axis and trajectory coordinates
	double dTHETA_dt, dPsi_dt, dGama_dt, Wx_t, Wy_t, Wz_t;
	double Windx, Windz, tt, pp, rho, aa;
	double Fcx3, Fcy3, Fcz3;
	double gx, gy, gz;
	double sinbeta, sinalfa, singamv, cosalfa, cosgamv, beta, alfa, gamv;
	double Qv, CX, CY, CZ;
	double Delt_x, Delt_y, Delt_z;
	double SonicV;
	double MMx, MMy;
	double Rx3, Ry3, Rz3;		//	Aerodynamic forces projected in velocity system, i.e., lift, drag, side force
	double Rx2, Ry2, Rz2;		//	Aerodynamic forces projected in trajectory system
	double Rx1, Ry1, Rz1;		//	Aerodynamic forces projected in body system
	double dy_CD, dy_CL, dy_CZ; //	Aerodynamic coefficients
	double MX, Mx_static, Mx_Wx, Mx_Wy, Mx_Wz;			//	Aerodynamic moments
	double MY, My_static, My_Wx, My_Wy, My_Wz;			//	Aerodynamic moments
	double MZ, Mz_static, Mz_Wx, Mz_Wy, Mz_Wz;			//	Aerodynamic moments
	double dy_Mz_dong, dy_My_dong, dy_Mx_dong;			//	Dynamic derivatives
	double end_parameter;	//End parameter
	
	// Variables used in AeroModel.cpp
	double F_aero;    // Total aerodynamic force
	double Fx_aero;   // Aerodynamic force in X direction
	double Fy_aero;   // Aerodynamic force in Y direction
	double Fz_aero;   // Aerodynamic force in Z direction
	double Mx_aero;   // Aerodynamic moment around X axis
	double My_aero;   // Aerodynamic moment around Y axis
	double Mz_aero;   // Aerodynamic moment around Z axis
	double WX;        // Angular velocity around X axis (rad/s)
	double WY;        // Angular velocity around Y axis (rad/s)
	double WZ;        // Angular velocity around Z axis (rad/s)

public:

	CMotiModel();
	~CMotiModel();
	void  IniData(CMassModel* ma, Data_Parameter* Gdata);//Initialization
	void  RightFun(CMassModel* ma, CEngiModel* en, CDynaModel* dy, Data_Parameter* Gdata, CNCModel* nc);  //Integration right function
	void  RenewStus(CMassModel* ma, CEngiModel* en, CDynaModel* dy, Data_Parameter* Gdata,CNCModel* nc); //Update position, velocity, attitude angles, etc.
	void  Motion(CMassModel* ma, CEngiModel* en, CDynaModel* dy, Data_Parameter* Gdata, CNCModel* nc, double hStep);//Main dynamics control program

};
#endif