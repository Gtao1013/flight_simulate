#include"math.h"
#include "Global_Constant.h"
#include <stdio.h>
#include <string.h>  // Added header file for string operations
#include "Data_Out.h"

// Define _countof if not defined
#ifndef _countof
#define _countof(array) (sizeof(array)/sizeof(array[0]))
#endif

/**********************************
          Data Output
***********************************/
CDataout::CDataout()
{ }

CDataout::~CDataout()
{	}

void CDataout::Initial(Data_Parameter* Gdata)
{
	int i;
	for (i = 0;i < 46;i++)
	{
		Dat[i].type = 1;
		Dat[i].dat = 0.0;
	}
	i = 0;
	if (Gdata->flag_2 == 1)	//origin
	{ 
	//Position and velocity information 1-12
	Dat[i++].name = "time(s)";		Dat[i++].name = "Ma";			Dat[i++].name = "V(m/s)";		//Flight time, Mach number
	Dat[i++].name = "X(km)";		Dat[i++].name = "Y(km)";		Dat[i++].name = "Z(km)";		//Missile position
	Dat[i++].name = "Vx(m/s)";		Dat[i++].name = "Vz(m/s)";		Dat[i++].name = "Vy(m/s)";		//Velocity in three directions
	Dat[i++].name = "Wx(°/s)";		Dat[i++].name = "Wy(°/s)";		Dat[i++].name = "Wz(°/s)";		//Pitch, yaw, roll angular velocity

	//Attitude module 13-20
	Dat[i++].name = "Theta(°)";		Dat[i++].name = "Psi_v(°)";											//Trajectory elevation and azimuth angles
	Dat[i++].name = "THETA1(°)";	    Dat[i++].name = "Psi(°)";			Dat[i++].name = "Gama(°)";			//Pitch, yaw, roll angles
	Dat[i++].name = "alpha(°)";		Dat[i++].name = "beta(°)";			Dat[i++].name = "gamv(°)";			//Angle of attack, sideslip angle, velocity roll angle

	//21-44 Aerodynamic module values
	Dat[i++].name = "CD";		Dat[i++].name = "CL";		Dat[i++].name = "CZ";			//Drag, lift, side force coefficients
	Dat[i++].name = "mx0";		Dat[i++].name = "my0";		Dat[i++].name = "mz0";			//Roll, yaw, pitch moment coefficients
	Dat[i++].name = "mx_wx";	Dat[i++].name = "mx_wy";	Dat[i++].name = "mx_wz";		//Roll moment derivatives
	Dat[i++].name = "my_wx";	Dat[i++].name = "my_wy";	Dat[i++].name = "my_wz";		//Yaw moment derivatives
	Dat[i++].name = "mz_wx";	Dat[i++].name = "mz_wy";	Dat[i++].name = "mz_wz";		//Pitch moment derivatives
	Dat[i++].name = "D";		Dat[i++].name = "L";		Dat[i++].name = "Z";			//Drag, lift, side forces
	Dat[i++].name = "MX";		Dat[i++].name = "MY";		Dat[i++].name = "MZ";			//Roll, yaw, pitch moments
	Dat[i++].name = "T";		Dat[i++].name = "rho";		Dat[i++].name = "P";			//Flow temperature, density, pressure
	Dat[i++].name = "dy_P/P0";	Dat[i++].name = "dy_rho/rho0";								//Pressure ratio, density ratio
	}

	else if (Gdata->flag_2 == 2)	//Tecplot
	{ 
		//Position and velocity information 1-12
		Dat[i++].name = "time(s)";		Dat[i++].name = "Ma";			Dat[i++].name = "V(m/s)";		//Flight time, Mach number
	Dat[i++].name = "X(km)";		Dat[i++].name = "Y(km)";		Dat[i++].name = "Z(km)";		//Missile position
	Dat[i++].name = "Vx(m/s)";		Dat[i++].name = "Vz(m/s)";		Dat[i++].name = "Vy(m/s)";		//Velocity in three directions
	Dat[i++].name = "Wx(<sup><greek>o</greek></sup>/s)";		Dat[i++].name = "Wy(<sup><greek>o</greek></sup>/s)";		Dat[i++].name = "Wz(<sup><greek>o</greek></sup>/s)";		//Pitch, yaw, roll angular velocity

	//Attitude module 13-20
	Dat[i++].name = "Theta(<sup><greek>o</greek></sup>)";		Dat[i++].name = "Psi_v(<sup><greek>o</greek></sup>)";											//Trajectory elevation and azimuth angles
	Dat[i++].name = "THETA1(<sup><greek>o</greek></sup>)";	    Dat[i++].name = "Psi(<sup><greek>o</greek></sup>)";			Dat[i++].name = "Gama(<sup><greek>o</greek></sup>)";			//Pitch, yaw, roll angles
	Dat[i++].name = "alpha(<sup><greek>o</greek></sup>)";		Dat[i++].name = "beta(<sup><greek>o</greek></sup>)";			Dat[i++].name = "gamv(<sup><greek>o</greek></sup>)";			//Angle of attack, sideslip angle, velocity roll angle

	//21-44 Aerodynamic module values
	Dat[i++].name = "CD";		Dat[i++].name = "CL";		Dat[i++].name = "CZ";			//Drag, lift, side force coefficients
	Dat[i++].name = "mx0";		Dat[i++].name = "my0";		Dat[i++].name = "mz0";			//Roll, yaw, pitch moment coefficients
	Dat[i++].name = "mx_wx";	Dat[i++].name = "mx_wy";	Dat[i++].name = "mx_wz";		//Roll moment derivatives
	Dat[i++].name = "my_wx";	Dat[i++].name = "my_wy";	Dat[i++].name = "my_wz";		//Yaw moment derivatives
	Dat[i++].name = "mz_wx";	Dat[i++].name = "mz_wy";	Dat[i++].name = "mz_wz";		//Pitch moment derivatives
	Dat[i++].name = "D";		Dat[i++].name = "L";		Dat[i++].name = "Z";			//Drag, lift, side forces
	Dat[i++].name = "MX";		Dat[i++].name = "MY";		Dat[i++].name = "MZ";			//Roll, yaw, pitch moments
	Dat[i++].name = "T";		Dat[i++].name = "rho";		Dat[i++].name = "P";			//Flow temperature, density, pressure
	Dat[i++].name = "dy_P/P0";	Dat[i++].name = "dy_rho/rho0";								//Pressure ratio, density ratio
	}

	return;
}

void CDataout::OpenOutFiles(Data_Parameter* Gdata)											//Open file
{
	int i;

	//fopen_s(&fp1, "Output\\Trajectory.dat", "w");										//Full trajectory data file					    
	//fprintf(fp1, "%10s%16s%16s%16s%16s%16s%16s%16s%16s%16s%16s%16s%16s%16s%16s%16s%16s\n", "time", "X", "Y", "Z", "Vx", "Vy", "Vz", "V", "θ", "ψv ", "α", "β", "γv", "Theta", "ψ", "γ", "Ma");

	if (Dat[0].type > 0)																			//Custom data output
	{
		fopen_s(&fp3, "Output\\Defined.dat", "w");
		if (Gdata->flag_2 == 1)
		{
			//origin interface
			fprintf(fp3, "%10s", Dat[0].name);
			for (i = 1;i < 9;i++)
				fprintf(fp3, "%15s", Dat[i].name);			//Set character length in sections, because "°" takes one character length but counts as two characters
			for (i = 9;i < 20;i++)
				fprintf(fp3, "%16s", Dat[i].name);
			for (i = 20;i < 46;i++)
				fprintf(fp3, "%15s", Dat[i].name);
		}
		else if (Gdata->flag_2 == 2)
		{
			//tecplot interface
			fprintf(fp3, "%s", "variables=");
			for (i = 0;i < 46;i++)
				if (Dat[i].type > 0)
				{
					fprintf(fp3, "%s", "\"");
					fprintf(fp3, "%s", Dat[i].name);
					fprintf(fp3, "%s", "\", ");
				}
		}
		fprintf(fp3, "\n");
	}
}

void CDataout::Outputdefined(Data_Parameter* Gdata, CDynaModel* dy, CMassModel* ma, CEngiModel* en, CMotiModel* mo)			//Custom data output
{
	int i;
	i = 0;

	//Missile and target position and velocity information 1-12
	Dat[i++].dat = mo->y[0];		Dat[i++].dat = mo->Ma;			Dat[i++].dat = mo->V;					//Flight time
	Dat[i++].dat = mo->X / 1000;	Dat[i++].dat = mo->H / 1000;	Dat[i++].dat = mo->Z / 1000;			//Missile position (km)
	Dat[i++].dat = mo->Vx;			Dat[i++].dat = mo->Vz;			Dat[i++].dat = mo->Vy;					//Velocity in three directions
	Dat[i++].dat = mo->Wx * RtD;	Dat[i++].dat = mo->Wy * RtD;	Dat[i++].dat = mo->Wz * RtD; 	//Pitch, yaw, roll angular velocity

	//Attitude module 13-20
	Dat[i++].dat = mo->Theta * RtD;	Dat[i++].dat = mo->Psi_v * RtD;										//Trajectory elevation and azimuth angles
	Dat[i++].dat = mo->THETA * RtD;	Dat[i++].dat = mo->Psi * RtD;		Dat[i++].dat = mo->Gama * RtD;	//Pitch, yaw, roll angles
	Dat[i++].dat = mo->alfa * RtD;	Dat[i++].dat = mo->beta * RtD;		Dat[i++].dat = mo->gamv * RtD;	//Angle of attack, sideslip angle, velocity roll angle

	//21-46 Aerodynamic module values
	Dat[i++].dat = mo->dy_CD;			Dat[i++].dat = mo->dy_CL;			Dat[i++].dat = mo->dy_CZ;						//Drag, lift/side force
	Dat[i++].dat = mo->dy_Mx;			Dat[i++].dat = mo->dy_My;			Dat[i++].dat = mo->dy_Mz;						//Roll, yaw, pitch moment coefficients
	Dat[i++].dat = dy->Mx_wx;			Dat[i++].dat = dy->Mx_wy;			Dat[i++].dat = dy->Mx_wz;						//Roll moment derivatives
	Dat[i++].dat = dy->My_wx;			Dat[i++].dat = dy->My_wy;			Dat[i++].dat = dy->My_wz;						//Yaw moment derivatives
	Dat[i++].dat = dy->Mz_wx;			Dat[i++].dat = dy->Mz_wy;			Dat[i++].dat = dy->Mz_wz;						//Pitch moment derivatives
	Dat[i++].dat = mo->Rx3;				Dat[i++].dat = mo->Ry3;				Dat[i++].dat = mo->Rz3;							//Drag, lift, side forces
	Dat[i++].dat = mo->MX;				Dat[i++].dat = mo->MY;				Dat[i++].dat = mo->MZ;							//Roll, yaw, pitch moments
	Dat[i++].dat = mo->tt;				Dat[i++].dat = mo->rho;				Dat[i++].dat = mo->pp;							//Flow temperature, density, pressure
	Dat[i++].dat = mo->dy_P_P0;			Dat[i++].dat = mo->dy_R_R0;															//Mach number, pressure ratio, density ratio
	
	if (Dat[0].type > 0)
	{
		fprintf(fp3, "%10.5f", Dat[0].dat);		//Output variable values with right alignment in 10-character field width, retaining 5 decimal places

		for (i = 1;i < 46;i++)
			if (Dat[i].type > 0)	fprintf(fp3, "%15.6e", Dat[i].dat);		//Output in exponential form

		fprintf(fp3, "\n");
	}
	return;
}

void CDataout::OutputTrajectory(Data_Parameter* Gdata, CMassModel* ma, CMotiModel* mo)		//Trajectory data output
{
	fprintf(fp1, "%10.3f%16.6e%16.6e%16.6e%16.6e%16.6e%16.6e%16.6e%16.6e%16.6e%16.6e%16.6e%16.6e%16.6e%16.6e%16.6e%16.6e\n",
		mo->y[0], mo->X, mo->H, mo->Z, mo->Vx, mo->Vy, mo->Vz, mo->V, mo->Theta * RtD, mo->Psi_v * RtD, mo->alfa * RtD, mo->beta * RtD, mo->gamv * RtD, mo->THETA * RtD, mo->Psi * RtD, mo->Gama * RtD, mo->Ma);
	return;
}
