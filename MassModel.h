/************  MassModel.h  ******************************
	 Mass Model
**********************************************************/

#ifndef  _C_MASS_MODEL_H_
#define  _C_MASS_MODEL_H_

#include "MotiModel.h"
#include "Data_Parameter.h"
#include "EngiModel.h"

class Data_Parameter;
class CEngiModel;
class CMotiModel;

double LAQL1(int n, double* x, double* y, double u);

/************************Mass Model**********************/
class CMassModel
{
public:
	double m;
	double Sm, Lk;						//Aerodynamic reference area, reference length
	double G_m0, dm_dt;		//Mass, mass consumption rate
	double current_mass_rate;  // Current mass consumption rate in kg/s
	double Xc_Lca;				//Center of mass distance	  
	double Xcg, Jx, Jy, Jz, Jxy;				//Center of mass position, moment of inertia

public:
	void IniData(Data_Parameter* Gdata, CEngiModel* en, CMotiModel* mo);			                 //Initialization function
	CMassModel();
	~CMassModel();
	
	// Get current mass consumption rate in kg/s
	double GetCurrentMassRate() const { return current_mass_rate; }
};

#endif





























