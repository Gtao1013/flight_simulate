/******积分右函数及积分函数********
***********************************************/
#include"math.h"
#include <stdio.h>
#include <stdlib.h>

#include  "MotiModel.h"
#include  "MassModel.h"
#include  "Global_Constant.h"
#include  "Data_Parameter.h"
#include  "EngiModel.h"


//################################################################################################以下为欧拉积分
void Integral_motion(CMassModel* ma, CDynaModel* dy, CEngiModel* en, CMotiModel* mo, Data_Parameter* Gdata, CNCModel* nc, double hStep)
{
	int i;
	double z[100];
	int	n;
	double	h = hStep;

	n = mo->VariNum;


	for (i = 0;i < n;i++)	z[i] = 0.0;
	for (i = 0; i < n; i++) z[i] = mo->y[i];

	// K1	
	mo->RightFun(ma, en, dy, Gdata, nc);

	//Yn+K1*h
	for (i = 0; i < n; i++)	mo->y[i] = mo->y[i] + h * mo->d[i];

	//  Yn+0.5*h*K1	
	for (i = 0; i < n; i++)	z[i] = z[i] + 0.5 * h * mo->d[i];

	mo->RightFun(ma, en, dy, Gdata, nc);

	// Yn+0.5*h*K1+0.5*h*K2
	for (i = 0; i < n; i++)	mo->y[i] = z[i] + 0.5 * h * mo->d[i];

	return;
}

//################################################################################################以下为4阶龙格库塔法
void Integral_ode45(CMassModel* ma, CDynaModel* dy, CEngiModel* en, CMotiModel* mo, Data_Parameter* Gdata, CNCModel* nc, double hStep)
{
	int i;
	double z1[100], z2[100];
	int	n;
	double	h = hStep;

	n = mo->VariNum;


	for (i = 0;i < n;i++)	z1[i] = 0.0;
	for (i = 0;i < n;i++)	z2[i] = 0.0;
	for (i = 0; i < n; i++) z1[i] = mo->y[i];
	for (i = 0; i < n; i++) z2[i] = mo->y[i];

	// K1	
	mo->RightFun(ma, en, dy,  Gdata, nc);	//Get yn,dn
	for (i = 0; i < n; i++)	z2[i] = z2[i] + h / 6 * mo->d[i];

	//K2
	for (i = 0; i < n; i++) mo->y[i] = z1[i] + 0.5 * h * mo->d[i];	//Get y+0.5hd
	mo->RightFun(ma, en, dy, Gdata, nc);									//Get d(t+0.5h,y+0.5hd)
	for (i = 0; i < n; i++)	z2[i] = z2[i] + h / 3 * mo->d[i];

	//K3
	for (i = 0; i < n; i++) mo->y[i] = z1[i] + 0.5 * h * mo->d[i];
	mo->RightFun(ma, en, dy, Gdata, nc);
	for (i = 0; i < n; i++)	z2[i] = z2[i] + h / 3 * mo->d[i];

	//K4
	for (i = 0; i < n; i++) mo->y[i] = z1[i] +  h * mo->d[i];
	mo->RightFun(ma, en, dy, Gdata, nc);
	for (i = 0; i < n; i++)	z2[i] = z2[i] + h / 6 * mo->d[i];
	
	for (i = 0; i < n; i++)	mo->y[i] = z2[i];
	return;
}
