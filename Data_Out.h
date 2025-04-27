/************  Data_Out->h  ******************************
	 Data output file header
**********************************************************/
#ifndef  _C_DATA_OUT_H_
#define  _C_DATA_OUT_H_

//#include <fstream>
#include <stdio.h>
#include <string.h> // Adding string.h header file, supporting strstr and strcmp functions
#include "MotiModel.h"
#include "MassModel.h"
#include "AeroModel.h"
#include "EngiModel.h"
#include "Data_Parameter.h"
#include <direct.h>

//using namespace std;
class   CMotiModel;
class	Data_Parameter;
class	CMassModel;
class	CDynaModel;
class   CEngiModel;



class CDataout
{
public:

	FILE* fp1, * fp2, * fp3, * fp4, * fp5, * fp6, * fp7, * fp8, * fp9;


	struct Tra_data
	{
		int type;											//type=0: indicates not output; 1 indicates output
		const char* name;									//Output data name	
		double dat;											//Output data value
	};

	Tra_data Dat[86];


public:
	void Initial(Data_Parameter* Gdata);		           //			
	void OpenOutFiles(Data_Parameter* Gdata);              //Open file
	void OutputTrajectory(Data_Parameter* Gdata, CMassModel* ma, CMotiModel* mo);
	void CloseOutFile(Data_Parameter* Gdata);
	void Outputdefined(Data_Parameter* Gdata, CDynaModel* dy, CMassModel* ma, CEngiModel* en, CMotiModel* mo);
	CDataout();
	~CDataout();

};
#endif

