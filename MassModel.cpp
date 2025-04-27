#include "MassModel.h"
#include "Global_Constant.h"
#include "math.h"
#include <stdio.h>
#include <stdlib.h>

/////////// Implementation for CMassModel ///////////////
CMassModel::CMassModel()
{
    // 初始化当前质量流率为0
    current_mass_rate = 0.0;
}

CMassModel::~CMassModel()
{ }

void CMassModel::IniData(Data_Parameter* Gdata, CEngiModel* en, CMotiModel* mo)
{
    Sm = Gdata->Sm;                        // Reference area
    Lk = Gdata->Lk;                        // Reference length
    G_m0 = Gdata->G_m0;                    // Initial mass
    dm_dt = Gdata->dm_dt;                  // Mass consumption rate
    
    // Set mass to initial mass during initialization
    if (mo->t <= 0.0) {
        m = G_m0;
    }
    // Otherwise update mass according to thrust model
    else if (Gdata->flag_Fp == 1) {
        if (mo->t >= en->t_Fp_begin && mo->t <= en->t_Fp_end) {
            // If using variable RPM thrust model, use dynamic mass consumption rate
            if (en->use_rpm_model == 1) {
                // 使用GetMassRate()方法来获取当前质量流率，提高代码一致性
                current_mass_rate = en->GetMassRate();
                m = m - current_mass_rate * Gdata->h;
            }
            // Otherwise use fixed consumption rate
            else {
                current_mass_rate = dm_dt;
                m = m - current_mass_rate * Gdata->h;
            }
        }
    }
    
    Jx = Gdata->J_x * m;                   // Moment of inertia
    Jy = Gdata->J_y * m;
    Jz = Gdata->J_z * m;
    Jxy = Gdata->J_xy * m;
}
