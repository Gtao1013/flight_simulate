#define _CRT_SECURE_NO_WARNINGS
#include "math.h"
#include "MotiModel.h"
#include "Global_Constant.h"
#include "Atmosphere.h"  // Add reference to Atmosphere.h
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

// Declaration of LAQL1 function
double LAQL1(int m, double* data_m, double* data_C, double x);

using namespace std;

CMotiModel::CMotiModel()
{
	VariNum = 15;
	for (int i = 0;i < VariNum;i++)
	{
		y[i] = 0.0;
		d[i] = 0.0;
	}
}

CMotiModel::~CMotiModel()
{

}

void CMotiModel::IniData(CMassModel* ma, Data_Parameter* Gdata)   // Initialization
{
	for (int i = 0;i < VariNum;i++)
	{
		y[i] = 0.0;
		d[i] = 0.0;
	}
	//

	t = Gdata->time;
	m = ma->m;
	X = Gdata->X0;
	Y = Gdata->Y0;
	Z = Gdata->Z0;
	Ma = Gdata->Ma;
	pp = Get_Atmo_pp(Y);
	tt = Get_Atmo_tt(Y);
	SonicV = Get_Atmo_aa(tt);
	rho = Get_Atmo_rr(pp, tt);
	V = Ma * SonicV;
	Wx = Gdata->Wx0;
	Wy = Gdata->Wy0;
	Wz = Gdata->Wz0;

	Theta = Gdata->Theta0;							// Pitch angle
	Psi_v = Gdata->Psi_v0;							// Velocity yaw angle, referred to as psi_v
	Gama = Gdata->Gama0;							// Roll angle
	THETA = Gdata->THETA0;                          // Initial pitch attitude angle, referred to as theta
	Psi = Gdata->Psi0;                              // Initial yaw angle, referred to as posi
	pp = pp0;									// Standard atmospheric pressure

	dTHETA_dt = Wy * sin(Gama) + Wz * cos(Gama);				// Pitch angle rate
	dPsi_dt = (Wy * cos(Gama) - Wz * sin(Gama)) / cos(THETA);		// Yaw angle rate
	dGama_dt = Wx - (Wy * cos(Gama) - Wz * sin(Gama)) * tan(THETA);		// Roll angle rate

	Vx = V * cos(Theta) * cos(Psi_v);
	Vy = V * sin(Theta);
	Vz = -V * cos(Theta) * sin(Psi_v);

	y[0] = t;		// Time
	y[1] = m;		// Mass
	y[2] = V;		// Velocity

	y[3] = Theta;	// Pitch angle
	y[4] = Psi_v;	// Velocity yaw angle
	y[5] = Wx;		// Roll angular velocity
	y[6] = Wy;		// Yaw angular velocity
	y[7] = Wz;		// Pitch angular velocity

	y[8] = THETA;	// Pitch attitude
	y[9] = Psi;	    // Yaw attitude
	y[10] = X;		// X position
	y[11] = Y;		// Y position
	y[12] = Z;		// Z position
	y[13] = Gama;	// Roll angle
	y[14] = pp;		// Pressure

	Qv = 0.5*rho*V*V;		// Dynamic pressure

	sinbeta = cos(Theta) * (cos(Gama) * sin(Psi - Psi_v) + sin(THETA) * sin(Gama) * cos(Psi - Psi_v)) - sin(Theta) * cos(THETA) * sin(Gama);
	beta = asin(sinbeta);   // Sideslip angle (beta)

	sinalfa = (cos(Theta) * (sin(THETA) * cos(Gama) * cos(Psi - Psi_v) - sin(Gama) * sin(Psi - Psi_v)) - sin(Theta) * cos(THETA) * cos(Gama)) / cos(beta);
	alfa = asin(sinalfa);  // Angle of attack

	singamv = (cos(alfa) * sin(beta) * sin(THETA) - sin(alfa) * sin(beta) * cos(Gama) * cos(THETA) + cos(beta) * sin(Gama) * cos(THETA)) / cos(Theta);
	gamv = asin(singamv);

	return;
}

void CMotiModel::RightFun(CMassModel* ma, CEngiModel* en, CDynaModel* dy, Data_Parameter* Gdata, CNCModel* nc) // Right side function of differential equations
{
	RenewStus(ma, en, dy, Gdata, nc);

	d[0] = 1.0;												// Time derivative

	d[1] = 0.0;                                            // Mass derivative

	d[2] = (Fpx2 + Gx2 + Rx2) / ma->m;			        // dv/dt

	d[3] = (Fpy2 + Gy2 + Ry2) / (ma->m * V);	            // dTheta/dt (trajectory pitch angle rate)

	d[4] = (Fpz2 + Gz2 + Rz2) / (-ma->m * V * cos(Theta));	// dPsi_v/dt (trajectory heading angle rate)

	MMx = MX - (ma->Jz - ma->Jy) * Wz * Wy - ma->Jxy * Wx * Wz;		// dwx modified moment
	MMy = MY - (ma->Jx - ma->Jz) * Wx * Wz - ma->Jxy * Wz * Wy;		// dwy modified moment

	d[5] = (ma->Jxy * MMy + ma->Jy * MMx) / (ma->Jx * ma->Jy - ma->Jxy * ma->Jxy);		// d(Wx) / dt

	d[6] = (ma->Jxy * MMx + ma->Jx * MMy) / (ma->Jx * ma->Jy - ma->Jxy * ma->Jxy);		// d(Wy) / dt

	d[7] = (MZ - (ma->Jy - ma->Jx) * Wx * Wy - ma->Jxy * (Wy * Wy - Wx * Wx)) / ma->Jz;      // d(Wz)/dt

	if (Gdata->flag_NC == 1)    // Using navigation control mode
	{
		if (nc->is_theta_control) // Using trajectory pitch angle control (Theta)
		{
			nc->Get_Theta_Ctrl(Gdata, this);
		}
		else // Using traditional angle of attack control
		{
			nc->Get_Alfa_Ctrl(Gdata, this);
		}

		if (t >= nc->t_NC_begin && t <= nc->t_NC_end)
		{
			// For trajectory pitch angle control, we'll manipulate body angular rates
			// to indirectly control the trajectory pitch angle through angle of attack

			// This approach maintains the right dynamics (we don't directly set Theta)
			// The angle of attack command from Get_Theta_Ctrl will be used in RenewStus

			// We're leaving d[7] (Wz - pitch rate) calculation to follow guidance,
			// which should help steer toward the desired trajectory pitch angle
			d[7] = 0;
		}
	}

	// These differential equation calculations remain unchanged
	d[8] = Wy * sin(Gama) + Wz * cos(Gama);						// dTHETA/dt (body pitch angle rate)

	d[9] = (Wy * cos(Gama) - Wz * sin(Gama)) / cos(THETA);		    // dPsi/dt (body heading angle rate)

	d[10] = V * cos(Theta) * cos(Psi_v);							// dx/dt

	d[11] = V * sin(Theta);									// dy/dt

	d[12] = -V * cos(Theta) * sin(Psi_v);						// dz/dt

	d[13] = Wx - (Wy * cos(Gama) - Wz * sin(Gama)) * tan(THETA);		// dGama/dt (roll angle rate)

	d[14] = -rho * g00 * V * sin(Theta);							//dp/dt
}

void CMotiModel::RenewStus(CMassModel* ma, CEngiModel* en, CDynaModel* dy, Data_Parameter* Gdata, CNCModel* nc)
{
    t = y[0];
    m = y[1];
    V = y[2];
    Theta = y[3];
    Psi_v = y[4];

    // 确保速度为正值
    if (V < 1.0) {
        V = 1.0; // 最小安全速度
        cout << "警告: 强制应用最小速度: " << V << " m/s" << endl;
    }

    Vx = V * cos(Theta) * cos(Psi_v);
    Vy = V * sin(Theta);
    Vz = -V * cos(Theta) * sin(Psi_v);

    Wx = y[5];
    Wy = y[6];
    Wz = y[7];

    THETA = y[8];
    Psi = y[9];

    X = y[10];
    Y = y[11];
    Z = y[12];

    Gama = y[13];
    pp = y[14];

    Wx_t = d[5];
    Wy_t = d[6];
    Wz_t = d[7];

    dTHETA_dt = d[8];
    dPsi_dt = d[9];
    dGama_dt = d[13];

    H = Y;

    if (Gdata->flag_NC == 1)    // 使用导航控制模式
    {
        if (nc->is_theta_control) // 使用轨迹俯仰角控制
        {
            // 调试输出，显示控制调用前的状态
            if (int(t * 100) % 50 == 0) { // 每0.5秒打印一次
                printf("控制调用前状态: t=%.3f, Theta=%.1f°, THETA=%.1f°, alfa=%.1f°\n",
                    t, Theta * RtD, THETA * RtD, alfa * RtD);
            }

            nc->Get_Theta_Ctrl(Gdata, this);

            if (t >= nc->t_NC_begin && t <= nc->t_NC_end)
            {
                // 对于轨迹俯仰角控制，我们不直接设置Theta
                // 而是修改飞机状态以实现所需的轨迹角度

                // 记录当前攻角用于调试
                double prev_alfa = alfa;

                // Get_Theta_Ctrl计算的攻角命令(nc->alfa_NC)
                // 将被气动模型用于生成引导飞机向目标轨迹俯仰角的力和力矩

                // 可以选择性地基于alfa_NC对THETA进行修正，以帮助
                // 实现期望的轨迹俯仰角
                double theta_correction = nc->alfa_NC * 0.5; // 缩放因子调整
                THETA += theta_correction;

                // 定期输出调试信息
                if (int(t * 100) % 50 == 0) { // 每0.5秒
                    printf("轨迹控制: 目标Theta=%.1f°, 当前Theta=%.1f°, 攻角命令=%.1f°, 机体THETA=%.1f°\n",
                        nc->theta_target * RtD, Theta * RtD, nc->alfa_NC * RtD, THETA * RtD);
                }
            }
        }
        else // 使用传统攻角控制
        {
            nc->Get_Alfa_Ctrl(Gdata, this);

            if (t >= nc->t_NC_begin && t <= nc->t_NC_end)
            {
                // 对于alpha控制模式，我们直接使用控制角度
                // 每5秒输出一次调试信息
                if (int(t * 100) % 500 == 0) {
                    printf("攻角控制: 命令alfa=%.1f°\n", nc->alfa_NC * RtD);
                }
            }
        }
    }

    // 根据高度获取大气参数(压力、密度、温度、声速)
    // 根据Katmo设置使用标准大气模型或文件数据
    if (Gdata->Katmo == 0)
    {
        tt = 288.15 - 0.0065 * H;
        pp = pp0 * pow(tt / 288.15, 5.25588); // 标准大气模型
        rho = rho0 * pow(tt / 288.15, 4.25588);
        aa = sqrt(1.4 * 287 * tt); // 计算声速
    }
    else
    {
        tt = Get_Atmo_tt(H);
        pp = Get_Atmo_pp(H);
        rho = Get_Atmo_rr(pp, tt);
        aa = Get_Atmo_aa(tt);
    }

    // 确保aa不为零
    if (fabs(aa) < 1e-6) {
        aa = 1e-6; // 防止除零错误
    }

    Ma = V / aa; // 计算马赫数

    // 计算侧滑角，并增加边界检查
    sinbeta = cos(Theta) * (cos(Gama) * sin(Psi - Psi_v) + sin(THETA) * sin(Gama) * cos(Psi - Psi_v)) - sin(Theta) * cos(THETA) * sin(Gama);
    // 边界检查防止asin接收无效输入
    if (sinbeta > 1.0) sinbeta = 1.0;
    if (sinbeta < -1.0) sinbeta = -1.0;
    beta = asin(sinbeta);    // 侧滑角

    // 计算攻角，增加除零保护和边界检查
    double cos_beta = cos(beta);
    if (fabs(cos_beta) < 1e-6) {
        // 防止除零
        cos_beta = (cos_beta >= 0) ? 1e-6 : -1e-6;
    }

    sinalfa = (cos(Theta) * (sin(THETA) * cos(Gama) * cos(Psi - Psi_v) - sin(Gama) * sin(Psi - Psi_v)) - sin(Theta) * cos(THETA) * cos(Gama)) / cos_beta;
    // 边界检查防止asin接收无效输入
    if (sinalfa > 1.0) sinalfa = 1.0;
    if (sinalfa < -1.0) sinalfa = -1.0;
    alfa = asin(sinalfa);    // 攻角

    // 计算bank angle，增加边界检查和正确的公式
    singamv = (cos(alfa) * sin(beta) * sin(THETA) - sin(alfa) * sin(beta) * cos(Gama) * cos(THETA) + cos(beta) * sin(Gama) * cos(THETA)) / cos(Theta);
    // 检查cos(Theta)是否接近零
    double cos_theta = cos(Theta);
    if (fabs(cos_theta) < 1e-6) {
        // 防止除零
        cos_theta = (cos_theta >= 0) ? 1e-6 : -1e-6;
    }
    singamv = (cos(alfa) * sin(beta) * sin(THETA) - sin(alfa) * sin(beta) * cos(Gama) * cos(THETA) + cos(beta) * sin(Gama) * cos(THETA)) / cos_theta;

    // 边界检查防止asin接收无效输入
    if (singamv > 1.0) singamv = 1.0;
    if (singamv < -1.0) singamv = -1.0;
    gamv = asin(singamv);  // Bank angle

    // 防止加速度计算中的NaN
    double g_safe = g00;
    if (fabs(g_safe) < 1e-6) {
        g_safe = 1e-6; // 防止除零
    }

    nx2 = d[2] / g_safe + sin(Theta);           // 体坐标系中的加速度

    // 防止V为零或接近零导致的计算问题
    double V_safe = V;
    if (fabs(V_safe) < 1e-6) {
        V_safe = 1e-6; // 防止除零
    }

    ny2 = V_safe * d[3] / g_safe + cos(Theta);
    nz2 = -V_safe * cos(Theta) * d[4] / g_safe;

    dy->GetForce_Moment_Motion(ma, this, Gdata);    // 获取气动力和力矩

    // 体坐标系中的表达式(未使用)
    Rx1 = 0.5 * rho * V * V * ma->Sm * (dy->Cx);
    Ry1 = 0.5 * rho * V * V * ma->Sm * dy->Cy;
    Rz1 = 0.5 * rho * V * V * ma->Sm * (dy->Cz);

    // 速度坐标系中的力
    Rx3 = 0.5 * rho * V * V * ma->Sm * dy->CD;       // 阻力
    Ry3 = 0.5 * rho * V * V * ma->Sm * dy->CL;       // 升力
    Rz3 = 0.5 * rho * V * V * ma->Sm * dy->CZ;       // 侧向力

    // 轨迹坐标系中的投影
    Rx2 = -Rx3;                                     // 前向方向的力
    Ry2 = Ry3 * cos(gamv) - Rz3 * sin(gamv);        // 法向方向的力
    Rz2 = Ry3 * sin(gamv) + Rz3 * cos(gamv);        // 侧向方向的力

    // 基于飞行阶段的RPM控制
    if (Gdata->flag_Fp == 1 && en->use_rpm_model == 1)
    {
        // 首先更新飞行阶段(如果需要)
        nc->UpdateFlightPhase(this);

        // 根据当前飞行阶段设置适当的RPM
        switch (nc->current_phase) {
        case CNCModel::TAKEOFF:
            // 起飞阶段需要更高的推力
            en->SetRPM(7000);
            break;

        case CNCModel::CLIMB:
            // 爬升阶段需要中等推力
            en->SetRPM(6000);
            break;

        case CNCModel::CRUISE:
            // 巡航阶段可以使用较低的推力以节省燃料
            en->SetRPM(4000);
            break;

        case CNCModel::DESCENT:
            // 下降阶段可以使用最小推力
            en->SetRPM(3000);
            break;

        case CNCModel::TERMINAL:
            // 终端阶段需要更精确的速度控制
            if (nc->terminal_cruise_enabled) {
                // 在终端巡航模式下根据速度和高度调整推力
                if (V < nc->target_speed - 1.0) {
                    // 速度不足，增加推力
                    en->SetRPM(4500);
                }
                else if (V > nc->target_speed + 1.0) {
                    // 速度过高，减少推力
                    en->SetRPM(3500);
                }
                else {
                    // 速度适中，维持推力
                    en->SetRPM(4000);
                }
            }
            else {
                // 常规终端阶段
                en->SetRPM(4000);
            }
            break;

        default:
            // 默认使用中等RPM
            en->SetRPM(4000);
            break;
        }

        // 当前飞行阶段和RPM状态的调试输出
        if (int(t * 10) % 50 == 0) { // 每5秒
            cout << "t=" << t << "s, 当前阶段=" << nc->current_phase
                << ", 高度=" << Y << "m, 速度=" << V
                << "m/s, RPM=" << en->rpm_current << "rpm"
                << ", 推力=" << en->Fp << "N"
                << ", 燃料消耗率=" << en->dm_dt_current << "g/s";

            // 如果在终端巡航模式，显示额外信息
            if (nc->terminal_cruise_enabled) {
                cout << ", 终端巡航时间: " << nc->terminal_cruise_duration << "s";
            }
            cout << endl;
        }
    }

    // 获取基于当前条件的推力值
    if (Gdata->flag_Fp == 1)
    {
        // 添加推力计算前诊断
        if (int(t * 100) % 100 == 0) { // 每秒输出一次诊断信息
            printf("推力计算前: 高度=%.1f米, 速度=%.1f米/秒, RPM=%.1f\n",
                H, V, en->rpm_current);
        }

        en->Get_Fp(Gdata, this);

        // 添加推力计算后诊断
        if (int(t * 100) % 100 == 0 || en->Fp <= 0.01) {
            printf("推力计算结果: Fp=%.1f牛顿, dm_dt=%.4f克/秒\n",
                en->Fp, en->dm_dt_current);

            if (en->Fp <= 0.01) {
                printf("警告: 推力接近零! 检查发动机插值表和RPM设置 (t=%.2f, RPM=%.1f)\n",
                    t, en->rpm_current);
            }
        }

        if (t >= en->t_Fp_begin && t <= en->t_Fp_end)
        {
            // 修正推力分解公式，增加NaN值检查
            if (isnan(alfa) || isnan(beta)) {
                // 如果攻角或侧滑角是NaN，使用安全的默认值
                Fpx2 = en->Fp; // 默认前向推力
                Fpy2 = 0.0;
                Fpz2 = 0.0;
                // 输出警告
                if (int(t * 100) % 100 == 0) { // 限制警告频率
                    cout << "警告: 推力计算中检测到无效角度。使用默认推力方向。" << endl;
                }
            }
            else {
                // 正常计算推力分量
                Fpx2 = en->Fp * cos(alfa) * cos(beta);
                Fpy2 = en->Fp * sin(alfa);
                Fpz2 = -en->Fp * cos(alfa) * sin(beta);
            }
        }
        else {
            // 不在推力应用时间范围内
            Fpx2 = Fpy2 = Fpz2 = 0.0;
            if (int(t * 100) % 200 == 0) { // 每2秒输出一次
                printf("注意: 当前时间t=%.2f不在推力应用范围[%.1f,%.1f]内\n",
                    t, en->t_Fp_begin, en->t_Fp_end);
            }
        }
    }
    else {
        // 没有应用推力模型
        Fpx2 = Fpy2 = Fpz2 = 0.0;
    }

    // 每10秒输出一次详细推力信息
    if (int(t * 10) % 100 == 0) { // 每10秒详细信息
        cout << "== 详细推力信息 ==" << endl;
        cout << "时间=" << t << "s, 阶段=" << nc->current_phase << endl;
        cout << "高度=" << Y << "m, 速度=" << V << "m/s, 马赫数=" << Ma << endl;
        cout << "RPM=" << en->rpm_current << "rpm, 总推力=" << en->Fp << "N" << endl;
        cout << "推力分量: Fx=" << Fpx2 << "N, Fy=" << Fpy2 << "N, Fz=" << Fpz2 << "N" << endl;
        cout << "升力=" << Ry3 << "N, 阻力=" << -Rx3 << "N" << endl;
        cout << "质量=" << m << "kg, 燃料消耗率=" << en->dm_dt_current << "g/s" << endl;
        cout << "========================" << endl;
    }

    // 轨迹坐标系中的投影
    Gx2 = -ma->m * g00 * sin(Theta);
    Gy2 = -ma->m * g00 * cos(Theta);
    Gz2 = 0;

    // 体坐标系中的表达式
    Mx_static = 0.5 * rho * V * V * ma->Sm * ma->Lk * (dy->Mx);              // 静态滚转力矩
    Mx_Wx = 0.5 * rho * V * ma->Sm * ma->Lk * ma->Lk * dy->Mx_wx * Wx;       // 动态阻尼
    Mx_Wy = 0.5 * rho * V * ma->Sm * ma->Lk * ma->Lk * dy->Mx_wy * Wy;
    Mx_Wz = 0.5 * rho * V * ma->Sm * ma->Lk * ma->Lk * dy->Mx_wz * Wz;

    My_static = 0.5 * rho * V * V * ma->Sm * ma->Lk * (dy->My);              // 静态俯仰力矩
    My_Wx = 0.5 * rho * V * ma->Sm * ma->Lk * ma->Lk * dy->My_wx * Wx;       // 动态阻尼
    My_Wy = 0.5 * rho * V * ma->Sm * ma->Lk * ma->Lk * dy->My_wy * Wy;
    My_Wz = 0.5 * rho * V * ma->Sm * ma->Lk * ma->Lk * dy->My_wz * Wz;

    Mz_static = 0.5 * rho * V * V * ma->Sm * ma->Lk * (dy->Mz);              // 静态偏航力矩
    Mz_Wx = 0.5 * rho * V * ma->Sm * ma->Lk * ma->Lk * dy->Mz_wx * Wx;       // 动态阻尼
    Mz_Wy = 0.5 * rho * V * ma->Sm * ma->Lk * ma->Lk * dy->Mz_wy * Wy;
    Mz_Wz = 0.5 * rho * V * ma->Sm * ma->Lk * ma->Lk * dy->Mz_wz * Wz;

    if (Gdata->flag_dy == 1)        // 动态模型
    {
        MX = Mx_static + Mx_Wx;                                   // 总滚转力矩
        MY = My_static + My_Wy;                                   // 总俯仰力矩
        MZ = Mz_static + Mz_Wz;                                   // 总偏航力矩
    }
    else if (Gdata->flag_dy == 2)
    {
        MX = Mx_static + Mx_Wx + Mx_Wy;                               // 总滚转力矩
        MY = My_static + My_Wx + My_Wy;                               // 总俯仰力矩
        MZ = Mz_static + Mz_Wz;                                   // 总偏航力矩
    }
    else if (Gdata->flag_dy == 3)
    {
        MX = Mx_static + Mx_Wx + Mx_Wy + Mx_Wz;                               // 总滚转力矩
        MY = My_static + My_Wx + My_Wy + My_Wz;                               // 总俯仰力矩
        MZ = Mz_static + Mz_Wx + Mz_Wy + Mz_Wz;                               // 总偏航力矩
    }

    nx2 = (Fpx2 + Rx2) / (ma->m * g00);
    ny2 = (Fpy2 + Ry2) / (ma->m * g00);
    nz2 = (Fpz2 + Rz2) / (ma->m * g00);

    nx = nx2 * cos(alfa) * cos(beta) + ny2 * (sin(alfa) * cos(gamv) + cos(alfa) * sin(beta) * sin(gamv)) + nz2 * (sin(alfa) * sin(gamv) - cos(alfa) * sin(beta) * cos(gamv));   // 体坐标系中的投影
    ny = -nx2 * sin(alfa) * cos(beta) + ny2 * (cos(alfa) * cos(gamv) - sin(alfa) * sin(beta) * sin(gamv)) + nz2 * (cos(alfa) * sin(gamv) + sin(alfa) * sin(beta) * cos(gamv));
    nz = nx2 * sin(beta) - ny2 * (cos(beta) * sin(gamv)) + nz2 * cos(beta) * cos(gamv);

    // 记录气动参数用于后续分析
    dy_CD = dy->CD;
    dy_CL = dy->CL;
    dy_CZ = dy->CZ;
    dy_Mz = dy->Mz;
    dy_Mz_dong = dy->Mz_wz;
    dy_My = dy->My;
    dy_My_dong = dy->My_wy;
    dy_Mx = dy->Mx;
    dy_Mx_dong = dy->Mx_wx;
    dy_Xcp = dy->Mz / dy->Cy;/////////////////////
    dy_P_P0 = pp / pp0;
    dy_R_R0 = rho / rho0;

    Qv = 0.5 * rho * V * V;
}

void CMotiModel::Motion(CMassModel* ma, CEngiModel* en, CDynaModel* dy, Data_Parameter* Gdata, CNCModel* nc, double hStep) // Motion dynamics modeling
{

	RenewStus(ma, en, dy, Gdata, nc);

	Gdata->time = t;

		switch (Gdata->flag_3)		// End parameter judgment
		{ 
		case 1:
			end_parameter = X;
			break;
		case 2:
			end_parameter = Y;
			break;
		case 3:
			end_parameter = Z;
			break;
		case 4:
			end_parameter = t;
			break;
		}
		
		switch (Gdata->flag_4)
		{
		case 1:
			if (end_parameter - Gdata->H_end > 0)          // End condition judgment, when height is less than 10
			{
				Gdata->Kfly = 2;
				cout << "Coordinate transformation matrix update completed" << endl;
			}
			break;
		case 2:
			if (end_parameter - Gdata->H_end < 0)          // End condition judgment, when height is less than 10
			{
				Gdata->Kfly = 2;
				cout << "Coordinate transformation matrix update completed" << endl;
			}
			break;
		case 3:
			if (end_parameter - Gdata->H_end == 0)          // End condition judgment, when height equals to the target
			{
				Gdata->Kfly = 2;
				cout << "Coordinate transformation matrix update completed" << endl;
			}
			break;
		}
	return;
}

