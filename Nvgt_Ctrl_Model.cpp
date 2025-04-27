#define _CRT_SECURE_NO_WARNINGS
#include "Nvgt_Ctrl_Model.h"
#include "FileUtils.h"
#include <string.h>
#include <iostream>

// If _countof is not defined, define it
#ifndef _countof
#define _countof(array) (sizeof(array)/sizeof(array[0]))
#endif

using namespace std;

///////////       Implementation of CNCModel class       ///////////////
CNCModel::CNCModel()
{
    Num_Ctrl = 100; // Initialize control points array size
    
    // Default to trajectory pitch angle control mode
    is_theta_control = true;  
    theta_target = 30.0;      // Initial target trajectory pitch angle in degrees

    // Initialize flight phase parameters
    current_phase = TAKEOFF;
    climb_target_height = 4000.0;    // Climbing target height 4000m
    descent_target_height = 1000.0;  // Descent target height 1000m
    climb_rate = 5.0;                // Target climb rate 5m/s
    target_speed = 50.0;             // Target speed 50m/s
    
    // Initialize terminal cruise parameters
    terminal_cruise_enabled = false;
    terminal_cruise_duration = 0.0;
    terminal_phase_start_time = 0.0;
    
    // Initialize trajectory control gains with more aggressive values
    Kp_theta = 2.0;       // Proportional gain for trajectory pitch angle control
    Ki_theta = 0.1;       // Integral gain
    Kd_theta = 0.5;       // Derivative gain
    theta_error_sum = 0.0; // Cumulative error for integral term
    prev_theta_error = 0.0; // Previous error for derivative term
}


CNCModel::~CNCModel()
{ }

void CNCModel::InputData_NC()
{
    FILE* input;
    char str[256]; // 增大缓冲区尺寸
    int num_points = 0;

    // 使用通用文件打开函数
    if ((input = tryOpenFile("Control.dat")) == NULL)
    {
        printf("Cannot open input file Control.dat\n");
        return;
    }

    // 读取控制模式设置 (THETA_CONTROL)
    if (fgets(str, sizeof(str), input) == NULL) {
        printf("Error reading control mode\n");
        fclose(input);
        return;
    }

    // 去除行尾的换行符
    str[strcspn(str, "\r\n")] = 0;

    // 确定控制类型 - 轨迹俯仰角或攻角控制
    if (strcmp(str, "THETA_CONTROL") == 0 || strcmp(str, "theta_control") == 0)
    {
        is_theta_control = true;
        cout << "Using trajectory pitch angle (Theta) control mode" << endl;
    }
    else
    {
        is_theta_control = false;
        cout << "Using angle of attack (alpha) control mode" << endl;
    }

    // 读取数据点数量
    if (fscanf(input, "%d", &num_points) != 1 || num_points <= 0) {
        printf("Error reading number of control points or invalid number\n");
        fclose(input);
        return;
    }

    // 更新实际控制点数量
    Num_Ctrl = num_points;
    printf("Reading %d control points\n", Num_Ctrl);

    // 清空缓冲区，读取标题行
    fgets(str, sizeof(str), input);  // 读取换行符
    fgets(str, sizeof(str), input);  // 读取标题行
    printf("Control header: %s", str);

    // 读取控制数据点
    for (int i = 0; i < Num_Ctrl; i++) {
        if (fscanf(input, "%lf%lf", &data_t_NC[i], &data_alfa_NC[i]) != 2) {
            printf("Error reading control point %d, stopping at %d points\n", i + 1, i);
            Num_Ctrl = i;
            break;
        }

        // 调试输出前几个点
        if (i < 3) {
            printf("Control point %d: t=%.1f, theta=%.1f degrees\n",
                i + 1, data_t_NC[i], data_alfa_NC[i]);
        }
    }

    fclose(input);

    // 设置控制时间范围
    if (Num_Ctrl > 0) {
        t_NC_begin = data_t_NC[0];
        t_NC_end = data_t_NC[Num_Ctrl - 1];

        printf("Navigation control time range: %.1f to %.1f seconds\n",
            t_NC_begin, t_NC_end);
    }
    else {
        printf("Warning: No control points read!\n");
        t_NC_begin = t_NC_end = 0.0;
    }
}

void CNCModel::Get_Alfa_Ctrl(Data_Parameter* Gdata, CMotiModel* mo)
{
    // Traditional angle of attack control mode
    t = mo->y[0];
    alfa_NC = LAQL1(Num_Ctrl, data_t_NC, data_alfa_NC, t);  // Get control angle from data table
    alfa_NC = alfa_NC * DtR;  // Convert to radians
    
    // Make sure alfa_NC is within reasonable bounds
    double max_alpha = 15.0 * DtR; // 15 degrees max
    if (alfa_NC > max_alpha) alfa_NC = max_alpha;
    if (alfa_NC < -max_alpha) alfa_NC = -max_alpha;
}

void CNCModel::Get_Theta_Ctrl(Data_Parameter* Gdata, CMotiModel* mo)
{
    // 确保theta_target总是有有效值
    if (isnan(theta_target) || fabs(theta_target) < 1e-6) {
        printf("发现theta_target异常值(%.6f)，重置为默认值\n", theta_target);
        // 根据当前飞行阶段设置默认值
        switch (current_phase) {
        case TAKEOFF:
            theta_target = 30.0 * DtR;
            break;
        case CLIMB:
            theta_target = 6.0 * DtR;
            break;
        case CRUISE:
            theta_target = 0.0 * DtR;
            break;
        case DESCENT:
            theta_target = -2.0 * DtR;
            break;
        case TERMINAL:
            theta_target = 0.0 * DtR;
            break;
        default:
            theta_target = 0.0 * DtR;
        }
        printf("theta_target重置为: %.1f度\n", theta_target / DtR);
    }
    
    // 强制使用当前仿真时间
    double current_time = mo->y[0];

    // 调试信息输出
    printf("\n===== THETA控制调试信息 (t=%.3f) =====\n", current_time);

    // 1. 更新飞行阶段
    UpdateFlightPhase(mo);

    // 2. 从控制数据表中计算插值
    double interpolated_theta = 0.0;

    // 确保控制点数组有效
    if (Num_Ctrl <= 0) {
        printf("警告: 无控制点数据, 使用默认值30度\n");
        interpolated_theta = 30.0;
    }
    else if (current_time <= data_t_NC[0]) {
        interpolated_theta = data_alfa_NC[0];
        printf("时间%.3f秒在首点前, 使用起始值%.1f度\n", current_time, interpolated_theta);
    }
    else if (current_time >= data_t_NC[Num_Ctrl - 1]) {
        interpolated_theta = data_alfa_NC[Num_Ctrl - 1];
        printf("时间%.3f秒在终点后, 使用终值%.1f度\n", current_time, interpolated_theta);
    }
    else {
        // 找到时间所在区间
        int idx = 0;
        while (idx < Num_Ctrl - 1 && current_time > data_t_NC[idx + 1]) {
            idx++;
        }

        // 线性插值
        double t_ratio = (current_time - data_t_NC[idx]) / (data_t_NC[idx + 1] - data_t_NC[idx]);
        interpolated_theta = data_alfa_NC[idx] + t_ratio * (data_alfa_NC[idx + 1] - data_alfa_NC[idx]);

        printf("插值: 时间%.3f在[%.1f,%.1f]内, 角度在[%.1f,%.1f]内, 计算结果=%.1f度\n",
            current_time, data_t_NC[idx], data_t_NC[idx + 1],
            data_alfa_NC[idx], data_alfa_NC[idx + 1], interpolated_theta);
    }

    // 重要: 直接将插值结果转换为弧度并赋值给theta_target
    // 在此赋值前保存当前值用于调试
    double previous_target = theta_target;
    theta_target = interpolated_theta * DtR;

    printf("theta_target更新: %.1f度 -> %.1f度\n", previous_target / DtR, theta_target / DtR);

    // 3. 根据飞行阶段调整控制参数
    double pre_adjust_target = theta_target;
    AdjustControlForPhase(Gdata, mo);

    if (fabs(theta_target - pre_adjust_target) > 0.001) {
        printf("阶段调整修改了目标角度: %.1f度 -> %.1f度\n", pre_adjust_target / DtR, theta_target / DtR);
    }

    // 4. 计算控制误差
    double theta_error = theta_target - mo->Theta;

    // 5. PID控制
    double p_term = Kp_theta * theta_error;

    // 积分项计算
    if (fabs(theta_error) < 15.0 * DtR) {
        theta_error_sum += theta_error * 0.1;
    }
    double max_i_sum = 10.0 * DtR;
    if (theta_error_sum > max_i_sum) theta_error_sum = max_i_sum;
    if (theta_error_sum < -max_i_sum) theta_error_sum = -max_i_sum;

    double i_term = Ki_theta * theta_error_sum;
    double d_term = Kd_theta * (theta_error - prev_theta_error) / 0.1;
    prev_theta_error = theta_error;

    // 6. 计算所需攻角命令
    double previous_alfa_nc = alfa_NC;
    alfa_NC = p_term + i_term + d_term;

    // 7. 添加姿态补偿
    double attitude_comp = 0.5 * (mo->THETA - mo->Theta);
    alfa_NC += attitude_comp;

    // 8. 限制最大攻角变化
    double max_alfa = 20.0 * DtR;
    if (alfa_NC > max_alfa) alfa_NC = max_alfa;
    if (alfa_NC < -max_alfa) alfa_NC = -max_alfa;

    printf("PID: P=%.2f, I=%.2f, D=%.2f, 补偿=%.2f, 总和=%.2f度\n",
        p_term / DtR, i_term / DtR, d_term / DtR, attitude_comp / DtR, alfa_NC / DtR);
    printf("最终命令: 目标角=%.1f度, 当前角=%.1f度, 攻角命令=%.1f度\n",
        theta_target / DtR, mo->Theta / DtR, alfa_NC / DtR);
    printf("=======================================\n");
}

void CNCModel::UpdateFlightPhase(CMotiModel* mo)
{
    double current_time = mo->y[0];
    double altitude = mo->Y;

    // 保存旧状态
    FlightPhase old_phase = current_phase;

    // 根据时间和控制文件的切换点决定飞行阶段
    if (current_time < 2.0) {
        current_phase = TAKEOFF;
        printf("当前为起飞阶段 (t=%.2f)\n", current_time);
    }
    else if (current_time < 700.0) {
        current_phase = CLIMB;
        printf("当前为爬升阶段 (t=%.2f)\n", current_time);
    }
    else if (current_time < 730.0) {
        current_phase = CRUISE;
        printf("当前为巡航阶段 (t=%.2f)\n", current_time);
    }
    else if (current_time < 852.0) {
        current_phase = DESCENT;
        printf("当前为下降阶段 (t=%.2f)\n", current_time);
    }
    else {
        current_phase = TERMINAL;
        printf("当前为终端阶段 (t=%.2f)\n", current_time);
    }

    // 如果飞行阶段发生变化，输出通知
    if (old_phase != current_phase) {
        printf("飞行阶段变化: %d -> %d (t=%.3f)\n", old_phase, current_phase, current_time);

        // 强制进行一次插值计算，确保目标角度与阶段变化同步
        double target_after_change = 0.0;
        if (Num_Ctrl > 0) {
            // 简化的插值查找
            for (int i = 0; i < Num_Ctrl - 1; i++) {
                if (data_t_NC[i] <= current_time && current_time <= data_t_NC[i + 1]) {
                    double t_ratio = (current_time - data_t_NC[i]) / (data_t_NC[i + 1] - data_t_NC[i]);
                    target_after_change = data_alfa_NC[i] + t_ratio * (data_alfa_NC[i + 1] - data_alfa_NC[i]);
                    printf("阶段变化后的目标角度(插值): %.1f度\n", target_after_change);
                    break;
                }
            }
        }
    }
}

void CNCModel::AdjustControlForPhase(Data_Parameter* Gdata, CMotiModel* mo)
{
    printf("阶段调整前: theta_target=%.1f度\n", theta_target / DtR);

    // 根据飞行阶段强制设置目标值
    switch (current_phase) {
    case TAKEOFF:
        // 起飞阶段固定使用30度
        if (fabs(theta_target / DtR - 30.0) > 0.5) {
            printf("强制设置起飞目标角度为30度\n");
            theta_target = 30.0 * DtR;
        }
        break;

    case CLIMB:
        // 爬升阶段固定使用6度
        if (fabs(theta_target / DtR - 6.0) > 0.5) {
            printf("强制设置爬升目标角度为6度\n");
            theta_target = 6.0 * DtR;
        }
        break;

    case CRUISE:
        // 巡航阶段固定使用0度
        if (fabs(theta_target / DtR) > 0.5) {
            printf("强制设置巡航目标角度为0度\n");
            theta_target = 0.0 * DtR;
        }
        break;

    case DESCENT:
        // 下降阶段固定使用-2度
        if (fabs(theta_target / DtR + 2.0) > 0.5) {
            printf("强制设置下降目标角度为-2度\n");
            theta_target = -2.0 * DtR;
        }
        break;

    case TERMINAL:
        // 终端阶段固定使用0度
        if (fabs(theta_target / DtR) > 0.5) {
            printf("强制设置终端目标角度为0度\n");
            theta_target = 0.0 * DtR;
        }
        break;
    }

    printf("阶段调整后: theta_target=%.1f度\n", theta_target / DtR);
}