#define _CRT_SECURE_NO_WARNINGS
#include "EngiModel.h"
#include "FileUtils.h"  // Added FileUtils.h header
#include <string.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm> // For std::sort

using namespace std;

// Declare LAQL1 function (already defined in laq.cpp)
double LAQL1(int m, double* data_m, double* data_C, double x);

// Define _countof if not defined
#ifndef _countof
#define _countof(array) (sizeof(array)/sizeof(array[0]))
#endif

/**********************************
Engine Model
***********************************/
//////////       CEngiModel Class Implementation     ///////////////////
////////////////////////////////////////////////////////////
CEngiModel::CEngiModel()
{
    Fp = 0.0;
    dm_dt_current = 0.0;
    rpm_current = 4000.0; // Default RPM
    
    Num_Fp = 0;
    t_Fp_begin = 0.0;
    t_Fp_end = 0.0;
    
    Num_H_Fp = 0;
    Num_V_Fp = 0;
    Num_RPM_Fp = 0;
    
    data_H_Fp = nullptr;
    data_V_Fp = nullptr;
    data_RPM_Fp = nullptr;
    data_Fp_3d = nullptr;
    data_dm_dt = nullptr;
    
    use_rpm_model = 1; // Default: use 3D interpolation model (based on H, V, RPM)
}

CEngiModel::~CEngiModel()
{
    // Free dynamically allocated memory
    if (data_H_Fp) delete[] data_H_Fp;
    if (data_V_Fp) delete[] data_V_Fp;
    if (data_RPM_Fp) delete[] data_RPM_Fp;
    if (data_Fp_3d) delete[] data_Fp_3d;
    if (data_dm_dt) delete[] data_dm_dt;
}


void CEngiModel::InputData_En()
{
    if (!readEngineDataFromFile()) {
        printf("Failed to read engine data file\n");
    }
}

bool CEngiModel::readEngineDataFromFile() {
    FILE* input = nullptr;
    char str[80];
    char header[80];
    int rpmColumnIndex = -1;
    int totalDataCount = 0;
    double h, v, rpm, f, dm;
    int i;

    try {
        // Use the common file opening function
        if ((input = tryOpenFile("Engine.dat")) == NULL) {
            printf("Cannot open input file Engine.dat\n");
            return false;
        }

        cout << "Reading engine data file..." << endl;

        // Read file header
        if (fscanf(input, "%s", str) != 1) {
            cout << "Error reading file header" << endl;
            fclose(input);
            return false;
        }
        cout << "Engine data file header: " << str << endl;

        // Read total data count
        if (fscanf(input, "%d", &totalDataCount) != 1) {
            cout << "Error reading total data count" << endl;
            fclose(input);
            return false;
        }
        cout << "Total data points: " << totalDataCount << endl;

        if (totalDataCount <= 0) {
            cout << "Invalid data count: " << totalDataCount << endl;
            fclose(input);
            return false;
        }

        // Read column headers, find RPM column
        for (i = 0; i < 5; i++) {
            if (fscanf(input, "%s", header) != 1) {
                cout << "Error reading column header " << i << endl;
                fclose(input);
                return false;
            }
            cout << "Column " << i << " header: " << header << endl;
            
            // Use strstr to find "rpm" or "RPM"
            if (strstr(header, "rpm") != NULL || strstr(header, "RPM") != NULL) {
                rpmColumnIndex = i;
                cout << "RPM column found at index " << i << endl;
            }
        }

        // If RPM column found, read data
        if (rpmColumnIndex >= 0) {
            // Allocate memory to store data
            data_h = new double[totalDataCount];
            data_v = new double[totalDataCount];
            data_rpm = new double[totalDataCount];
            data_f = new double[totalDataCount];
            data_dm = new double[totalDataCount];
            
            // Read data
            int validDataCount = 0;
            for (i = 0; i < totalDataCount; i++) {
                if (fscanf(input, "%lf%lf%lf%lf%lf", &h, &v, &rpm, &f, &dm) != 5) {
                    cout << "Error reading data row " << i << endl;
                    break;
                }
                
                // Validate data
                if (!isfinite(h) || !isfinite(v) || !isfinite(rpm) || !isfinite(f) || !isfinite(dm)) {
                    cout << "Warning: Invalid data at row " << i << ". Skipping." << endl;
                    continue;
                }
                
                data_h[validDataCount] = h;
                data_v[validDataCount] = v;
                data_rpm[validDataCount] = rpm;
                data_f[validDataCount] = f;
                data_dm[validDataCount] = dm;
                validDataCount++;
                
                // Print some sample data
                if (i < 5 || i > totalDataCount - 5) {
                    cout << "Data[" << i << "]: h=" << h << ", v=" << v << ", rpm=" << rpm 
                         << ", f=" << f << ", dm=" << dm << endl;
                }
            }
            
            Num_Engi = validDataCount; // Actual number of data points read
            cout << "Successfully read " << Num_Engi << " valid data points" << endl;
            
            if (Num_Engi <= 0) {
                cout << "Error: No valid data points read" << endl;
                fclose(input);
                return false;
            }
            
            // 初始化3D插值所需的数据
            // 提取唯一的高度值
            std::vector<double> unique_heights;
            for (i = 0; i < validDataCount; i++) {
                bool exists = false;
                for (size_t j = 0; j < unique_heights.size(); j++) {
                    if (fabs(data_h[i] - unique_heights[j]) < 1e-6) {
                        exists = true;
                        break;
                    }
                }
                if (!exists) {
                    unique_heights.push_back(data_h[i]);
                }
            }
            
            // 提取唯一的速度值
            std::vector<double> unique_velocities;
            for (i = 0; i < validDataCount; i++) {
                bool exists = false;
                for (size_t j = 0; j < unique_velocities.size(); j++) {
                    if (fabs(data_v[i] - unique_velocities[j]) < 1e-6) {
                        exists = true;
                        break;
                    }
                }
                if (!exists) {
                    unique_velocities.push_back(data_v[i]);
                }
            }
            
            // 提取唯一的RPM值
            std::vector<double> unique_rpms;
            for (i = 0; i < validDataCount; i++) {
                bool exists = false;
                for (size_t j = 0; j < unique_rpms.size(); j++) {
                    if (fabs(data_rpm[i] - unique_rpms[j]) < 1e-6) {
                        exists = true;
                        break;
                    }
                }
                if (!exists) {
                    unique_rpms.push_back(data_rpm[i]);
                }
            }
            
            // 排序
            std::sort(unique_heights.begin(), unique_heights.end());
            std::sort(unique_velocities.begin(), unique_velocities.end());
            std::sort(unique_rpms.begin(), unique_rpms.end());
            
            // 设置3D插值数据
            Num_H_Fp = static_cast<int>(unique_heights.size());
            Num_V_Fp = static_cast<int>(unique_velocities.size());
            Num_RPM_Fp = static_cast<int>(unique_rpms.size());
            
            cout << "Unique heights: " << Num_H_Fp << endl;
            cout << "Unique velocities: " << Num_V_Fp << endl;
            cout << "Unique RPMs: " << Num_RPM_Fp << endl;
            
            // 分配内存
            data_H_Fp = new double[Num_H_Fp];
            data_V_Fp = new double[Num_V_Fp];
            data_RPM_Fp = new double[Num_RPM_Fp];
            
            // 拷贝唯一值
            for (i = 0; i < Num_H_Fp; i++) {
                data_H_Fp[i] = unique_heights[i];
            }
            
            for (i = 0; i < Num_V_Fp; i++) {
                data_V_Fp[i] = unique_velocities[i];
            }
            
            for (i = 0; i < Num_RPM_Fp; i++) {
                data_RPM_Fp[i] = unique_rpms[i];
            }
            
            // 分配3D数据数组
            int totalSize = Num_H_Fp * Num_V_Fp * Num_RPM_Fp;
            data_Fp_3d = new double[totalSize];
            data_dm_dt = new double[totalSize];
            
            // 初始化为0
            for (i = 0; i < totalSize; i++) {
                data_Fp_3d[i] = 0.0;
                data_dm_dt[i] = 0.0;
            }
            
            // 填充3D数据
            for (i = 0; i < validDataCount; i++) {
                int h_index = -1, v_index = -1, rpm_index = -1;
                
                // 找到索引
                for (int j = 0; j < Num_H_Fp; j++) {
                    if (fabs(data_h[i] - data_H_Fp[j]) < 1e-6) {
                        h_index = j;
                        break;
                    }
                }
                
                for (int j = 0; j < Num_V_Fp; j++) {
                    if (fabs(data_v[i] - data_V_Fp[j]) < 1e-6) {
                        v_index = j;
                        break;
                    }
                }
                
                for (int j = 0; j < Num_RPM_Fp; j++) {
                    if (fabs(data_rpm[i] - data_RPM_Fp[j]) < 1e-6) {
                        rpm_index = j;
                        break;
                    }
                }
                
                if (h_index >= 0 && v_index >= 0 && rpm_index >= 0) {
                    int index = h_index + v_index * Num_H_Fp + rpm_index * Num_H_Fp * Num_V_Fp;
                    data_Fp_3d[index] = data_f[i];
                    data_dm_dt[index] = data_dm[i];
                }
            }
            
            // 启用RPM模型
            use_rpm_model = 1;
            cout << "3D thrust interpolation data initialized successfully" << endl;
        } else {
            // If RPM column not found, skip this part of data
            cout << "RPM column not found in headers. Using default thrust model." << endl;
            fscanf(input, "%s", str); // Skip data section
        }

        fclose(input);
        cout << "Engine data file read complete" << endl;
        return true;
    }
    catch (const std::exception& e) {
        cout << "Exception during engine data file reading: " << e.what() << endl;
        if (input) fclose(input);
        return false;
    }
    catch (...) {
        cout << "Unknown exception during engine data file reading" << endl;
        if (input) fclose(input);
        return false;
    }
}

void CEngiModel::Get_Fp(Data_Parameter* Gdata, CMotiModel* mo)
{
    // 检查参数有效性
    if (mo == nullptr || Gdata == nullptr) {
        cout << "Error: Null pointer passed to Get_Fp" << endl;
        Fp = 0.0;
        dm_dt_current = 0.0;
        return;
    }

    t = mo->y[0]; // Current time
    
    try {
        // 初始化推力和燃料消耗率为0，避免未初始化导致的随机值
        Fp = 0.0;
        dm_dt_current = 0.0;
        
        double height = mo->y[11]; // Current height
        double velocity = mo->V;   // Current velocity
        
        // 输出调试信息
        cout << "Debug: Calculating thrust at height=" << height 
             << ", velocity=" << velocity
             << ", rpm=" << rpm_current << endl;
        
        // 检查数据有效性，替换非法值
        if (!isfinite(height)) {
            height = 0.0;
            cout << "Warning: Invalid height value, using 0" << endl;
        }
        
        if (!isfinite(velocity)) {
            velocity = 0.0;
            cout << "Warning: Invalid velocity value, using 0" << endl;
        }
        
        if (!isfinite(rpm_current)) {
            rpm_current = 4000.0;  // Default RPM
            cout << "Warning: Invalid RPM value, using 4000" << endl;
        }
        
        if (use_rpm_model == 1 && Num_H_Fp > 0 && Num_V_Fp > 0 && Num_RPM_Fp > 0 &&
            data_H_Fp != nullptr && data_V_Fp != nullptr && data_RPM_Fp != nullptr && 
            data_Fp_3d != nullptr && data_dm_dt != nullptr) {
            
            // 确保输入值在有效范围内
            if (height < data_H_Fp[0]) height = data_H_Fp[0];
            if (height > data_H_Fp[Num_H_Fp-1]) height = data_H_Fp[Num_H_Fp-1];
            
            if (velocity < data_V_Fp[0]) velocity = data_V_Fp[0];
            if (velocity > data_V_Fp[Num_V_Fp-1]) velocity = data_V_Fp[Num_V_Fp-1];
            
            if (rpm_current < data_RPM_Fp[0]) rpm_current = data_RPM_Fp[0];
            if (rpm_current > data_RPM_Fp[Num_RPM_Fp-1]) rpm_current = data_RPM_Fp[Num_RPM_Fp-1];
            
            // 使用3D插值计算推力和燃料消耗率
            Fp = LAQL3_Fp(Num_H_Fp, Num_V_Fp, Num_RPM_Fp, 
                          data_H_Fp, data_V_Fp, data_RPM_Fp, 
                          data_Fp_3d, height, velocity, rpm_current);
            
            dm_dt_current = LAQL3_Fp(Num_H_Fp, Num_V_Fp, Num_RPM_Fp, 
                                     data_H_Fp, data_V_Fp, data_RPM_Fp, 
                                     data_dm_dt, height, velocity, rpm_current);
                                     
            // 验证输出结果
            if (!isfinite(Fp) || !isfinite(dm_dt_current)) {
                cout << "Warning: Invalid thrust calculation results: Fp=" << Fp 
                     << ", dm_dt=" << dm_dt_current << endl;
                
                // 使用安全的默认值
                Fp = 0.0;
                dm_dt_current = 0.0;
                
                cout << "Corrected to: Fp=" << Fp << ", dm_dt=" << dm_dt_current << endl;
            }
            
            // 输出推力计算结果
            cout << "Thrust calculation result: Fp=" << Fp 
                 << ", dm_dt=" << dm_dt_current << endl;
        }
        else {
            // 使用原始的时间序列推力模型
            if (Num_Fp > 0 && data_t_Fp != nullptr && data_Fp != nullptr) {
                Fp = LAQL1(Num_Fp, data_t_Fp, data_Fp, t);
                
                // 验证输出
                if (!isfinite(Fp)) {
                    cout << "Warning: Invalid thrust from time sequence: Fp=" << Fp << endl;
                    Fp = 0.0;
                }
            }
            else {
                // 无有效推力数据
                cout << "Warning: No valid thrust data available" << endl;
                Fp = 0.0;
            }
            
            dm_dt_current = 0.0; // 在原始模型中没有质量消耗率
        }
        
        // Make sure thrust is positive
        if (Fp < 0) {
            cout << "WARNING: Negative thrust calculated, setting to zero" << endl;
            Fp = 0;
        }
        
        // Check for excessive thrust based on mass and expected acceleration
        double max_expected_thrust = mo->m * 20.0; // Limit to 20 g's worth of thrust
        if (Fp > max_expected_thrust) {
            cout << "WARNING: Excessive thrust calculated (" << Fp << "N), limiting to " << max_expected_thrust << "N" << endl;
            Fp = max_expected_thrust;
        }
        
        // Adjust thrust based on flight phase to help control trajectory
        if (mo->y[0] >= t_Fp_begin && mo->y[0] <= t_Fp_end && use_rpm_model)
        {
            // Get the navigation controller instance
            CNCModel* nc = nullptr;
            // We need to find a way to access the navigation controller
            // For now, let's use flight conditions to adjust thrust
            
            // Reduce thrust if velocity is too high (> 70 m/s)
            if (mo->V > 70.0) {
                double scale_factor = 0.9;
                Fp *= scale_factor;
                cout << "Reducing thrust by " << ((1.0 - scale_factor) * 100) << "% due to high velocity" << endl;
            }
            
            // Increase thrust if climbing with insufficient vertical speed
            if (mo->Theta > 5.0 * DtR && mo->Vy < 1.0) {
                double scale_factor = 1.1;
                Fp *= scale_factor;
                cout << "Increasing thrust by " << ((scale_factor - 1.0) * 100) << "% to assist climb" << endl;
            }
            
            // Special handling for very low altitude to prevent ground impact
            if (mo->Y < 500.0 && mo->Vy < 0) {
                // Emergency thrust increase
                Fp *= 1.2;
                cout << "WARNING: Emergency thrust increase at low altitude" << endl;
            }
        }
    }
    catch(const std::exception& e) {
        cout << "Exception in Get_Fp: " << e.what() << endl;
        Fp = 0.0;
        dm_dt_current = 0.0;
    }
}

// Method to set RPM and calculate corresponding thrust
void CEngiModel::SetRPM(double rpm)
{
    // Make sure rpm is within reasonable bounds
    if (rpm < 0) {
        cout << "WARNING: Negative RPM requested, setting to zero" << endl;
        rpm = 0;
    }
    
    double max_rpm = 10000.0; // Example maximum RPM
    if (rpm > max_rpm) {
        cout << "WARNING: Excessive RPM requested (" << rpm << "), limiting to " << max_rpm << endl;
        rpm = max_rpm;
    }
    
    // Set the RPM value
    rpm_current = rpm;
    
    // 注意：推力和燃料消耗率将在Get_Fp函数中通过LAQL3_Fp插值Engine.dat表格来计算
    // 这里不再使用简化的公式直接计算
}

// Implement the missing LAQL3_Fp function in CEngiModel

double CEngiModel::LAQL3_Fp(int l, int m, int n, double* data_l, double* data_m, double* data_n, 
                            double* data_C, double x, double y, double z)
{
    int i, j, k;
    double v, vary1, vary2, vary3, v1, v2, v3, v4, v5, v6;
    // Removed unused variables v7, v8

    // Value limiting
    if (x > data_l[l - 1]) x = data_l[l - 1];
    else if (x < data_l[0]) x = data_l[0];

    if (y > data_m[m - 1]) y = data_m[m - 1];
    else if (y < data_m[0]) y = data_m[0];

    if (z > data_n[n - 1]) z = data_n[n - 1];
    else if (z < data_n[0]) z = data_n[0];

    // Search for range
    for (k = 0; k <= n - 2; k++) {
        if ((z - data_n[k] >= 0) && (z - data_n[k + 1] <= 0)) break;
    }

    for (j = 0; j <= m - 2; j++) {
        if ((y - data_m[j] >= 0) && (y - data_m[j + 1] <= 0)) break;
    }

    for (i = 0; i <= l - 2; i++) {
        if ((x - data_l[i] >= 0) && (x - data_l[i + 1] <= 0)) break;
    }

    vary1 = (x - data_l[i]) / (data_l[i + 1] - data_l[i]);
    vary2 = (y - data_m[j]) / (data_m[j + 1] - data_m[j]);
    vary3 = (z - data_n[k]) / (data_n[k + 1] - data_n[k]);

    // Interpolation
    v1 = (1 - vary1) * data_C[(k * m + j) * l + i] + vary1 * data_C[(k * m + j) * l + i + 1];
    v2 = (1 - vary1) * data_C[(k * m + (j + 1)) * l + i] + vary1 * data_C[(k * m + (j + 1)) * l + i + 1];
    v3 = (1 - vary2) * v1 + vary2 * v2;

    v4 = (1 - vary1) * data_C[((k + 1) * m + j) * l + i] + vary1 * data_C[((k + 1) * m + j) * l + i + 1];
    v5 = (1 - vary1) * data_C[((k + 1) * m + (j + 1)) * l + i] + vary1 * data_C[((k + 1) * m + (j + 1)) * l + i + 1];
    v6 = (1 - vary2) * v4 + vary2 * v5;

    v = (1 - vary3) * v3 + vary3 * v6;

    return v;
}