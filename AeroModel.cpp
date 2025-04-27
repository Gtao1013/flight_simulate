// AeroModel.cpp - Note this file is saved with UTF-8 encoding
#define _CRT_SECURE_NO_WARNINGS
#include "AeroModel.h"
#include <string.h>  // Added string.h header for string operations
#include <stdlib.h>  // Added stdlib.h header
#include <iostream>  // Added iostream for debug output
#include <stdio.h>
// If _countof is not defined, define it
#ifndef _countof
#define _countof(array) (sizeof(array)/sizeof(array[0]))
#endif

// Declaration of file opening function defined in Data_Parameter.cpp
extern FILE* tryOpenFile(const char* fileName);

using namespace std;
/**********************************
***********************************/

////////////             Implementation of CDynaModel           //////////
/////////////////////////// ////////////////////////////////
CDynaModel::CDynaModel()
{ }

CDynaModel::~CDynaModel()
{ }

void CDynaModel::InputData()
{
    FILE* input = nullptr;
    char str[512]; // 增大缓冲区大小，以容纳长字符串
    int i;
    bool fileOpened = false;

    // 尝试多个可能的路径
    const char* possiblePaths[] = {
        "Input\\Aerodynamic.dat",
        "Aerodynamic.dat",
        "../Input/Aerodynamic.dat",
        "../Aerodynamic.dat",
        "Input/Aerodynamic.dat"
    };

    for (const char* path : possiblePaths) {
        if (fopen_s(&input, path, "r") == 0) {
            printf("成功打开文件: %s\n", path);
            fileOpened = true;
            break;
        }
    }

    if (!fileOpened) {
        printf("错误: 无法打开气动数据文件\n");
        return;
    }

    // 初始化所有指针为NULL，防止未初始化访问
    data_H = data_Ma = data_Alfa = data_beta = nullptr;
    data_Cxa = data_Cya = data_Mza = data_Cxb = data_Cyb = data_Cz = nullptr;
    data_Mx = data_My = data_Mzb = nullptr;
    data_Mx_wx = data_Mx_wy = data_Mx_wz = nullptr;
    data_My_wx = data_My_wy = data_My_wz = nullptr;
    data_Mz_wx = data_Mz_wy = data_Mz_wz = nullptr;

    try {
        // 读取第一行标题（由攻角引起的气动系数）
        if (fgets(str, sizeof(str), input) == NULL) {
            printf("错误: 无法读取文件标题\n");
            fclose(input);
            return;
        }
        printf("读取标题: %s", str);

        // 读取维度标识行
        if (fgets(str, sizeof(str), input) == NULL) {
            printf("错误: 无法读取维度标识行\n");
            fclose(input);
            return;
        }
        printf("读取维度标识: %s", str);

        // 读取维度数值
        if (fscanf_s(input, "%d%d%d", &Num_H, &Num_Ma, &Num_Alfa) != 3) {
            printf("错误: 无法读取维度数值\n");
            fclose(input);
            return;
        }
        printf("维度: H=%d Ma=%d Alpha=%d\n", Num_H, Num_Ma, Num_Alfa);

        // 跳过空行
        fgets(str, sizeof(str), input);
        if (str[0] == '\n' || str[0] == '\r') fgets(str, sizeof(str), input);

        // 读取高度数据
        if (fscanf_s(input, "%s", str, _countof(str)) != 1) { // 读取"data_H"
            printf("错误: 无法读取高度数据标识\n");
            fclose(input);
            return;
        }

        data_H = new double[Num_H];
        for (i = 0; i < Num_H; i++) {
            if (fscanf_s(input, "%lf", &data_H[i]) != 1) {
                printf("错误: 无法读取高度数据 %d/%d\n", i, Num_H);
                fclose(input);
                return;
            }
        }
        printf("读取高度数据完成\n");

        // 读取马赫数数据
        fgets(str, sizeof(str), input); // 换行
        if (fscanf_s(input, "%s", str, _countof(str)) != 1) { // 读取"data_Ma"
            printf("错误: 无法读取马赫数数据标识\n");
            fclose(input);
            return;
        }

        data_Ma = new double[Num_Ma];
        for (i = 0; i < Num_Ma; i++) {
            if (fscanf_s(input, "%lf", &data_Ma[i]) != 1) {
                printf("错误: 无法读取马赫数数据 %d/%d\n", i, Num_Ma);
                fclose(input);
                return;
            }
        }
        printf("读取马赫数数据完成\n");

        // 读取攻角数据
        fgets(str, sizeof(str), input); // 换行
        if (fscanf_s(input, "%s", str, _countof(str)) != 1) { // 读取"data_alpha"
            printf("错误: 无法读取攻角数据标识\n");
            fclose(input);
            return;
        }

        data_Alfa = new double[Num_Alfa];
        for (i = 0; i < Num_Alfa; i++) {
            if (fscanf_s(input, "%lf", &data_Alfa[i]) != 1) {
                printf("错误: 无法读取攻角数据 %d/%d\n", i, Num_Alfa);
                fclose(input);
                return;
            }
        }
        printf("读取攻角数据完成\n");

        // 跳过空行和轴向力标题行
        fgets(str, sizeof(str), input); // 换行
        while (fgets(str, sizeof(str), input) != NULL) {
            if (strlen(str) > 5) break; // 找到非空行
        }
        printf("读取标题: %s", str);

        // 为轴向力系数分配内存
        data_Cxa = new double[Num_Alfa * Num_Ma * Num_H];

        // 读取Cxa数据 - 需要特殊处理高度标记行 (H0.5, H1, H2等)
        int dataIndex = 0;
        for (int h = 0; h < Num_H; h++) {
            // 读取高度标记行 (如"H0.5")
            if (fgets(str, sizeof(str), input) == NULL) {
                printf("错误: 无法读取高度标记行 h=%d\n", h);
                fclose(input);
                return;
            }
            printf("高度标记: %s", str);

            // 读取该高度下的所有马赫数和攻角组合
            for (int m = 0; m < Num_Ma; m++) {
                for (int a = 0; a < Num_Alfa; a++) {
                    if (fscanf_s(input, "%lf", &data_Cxa[dataIndex++]) != 1) {
                        printf("错误: 读取Cxa数据失败在位置 h=%d, m=%d, a=%d, index=%d\n", h, m, a, dataIndex - 1);
                        // 使用默认值并继续
                        data_Cxa[dataIndex - 1] = 0.0;
                    }
                }
                // 每读完一行数据，需要处理换行符
                fgets(str, sizeof(str), input);
            }
        }
        printf("读取轴向力系数完成\n");

        // 读取法向力数据
        // 寻找法向力标题行
        while (fgets(str, sizeof(str), input) != NULL) {
            if (strstr(str, "法向力") != NULL || strstr(str, "Cya") != NULL) {
                break; // 找到法向力标题行
            }
            if (feof(input)) {
                printf("错误: 未找到法向力标题行\n");
                fclose(input);
                return;
            }
        }
        printf("读取标题: %s", str);

        data_Cya = new double[Num_Alfa * Num_Ma * Num_H];
        dataIndex = 0;
        for (int h = 0; h < Num_H; h++) {
            // 读取高度标记行
            if (fgets(str, sizeof(str), input) == NULL) {
                printf("错误: 无法读取Cya高度标记行 h=%d\n", h);
                // 不中断，使用默认值
                memset(&data_Cya[dataIndex], 0, sizeof(double) * Num_Alfa * Num_Ma);
                dataIndex += Num_Alfa * Num_Ma;
                continue;
            }
            printf("Cya高度标记: %s", str);

            // 读取该高度下的所有马赫数和攻角组合
            for (int m = 0; m < Num_Ma; m++) {
                for (int a = 0; a < Num_Alfa; a++) {
                    if (fscanf_s(input, "%lf", &data_Cya[dataIndex]) != 1) {
                        printf("错误: 读取Cya数据失败在位置 h=%d, m=%d, a=%d, index=%d\n", h, m, a, dataIndex);
                        // 使用默认值
                        data_Cya[dataIndex] = 0.0;
                    }
                    dataIndex++;
                }
                // 每读完一行数据，需要处理换行符
                fgets(str, sizeof(str), input);
            }
        }
        printf("读取法向力系数完成\n");

        // 读取俯仰力矩系数
        // 寻找俯仰力矩标题行
        while (fgets(str, sizeof(str), input) != NULL) {
            if (strstr(str, "俯仰力矩") != NULL || strstr(str, "Mza") != NULL) {
                break; // 找到俯仰力矩标题行
            }
            if (feof(input)) {
                printf("错误: 未找到俯仰力矩标题行\n");
                fclose(input);
                return;
            }
        }
        printf("读取标题: %s", str);

        data_Mza = new double[Num_Alfa * Num_Ma * Num_H];
        dataIndex = 0;
        for (int h = 0; h < Num_H; h++) {
            // 读取高度标记行
            if (fgets(str, sizeof(str), input) == NULL) {
                printf("错误: 无法读取Mza高度标记行 h=%d\n", h);
                // 使用默认值
                memset(&data_Mza[dataIndex], 0, sizeof(double) * Num_Alfa * Num_Ma);
                dataIndex += Num_Alfa * Num_Ma;
                continue;
            }

            // 读取该高度下的所有马赫数和攻角组合
            for (int m = 0; m < Num_Ma; m++) {
                for (int a = 0; a < Num_Alfa; a++) {
                    if (fscanf_s(input, "%lf", &data_Mza[dataIndex]) != 1) {
                        printf("错误: 读取Mza数据失败在位置 h=%d, m=%d, a=%d\n", h, m, a);
                        // 使用默认值
                        data_Mza[dataIndex] = 0.0;
                    }
                    dataIndex++;
                }
                // 每读完一行数据，需要处理换行符
                fgets(str, sizeof(str), input);
            }
        }
        printf("读取俯仰力矩系数完成\n");

        // 读取侧向气动系数标题和侧滑角个数
        // 寻找侧滑角标题行
        while (fgets(str, sizeof(str), input) != NULL) {
            if (strstr(str, "侧滑角") != NULL || strstr(str, "beta") != NULL) {
                break; // 找到侧滑角标题行
            }
            if (feof(input)) {
                printf("错误: 未找到侧滑角标题行\n");
                fclose(input);
                return;
            }
        }

        // 寻找Num_beta行
        while (fgets(str, sizeof(str), input) != NULL) {
            if (strstr(str, "Num_beta") != NULL) {
                break; // 找到Num_beta行
            }
            if (feof(input)) {
                printf("错误: 未找到Num_beta行\n");
                fclose(input);
                return;
            }
        }

        if (fscanf_s(input, "%d", &Num_beta) != 1) {
            printf("错误: 无法读取侧滑角数量\n");
            // 使用一个默认值
            Num_beta = 1;
        }
        printf("侧滑角数量: %d\n", Num_beta);

        // 读取侧滑角数据
        fgets(str, sizeof(str), input); // 换行
        if (fscanf_s(input, "%s", str, _countof(str)) != 1) { // 读取"data_beta"
            printf("错误: 无法读取侧滑角数据标识\n");
            fclose(input);
            return;
        }

        data_beta = new double[Num_beta];
        for (i = 0; i < Num_beta; i++) {
            if (fscanf_s(input, "%lf", &data_beta[i]) != 1) {
                printf("错误: 无法读取侧滑角数据 %d/%d\n", i, Num_beta);
                // 使用默认值
                data_beta[i] = 0.0;
            }
        }
        printf("读取侧滑角数据完成\n");

        // 分配剩余的内存并填充默认值
        data_Cxb = new double[Num_beta * Num_Ma * Num_H];
        data_Cyb = new double[Num_beta * Num_Ma * Num_H];
        data_Cz = new double[Num_beta * Num_Ma * Num_H];
        data_Mx = new double[Num_beta * Num_Ma * Num_H];
        data_My = new double[Num_beta * Num_Ma * Num_H];
        data_Mzb = new double[Num_beta * Num_Ma * Num_H];

        data_Mx_wx = new double[Num_Ma * Num_H];
        data_Mx_wy = new double[Num_Ma * Num_H];
        data_Mx_wz = new double[Num_Ma * Num_H];
        data_My_wx = new double[Num_Ma * Num_H];
        data_My_wy = new double[Num_Ma * Num_H];
        data_My_wz = new double[Num_Ma * Num_H];
        data_Mz_wx = new double[Num_Ma * Num_H];
        data_Mz_wy = new double[Num_Ma * Num_H];
        data_Mz_wz = new double[Num_Ma * Num_H];

        // 初始化为默认值
        memset(data_Cxb, 0, sizeof(double) * Num_beta * Num_Ma * Num_H);
        memset(data_Cyb, 0, sizeof(double) * Num_beta * Num_Ma * Num_H);
        memset(data_Cz, 0, sizeof(double) * Num_beta * Num_Ma * Num_H);
        memset(data_Mx, 0, sizeof(double) * Num_beta * Num_Ma * Num_H);
        memset(data_My, 0, sizeof(double) * Num_beta * Num_Ma * Num_H);
        memset(data_Mzb, 0, sizeof(double) * Num_beta * Num_Ma * Num_H);

        memset(data_Mx_wx, 0, sizeof(double) * Num_Ma * Num_H);
        memset(data_Mx_wy, 0, sizeof(double) * Num_Ma * Num_H);
        memset(data_Mx_wz, 0, sizeof(double) * Num_Ma * Num_H);
        memset(data_My_wx, 0, sizeof(double) * Num_Ma * Num_H);
        memset(data_My_wy, 0, sizeof(double) * Num_Ma * Num_H);
        memset(data_My_wz, 0, sizeof(double) * Num_Ma * Num_H);
        memset(data_Mz_wx, 0, sizeof(double) * Num_Ma * Num_H);
        memset(data_Mz_wy, 0, sizeof(double) * Num_Ma * Num_H);
        memset(data_Mz_wz, 0, sizeof(double) * Num_Ma * Num_H);

        // 尝试继续读取余下数据（使用try-catch捕获异常）
        try {
            // 读取由侧滑角引起的轴向力系数
            while (fgets(str, sizeof(str), input) != NULL) {
                if (strstr(str, "轴向力Cxb") != NULL) {
                    break; // 找到标题行
                }
                if (feof(input)) break;
            }

            if (!feof(input)) {
                dataIndex = 0;
                for (int h = 0; h < Num_H; h++) {
                    // 读取高度标记行
                    fgets(str, sizeof(str), input);

                    // 读取该高度下的所有马赫数和侧滑角组合
                    for (int m = 0; m < Num_Ma; m++) {
                        for (int b = 0; b < Num_beta; b++) {
                            if (fscanf_s(input, "%lf", &data_Cxb[dataIndex++]) != 1) {
                                // 使用默认值
                                data_Cxb[dataIndex - 1] = 0.0;
                            }
                        }
                        // 每读完一行数据，需要处理换行符
                        fgets(str, sizeof(str), input);
                    }
                }
                printf("读取侧滑角轴向力系数完成\n");
            }

            // 读取由侧滑角引起的法向力系数
            while (fgets(str, sizeof(str), input) != NULL) {
                if (strstr(str, "法向力Cyb") != NULL) {
                    break; // 找到标题行
                }
                if (feof(input)) break;
            }

            if (!feof(input)) {
                dataIndex = 0;
                for (int h = 0; h < Num_H; h++) {
                    // 读取高度标记行
                    fgets(str, sizeof(str), input);

                    // 读取该高度下的所有马赫数和侧滑角组合
                    for (int m = 0; m < Num_Ma; m++) {
                        for (int b = 0; b < Num_beta; b++) {
                            if (fscanf_s(input, "%lf", &data_Cyb[dataIndex++]) != 1) {
                                // 使用默认值
                                data_Cyb[dataIndex - 1] = 0.0;
                            }
                        }
                        // 每读完一行数据，需要处理换行符
                        fgets(str, sizeof(str), input);
                    }
                }
                printf("读取侧滑角法向力系数完成\n");
            }

            // 读取由侧滑角引起的侧向力系数
            while (fgets(str, sizeof(str), input) != NULL) {
                if (strstr(str, "侧向力Czb") != NULL) {
                    break; // 找到标题行
                }
                if (feof(input)) break;
            }

            if (!feof(input)) {
                dataIndex = 0;
                for (int h = 0; h < Num_H; h++) {
                    // 读取高度标记行
                    fgets(str, sizeof(str), input);

                    // 读取该高度下的所有马赫数和侧滑角组合
                    for (int m = 0; m < Num_Ma; m++) {
                        for (int b = 0; b < Num_beta; b++) {
                            if (fscanf_s(input, "%lf", &data_Cz[dataIndex++]) != 1) {
                                // 使用默认值
                                data_Cz[dataIndex - 1] = 0.0;
                            }
                        }
                        // 每读完一行数据，需要处理换行符
                        fgets(str, sizeof(str), input);
                    }
                }
                printf("读取侧滑角侧向力系数完成\n");
            }

            // 读取由侧滑角引起的滚转力矩
            while (fgets(str, sizeof(str), input) != NULL) {
                if (strstr(str, "滚转力矩Mx") != NULL) {
                    break; // 找到标题行
                }
                if (feof(input)) break;
            }

            if (!feof(input)) {
                dataIndex = 0;
                for (int h = 0; h < Num_H; h++) {
                    // 读取高度标记行
                    fgets(str, sizeof(str), input);

                    // 读取该高度下的所有马赫数和侧滑角组合
                    for (int m = 0; m < Num_Ma; m++) {
                        for (int b = 0; b < Num_beta; b++) {
                            if (fscanf_s(input, "%lf", &data_Mx[dataIndex++]) != 1) {
                                // 使用默认值
                                data_Mx[dataIndex - 1] = 0.0;
                            }
                        }
                        // 每读完一行数据，需要处理换行符
                        fgets(str, sizeof(str), input);
                    }
                }
                printf("读取侧滑角滚转力矩完成\n");
            }

            // 读取由侧滑角引起的偏航力矩
            while (fgets(str, sizeof(str), input) != NULL) {
                if (strstr(str, "偏航力矩My") != NULL) {
                    break; // 找到标题行
                }
                if (feof(input)) break;
            }

            if (!feof(input)) {
                dataIndex = 0;
                for (int h = 0; h < Num_H; h++) {
                    // 读取高度标记行
                    fgets(str, sizeof(str), input);

                    // 读取该高度下的所有马赫数和侧滑角组合
                    for (int m = 0; m < Num_Ma; m++) {
                        for (int b = 0; b < Num_beta; b++) {
                            if (fscanf_s(input, "%lf", &data_My[dataIndex++]) != 1) {
                                // 使用默认值
                                data_My[dataIndex - 1] = 0.0;
                            }
                        }
                        // 每读完一行数据，需要处理换行符
                        fgets(str, sizeof(str), input);
                    }
                }
                printf("读取侧滑角偏航力矩完成\n");
            }

            // 读取由侧滑角引起的俯仰力矩
            while (fgets(str, sizeof(str), input) != NULL) {
                if (strstr(str, "俯仰力矩Mzb") != NULL) {
                    break; // 找到标题行
                }
                if (feof(input)) break;
            }

            if (!feof(input)) {
                dataIndex = 0;
                for (int h = 0; h < Num_H; h++) {
                    // 读取高度标记行
                    fgets(str, sizeof(str), input);

                    // 读取该高度下的所有马赫数和侧滑角组合
                    for (int m = 0; m < Num_Ma; m++) {
                        for (int b = 0; b < Num_beta; b++) {
                            if (fscanf_s(input, "%lf", &data_Mzb[dataIndex++]) != 1) {
                                // 使用默认值
                                data_Mzb[dataIndex - 1] = 0.0;
                            }
                        }
                        // 每读完一行数据，需要处理换行符
                        fgets(str, sizeof(str), input);
                    }
                }
                printf("读取侧滑角俯仰力矩完成\n");
            }

            // 读取动导数标题
            while (fgets(str, sizeof(str), input) != NULL) {
                if (strstr(str, "动导数") != NULL) {
                    break; // 找到动导数标题行
                }
                if (feof(input)) break;
            }

            // 读取mx_wx动导数
            if (!feof(input)) {
                while (fgets(str, sizeof(str), input) != NULL) {
                    if (strstr(str, "mx_wx") != NULL) {
                        break; // 找到mx_wx标题行
                    }
                    if (feof(input)) break;
                }

                if (!feof(input)) {
                    dataIndex = 0;
                    for (int h = 0; h < Num_H; h++) {
                        for (int m = 0; m < Num_Ma; m++) {
                            if (fscanf_s(input, "%lf", &data_Mx_wx[dataIndex++]) != 1) {
                                // 使用默认值
                                data_Mx_wx[dataIndex - 1] = 0.0;
                            }
                            // 如果读到一行末尾
                            if ((m + 1) % Num_Ma == 0) {
                                fgets(str, sizeof(str), input); // 处理换行
                            }
                        }
                    }
                    printf("读取mx_wx动导数完成\n");
                }
            }

            // 读取mx_wy动导数
            if (!feof(input)) {
                while (fgets(str, sizeof(str), input) != NULL) {
                    if (strstr(str, "mx_wy") != NULL) {
                        break; // 找到mx_wy标题行
                    }
                    if (feof(input)) break;
                }

                if (!feof(input)) {
                    dataIndex = 0;
                    for (int h = 0; h < Num_H; h++) {
                        for (int m = 0; m < Num_Ma; m++) {
                            if (fscanf_s(input, "%lf", &data_Mx_wy[dataIndex++]) != 1) {
                                // 使用默认值
                                data_Mx_wy[dataIndex - 1] = 0.0;
                            }
                            // 如果读到一行末尾
                            if ((m + 1) % Num_Ma == 0) {
                                fgets(str, sizeof(str), input); // 处理换行
                            }
                        }
                    }
                    printf("读取mx_wy动导数完成\n");
                }
            }

            // 读取mx_wz动导数
            if (!feof(input)) {
                while (fgets(str, sizeof(str), input) != NULL) {
                    if (strstr(str, "mx_wz") != NULL) {
                        break; // 找到mx_wz标题行
                    }
                    if (feof(input)) break;
                }

                if (!feof(input)) {
                    dataIndex = 0;
                    for (int h = 0; h < Num_H; h++) {
                        for (int m = 0; m < Num_Ma; m++) {
                            if (fscanf_s(input, "%lf", &data_Mx_wz[dataIndex++]) != 1) {
                                // 使用默认值
                                data_Mx_wz[dataIndex - 1] = 0.0;
                            }
                            // 如果读到一行末尾
                            if ((m + 1) % Num_Ma == 0) {
                                fgets(str, sizeof(str), input); // 处理换行
                            }
                        }
                    }
                    printf("读取mx_wz动导数完成\n");
                }
            }

            // 读取my_wx动导数
            if (!feof(input)) {
                while (fgets(str, sizeof(str), input) != NULL) {
                    if (strstr(str, "my_wx") != NULL) {
                        break; // 找到my_wx标题行
                    }
                    if (feof(input)) break;
                }

                if (!feof(input)) {
                    dataIndex = 0;
                    for (int h = 0; h < Num_H; h++) {
                        for (int m = 0; m < Num_Ma; m++) {
                            if (fscanf_s(input, "%lf", &data_My_wx[dataIndex++]) != 1) {
                                // 使用默认值
                                data_My_wx[dataIndex - 1] = 0.0;
                            }
                            // 如果读到一行末尾
                            if ((m + 1) % Num_Ma == 0) {
                                fgets(str, sizeof(str), input); // 处理换行
                            }
                        }
                    }
                    printf("读取my_wx动导数完成\n");
                }
            }

            // 读取my_wy动导数
            if (!feof(input)) {
                while (fgets(str, sizeof(str), input) != NULL) {
                    if (strstr(str, "my_wy") != NULL) {
                        break; // 找到my_wy标题行
                    }
                    if (feof(input)) break;
                }

                if (!feof(input)) {
                    dataIndex = 0;
                    for (int h = 0; h < Num_H; h++) {
                        for (int m = 0; m < Num_Ma; m++) {
                            if (fscanf_s(input, "%lf", &data_My_wy[dataIndex++]) != 1) {
                                // 使用默认值
                                data_My_wy[dataIndex - 1] = 0.0;
                            }
                            // 如果读到一行末尾
                            if ((m + 1) % Num_Ma == 0) {
                                fgets(str, sizeof(str), input); // 处理换行
                            }
                        }
                    }
                    printf("读取my_wy动导数完成\n");
                }
            }

            // 读取my_wz动导数
            if (!feof(input)) {
                while (fgets(str, sizeof(str), input) != NULL) {
                    if (strstr(str, "my_wz") != NULL) {
                        break; // 找到my_wz标题行
                    }
                    if (feof(input)) break;
                }

                if (!feof(input)) {
                    dataIndex = 0;
                    for (int h = 0; h < Num_H; h++) {
                        for (int m = 0; m < Num_Ma; m++) {
                            if (fscanf_s(input, "%lf", &data_My_wz[dataIndex++]) != 1) {
                                // 使用默认值
                                data_My_wz[dataIndex - 1] = 0.0;
                            }
                            // 如果读到一行末尾
                            if ((m + 1) % Num_Ma == 0) {
                                fgets(str, sizeof(str), input); // 处理换行
                            }
                        }
                    }
                    printf("读取my_wz动导数完成\n");
                }
            }

            // 读取mz_wx动导数
            if (!feof(input)) {
                while (fgets(str, sizeof(str), input) != NULL) {
                    if (strstr(str, "mz_wx") != NULL) {
                        break; // 找到mz_wx标题行
                    }
                    if (feof(input)) break;
                }

                if (!feof(input)) {
                    dataIndex = 0;
                    for (int h = 0; h < Num_H; h++) {
                        for (int m = 0; m < Num_Ma; m++) {
                            if (fscanf_s(input, "%lf", &data_Mz_wx[dataIndex++]) != 1) {
                                // 使用默认值
                                data_Mz_wx[dataIndex - 1] = 0.0;
                            }
                            // 如果读到一行末尾
                            if ((m + 1) % Num_Ma == 0) {
                                fgets(str, sizeof(str), input); // 处理换行
                            }
                        }
                    }
                    printf("读取mz_wx动导数完成\n");
                }
            }

            // 读取mz_wy动导数
            if (!feof(input)) {
                while (fgets(str, sizeof(str), input) != NULL) {
                    if (strstr(str, "mz_wy") != NULL) {
                        break; // 找到mz_wy标题行
                    }
                    if (feof(input)) break;
                }

                if (!feof(input)) {
                    dataIndex = 0;
                    for (int h = 0; h < Num_H; h++) {
                        for (int m = 0; m < Num_Ma; m++) {
                            if (fscanf_s(input, "%lf", &data_Mz_wy[dataIndex++]) != 1) {
                                // 使用默认值
                                data_Mz_wy[dataIndex - 1] = 0.0;
                            }
                            // 如果读到一行末尾
                            if ((m + 1) % Num_Ma == 0) {
                                fgets(str, sizeof(str), input); // 处理换行
                            }
                        }
                    }
                    printf("读取mz_wy动导数完成\n");
                }
            }

            // 读取mz_wz动导数
            if (!feof(input)) {
                while (fgets(str, sizeof(str), input) != NULL) {
                    if (strstr(str, "mz_wz") != NULL) {
                        break; // 找到mz_wz标题行
                    }
                    if (feof(input)) break;
                }

                if (!feof(input)) {
                    dataIndex = 0;
                    for (int h = 0; h < Num_H; h++) {
                        for (int m = 0; m < Num_Ma; m++) {
                            if (fscanf_s(input, "%lf", &data_Mz_wz[dataIndex++]) != 1) {
                                // 使用默认值
                                data_Mz_wz[dataIndex - 1] = 0.0;
                            }
                            // 如果读到一行末尾
                            if ((m + 1) % Num_Ma == 0) {
                                fgets(str, sizeof(str), input); // 处理换行
                            }
                        }
                    }
                    printf("读取mz_wz动导数完成\n");
                }
            }
        }
        catch (const std::exception& e) {
            printf("警告: 读取剩余数据时发生异常: %s\n", e.what());
            // 继续使用默认值
        }
    }
    catch (const std::exception& e) {
        printf("错误: 读取数据过程中发生异常: %s\n", e.what());
    }

    fclose(input);
    printf("气动数据读取完成，已使用默认值填充未读取的数据\n");

    // 在InputData函数末尾添加数据验证
    bool dataValid = false;
    if (data_Cxa && data_Cya && data_Mza) {
        // 检查数据中是否有非零值
        for (int i = 0; i < Num_Alfa * Num_Ma * Num_H; i++) {
            if (data_Cxa[i] != 0.0 || data_Cya[i] != 0.0 || data_Mza[i] != 0.0) {
                dataValid = true;
                break;
            }
        }
    }

    printf("气动数据验证: %s (检查是否包含非零值)\n", dataValid ? "成功" : "失败");
    // 打印第一组数据用于验证
    if (data_Cxa && data_Cya && data_Mza) {
        printf("数据样本: Cxa[0]=%f, Cya[0]=%f, Mza[0]=%f\n",
            data_Cxa[0], data_Cya[0], data_Mza[0]);
    }

    // 添加更多数据验证和样本输出
    if (data_Cxa && data_Cya && data_Mza) {
        printf("\n===== 数据样本详细检查 =====\n");

        // 打印数据存储顺序信息
        printf("数据存储维度: [%d高度 x %d马赫数 x %d攻角] = 总计%d个元素\n",
            Num_H, Num_Ma, Num_Alfa, Num_H * Num_Ma * Num_Alfa);

        // 打印攻角数值范围
        printf("攻角范围: %.2f到%.2f度\n", data_Alfa[0], data_Alfa[Num_Alfa - 1]);

        // 打印每个高度第一个马赫数下的首尾攻角对应的气动系数
        printf("不同高度下的数据样本:\n");
        for (int h = 0; h < Num_H; h++) {
            printf("  高度H=%.1f米:\n", data_H[h]);

            // 计算第一个马赫数下，首尾两个攻角的索引位置
            int idx_first = h * Num_Ma * Num_Alfa + 0 * Num_Alfa + 0;
            int idx_last = h * Num_Ma * Num_Alfa + 0 * Num_Alfa + (Num_Alfa - 1);

            // 打印这两个位置的数据
            printf("    攻角=%.2f度: Cxa=%.6f, Cya=%.6f, Mza=%.6f\n",
                data_Alfa[0], data_Cxa[idx_first], data_Cya[idx_first], data_Mza[idx_first]);
            printf("    攻角=%.2f度: Cxa=%.6f, Cya=%.6f, Mza=%.6f\n",
                data_Alfa[Num_Alfa - 1], data_Cxa[idx_last], data_Cya[idx_last], data_Mza[idx_last]);
        }
    }
}

void CDynaModel::GetForce_Moment_Motion(CMassModel* ma, CMotiModel* mo, Data_Parameter* Gdata)
{
    t = mo->y[0];

    // 调试输出-显示当前插值条件
    printf("气动计算: 高度=%.1f米, 马赫数=%.4f, 攻角=%.4f度, 侧滑角=%.4f度\n",
        mo->H, mo->Ma, mo->alfa * RtD, mo->beta * RtD);

    // 添加坐标系标志调试输出
    printf("坐标系标志 flag_1=%d (1=体轴系, 2=风轴系)\n", Gdata->flag_1);

    // 计算气动系数 - 使用当前状态进行查表插值
    try {
        // 攻角引起的气动系数
        double alpha_deg = mo->alfa * RtD;  // 转换为角度
        double beta_deg = mo->beta * RtD;   // 转换为角度

        // 限制参数在插值表范围内
        double h_constrained = mo->H;
        double ma_constrained = mo->Ma;
        double alpha_constrained = alpha_deg;
        double beta_constrained = beta_deg;

        // 参数范围检查
        if (h_constrained < data_H[0]) h_constrained = data_H[0];
        if (h_constrained > data_H[Num_H - 1]) h_constrained = data_H[Num_H - 1];

        if (ma_constrained < data_Ma[0]) ma_constrained = data_Ma[0];
        if (ma_constrained > data_Ma[Num_Ma - 1]) ma_constrained = data_Ma[Num_Ma - 1];

        if (alpha_constrained < data_Alfa[0]) alpha_constrained = data_Alfa[0];
        if (alpha_constrained > data_Alfa[Num_Alfa - 1]) alpha_constrained = data_Alfa[Num_Alfa - 1];

        if (beta_constrained < data_beta[0]) beta_constrained = data_beta[0];
        if (beta_constrained > data_beta[Num_beta - 1]) beta_constrained = data_beta[Num_beta - 1];

        // 进行插值计算 - 详细调试输出
        printf("插值参数: 高度=%.1f米, 马赫数=%.4f, 攻角=%.4f度, 侧滑角=%.4f度\n",
            h_constrained, ma_constrained, alpha_constrained, beta_constrained);

        // 注意！关键修改：确保插值时使用正确的索引顺序
        Cx_a = LAQL3(Num_H, Num_Ma, Num_Alfa, data_H, data_Ma, data_Alfa, data_Cxa,
            h_constrained, ma_constrained, alpha_constrained);

        printf("插值结果: Cx_a = %.6f\n", Cx_a);

        // 下面计算其他气动系数...
        Cx_b = LAQL3(Num_H, Num_Ma, Num_beta, data_H, data_Ma, data_beta, data_Cxb,
            h_constrained, ma_constrained, beta_constrained);
        Cx_0 = LAQL3(Num_H, Num_Ma, Num_Alfa, data_H, data_Ma, data_Alfa, data_Cxa,
            h_constrained, ma_constrained, 0.0);
        Cx = Cx_a + Cx_b - Cx_0;

        Cy_a = LAQL3(Num_H, Num_Ma, Num_Alfa, data_H, data_Ma, data_Alfa, data_Cya,
            h_constrained, ma_constrained, alpha_constrained);
        Cy_b = LAQL3(Num_H, Num_Ma, Num_beta, data_H, data_Ma, data_beta, data_Cyb,
            h_constrained, ma_constrained, beta_constrained);
        Cy_0 = LAQL3(Num_H, Num_Ma, Num_Alfa, data_H, data_Ma, data_Alfa, data_Cya,
            h_constrained, ma_constrained, 0.0);
        Cy = Cy_a; // 确认这里是否应该是Cy_a

        Cz_b = LAQL3(Num_H, Num_Ma, Num_beta, data_H, data_Ma, data_beta, data_Cz,
            h_constrained, ma_constrained, beta_constrained);
        Cz = Cz_b;

        printf("体轴系气动系数: Cx=%.6f, Cy=%.6f, Cz=%.6f\n", Cx, Cy, Cz);

        // 关键修改: 确保正确的坐标系转换
        // 检查坐标系标志并确保总是执行转换
        if (Gdata->flag_1 == 1)
        {
            // 体轴系到风轴系的转换
            // 注意：阻力系数(CD)定义为向后为正，所以前面有负号
            CD = -(cos(mo->alfa) * cos(mo->beta) * Cx - sin(mo->alfa) * cos(mo->beta) * Cy + sin(mo->beta) * Cz);
            CL = sin(mo->alfa) * Cx + cos(mo->alfa) * Cy;
            CZ = -cos(mo->alfa) * sin(mo->beta) * Cx + sin(mo->alfa) * sin(mo->beta) * Cy + cos(mo->beta) * Cz;

            printf("使用体轴系到风轴系转换 (flag_1=1)\n");
        }
        else if (Gdata->flag_1 == 2)
        {
            // 如果已经是风轴系，直接赋值
            CD = Cx;  // 注意：确认是否需要反转符号
            CL = Cy;
            CZ = Cz;

            printf("使用风轴系直接赋值 (flag_1=2)\n");
        }
        else
        {
            // 未知标志，使用默认转换
            printf("警告: 未知的坐标系标志 flag_1=%d，使用默认体轴系转换\n", Gdata->flag_1);
            CD = -(cos(mo->alfa) * cos(mo->beta) * Cx - sin(mo->alfa) * cos(mo->beta) * Cy + sin(mo->beta) * Cz);
            CL = sin(mo->alfa) * Cx + cos(mo->alfa) * Cy;
            CZ = -cos(mo->alfa) * sin(mo->beta) * Cx + sin(mo->alfa) * sin(mo->beta) * Cy + cos(mo->beta) * Cz;
        }

        // 打印最终风轴系气动系数
        printf("风轴系气动系数: CD=%.6f, CL=%.6f, CZ=%.6f\n", CD, CL, CZ);

        // 计算力矩系数
        Mx_b = LAQL3(Num_H, Num_Ma, Num_beta, data_H, data_Ma, data_beta, data_Mx,
            h_constrained, ma_constrained, beta_constrained);
        Mx = Mx_b;

        My_b = LAQL3(Num_H, Num_Ma, Num_beta, data_H, data_Ma, data_beta, data_My,
            h_constrained, ma_constrained, beta_constrained);
        My = My_b;

        Mz_a = LAQL3(Num_H, Num_Ma, Num_Alfa, data_H, data_Ma, data_Alfa, data_Mza,
            h_constrained, ma_constrained, alpha_constrained);
        Mz_b = LAQL3(Num_H, Num_Ma, Num_beta, data_H, data_Ma, data_beta, data_Mzb,
            h_constrained, ma_constrained, beta_constrained);
        Mz_0 = LAQL3(Num_H, Num_Ma, Num_Alfa, data_H, data_Ma, data_Alfa, data_Mza,
            h_constrained, ma_constrained, 0.0);
        Mz = Mz_a; // 确认是否应该只用Mz_a

        // 计算动导数
        Mx_wx = LAQL2(Num_H, Num_Ma, data_H, data_Ma, data_Mx_wx, h_constrained, ma_constrained);
        Mx_wy = LAQL2(Num_H, Num_Ma, data_H, data_Ma, data_Mx_wy, h_constrained, ma_constrained);
        Mx_wz = LAQL2(Num_H, Num_Ma, data_H, data_Ma, data_Mx_wz, h_constrained, ma_constrained);

        My_wx = LAQL2(Num_H, Num_Ma, data_H, data_Ma, data_My_wx, h_constrained, ma_constrained);
        My_wy = LAQL2(Num_H, Num_Ma, data_H, data_Ma, data_My_wy, h_constrained, ma_constrained);
        My_wz = LAQL2(Num_H, Num_Ma, data_H, data_Ma, data_My_wz, h_constrained, ma_constrained);

        Mz_wx = LAQL2(Num_H, Num_Ma, data_H, data_Ma, data_Mz_wx, h_constrained, ma_constrained);
        Mz_wy = LAQL2(Num_H, Num_Ma, data_H, data_Ma, data_Mz_wy, h_constrained, ma_constrained);
        Mz_wz = LAQL2(Num_H, Num_Ma, data_H, data_Ma, data_Mz_wz, h_constrained, ma_constrained);
    }
    catch (const std::exception& e) {
        printf("气动系数计算异常: %s\n", e.what());
        // 使用默认值
        CD = 0;
        CL = 0;
        CZ = 0;
        Mx = My = Mz = 0;
    }
}


void CDynaModel::FreeData()                      // 释放动态内存
{
    // 安全检查 - 仅释放非空指针
    if (data_H) delete[] data_H;
    if (data_Ma) delete[] data_Ma;
    if (data_Alfa) delete[] data_Alfa;
    if (data_beta) delete[] data_beta;

    if (data_Cxa) delete[] data_Cxa;
    if (data_Cya) delete[] data_Cya;
    if (data_Mza) delete[] data_Mza;
    if (data_Cxb) delete[] data_Cxb;
    if (data_Cyb) delete[] data_Cyb;
    if (data_Cz) delete[] data_Cz;
    if (data_Mx) delete[] data_Mx;
    if (data_My) delete[] data_My;
    if (data_Mzb) delete[] data_Mzb;

    if (data_Mx_wx) delete[] data_Mx_wx;
    if (data_Mx_wy) delete[] data_Mx_wy;
    if (data_Mx_wz) delete[] data_Mx_wz;
    if (data_My_wx) delete[] data_My_wx;
    if (data_My_wy) delete[] data_My_wy;
    if (data_My_wz) delete[] data_My_wz;
    if (data_Mz_wx) delete[] data_Mz_wx;
    if (data_Mz_wy) delete[] data_Mz_wy;
    if (data_Mz_wz) delete[] data_Mz_wz;
    
    // 重置指针为空，防止重复释放
    data_H = data_Ma = data_Alfa = data_beta = nullptr;
    data_Cxa = data_Cya = data_Mza = data_Cxb = data_Cyb = data_Cz = nullptr;
    data_Mx = data_My = data_Mzb = nullptr;
    data_Mx_wx = data_Mx_wy = data_Mx_wz = nullptr;
    data_My_wx = data_My_wy = data_My_wz = nullptr;
    data_Mz_wx = data_Mz_wy = data_Mz_wz = nullptr;
}
