#define _CRT_SECURE_NO_WARNINGS
#include "FileUtils.h"
#include <string.h>  // For string operations
#include <iostream>  // For cout
#include <direct.h>  // For directory operations

using namespace std;

// Implement file opening utility function
FILE* tryOpenFile(const char* fileName) {
    FILE* file = NULL;
    char pathBuffer[512] = {0}; // 初始化为0确保字符串总是有终止符
    
    cout << "Attempting to open file: " << fileName << endl;
    
    // Method 1: Direct filename
    if ((file = fopen(fileName, "r")) != NULL) {
        cout << "Successfully opened file: " << fileName << endl;
        return file;
    }
    cout << "  Path 1 failed: " << fileName << endl;
    
    // Method 2: Using Input/ format
    memset(pathBuffer, 0, sizeof(pathBuffer)); // 每次使用前清空缓冲区
    snprintf(pathBuffer, sizeof(pathBuffer) - 1, "Input/%s", fileName);
    if ((file = fopen(pathBuffer, "r")) != NULL) {
        cout << "Successfully opened file: " << pathBuffer << endl;
        return file;
    }
    cout << "  Path 2 failed: " << pathBuffer << endl;
    
    // Method 3: Using ./Input/ format
    memset(pathBuffer, 0, sizeof(pathBuffer));
    snprintf(pathBuffer, sizeof(pathBuffer) - 1, "./Input/%s", fileName);
    if ((file = fopen(pathBuffer, "r")) != NULL) {
        cout << "Successfully opened file: " << pathBuffer << endl;
        return file;
    }
    cout << "  Path 3 failed: " << pathBuffer << endl;
    
    // Method 4: Using ../Input/ format
    memset(pathBuffer, 0, sizeof(pathBuffer));
    snprintf(pathBuffer, sizeof(pathBuffer) - 1, "../Input/%s", fileName);
    if ((file = fopen(pathBuffer, "r")) != NULL) {
        cout << "Successfully opened file: " << pathBuffer << endl;
        return file;
    }
    cout << "  Path 4 failed: " << pathBuffer << endl;
    
    // Method 5: Using ../../Input/ format (for x64/Debug directory case)
    memset(pathBuffer, 0, sizeof(pathBuffer));
    snprintf(pathBuffer, sizeof(pathBuffer) - 1, "../../Input/%s", fileName);
    if ((file = fopen(pathBuffer, "r")) != NULL) {
        cout << "Successfully opened file: " << pathBuffer << endl;
        return file;
    }
    cout << "  Path 5 failed: " << pathBuffer << endl;
    
    // Method 6: Using current working directory
    char currentDir[512] = {0};
    if (_getcwd(currentDir, sizeof(currentDir) - 1)) {
        memset(pathBuffer, 0, sizeof(pathBuffer));
        snprintf(pathBuffer, sizeof(pathBuffer) - 1, "%s/Input/%s", currentDir, fileName);
        if ((file = fopen(pathBuffer, "r")) != NULL) {
            cout << "Successfully opened file: " << pathBuffer << endl;
            fflush(stdout); // 确保输出立即显示
            return file;
        }
        cout << "  Path 6 failed: " << pathBuffer << endl;
    }
    
    cout << "Cannot open file: " << fileName << ", please ensure the file exists and has read permissions" << endl;
    fflush(stdout); // 确保错误消息立即显示
    return NULL;
}