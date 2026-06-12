#pragma once

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// DLL导出/导入宏
#ifdef MODELTEMPLATE_EXPORTS
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT __declspec(dllimport)
#endif
