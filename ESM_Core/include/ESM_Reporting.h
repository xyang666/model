// ****************************************************************************
// ESM_Reporting.h - 报告规则数据结构
//
// 定义发射器识别和目标平台分类的报告规则和运行状态。
// 对应 AFSIM WsfEmitterTypeReporting / WsfTargetTypeReporting。
// ****************************************************************************
#pragma once

#include <map>
#include <string>
#include <vector>

// ============================================================================
// 发射器报告规则
// ============================================================================
struct EmitterReportRule
{
    std::string emitter_truth_type;       // 匹配的真实发射器类型
    double      time_to_declare = 5.0;    // 首次声明等待时间 (s)
    double      time_to_reevaluate = 10.0; // 重新评估间隔 (s)
    bool        report_truth = false;      // true = 始终以100%置信度报告真实类型
    // cTABLE 模式：概率抽取表 {类型名, 置信度}，最后一个 confidence==0 表示 remainder
    std::vector<std::pair<std::string, double>> report_table;
};

// ============================================================================
// 目标平台报告规则
// ============================================================================
struct TargetReportRule
{
    std::string target_truth_type;
    double      time_to_declare = 5.0;
    double      time_to_reevaluate = 10.0;
    bool        report_truth = false;
    std::vector<std::pair<std::string, double>> report_table;
    // cEMITTERS 模式：从检测到的发射器 ID 组合推断平台类型
    // key = 发射器 ID 列表（任意顺序即可匹配），value = 声明的目标类型
    std::vector<std::pair<std::vector<std::string>, std::string>> emitter_to_target;
};

// ============================================================================
// 发射器报告状态（每个已探测发射器一个实例）
// ============================================================================
struct EmitterReportState
{
    std::string emitter_id;
    std::string truth_id;            // 真实发射器类型
    std::string declared_id;         // 当前声明的类型
    double      declared_confidence = 0.0; // 声明置信度 [0, 1]
    double      first_detect_time = 0.0;  // 首次探测时刻
    double      next_evaluate_time = 0.0; // 下次重新评估时刻
    bool        declared = false;         // 是否已首次声明
    EmitterReportRule rule;
};

// ============================================================================
// 目标报告状态
// ============================================================================
struct TargetReportState
{
    std::string target_id;
    std::string declared_type;
    double      declared_confidence = 0.0;
    double      next_evaluate_time = 0.0;
    std::vector<std::string> detected_emitters; // 已探测到的发射器 ID 列表
    TargetReportRule rule;
};
