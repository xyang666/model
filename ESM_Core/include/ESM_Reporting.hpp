// ****************************************************************************
// ESM_Reporting.hpp - 报告引擎
//
// EmitterTypeReporting: 基于置信度的发射器类型识别
// TargetTypeReporting:  基于发射器组合的目标平台分类
//
// 对应 AFSIM WsfEmitterTypeReporting / WsfTargetTypeReporting。
// ****************************************************************************
#pragma once

#include "ESM_Reporting.h"

#include <random>
#include <map>
#include <string>

class EmitterTypeReporting
{
public:
    // 添加一条发射器报告规则
    void AddRule(const EmitterReportRule& rule);

    // 检测到发射器时调用（每次成功探测后）
    void OnDetection(const std::string& emitter_id, const std::string& truth_type,
                     double sim_time);

    // 每帧评估：对达到评估时间的发射器执行声明逻辑
    void Evaluate(double sim_time);

    // 查询指定发射器的声明结果。返回 true 表示已声明。
    bool GetDeclaration(const std::string& emitter_id,
                        std::string& declared_type, double& confidence) const;

    // 检查指定发射器本次 Evaluate 是否刚刚完成首次声明（用于事件生成）
    bool JustDeclared(const std::string& emitter_id) const;

    // 清空所有状态
    void Reset();

private:
    std::string evaluate_rule(const EmitterReportRule& rule);

    std::map<std::string, EmitterReportState> states_;
    std::map<std::string, EmitterReportRule>  rules_;
    std::vector<std::string> just_declared_;  // 本轮刚刚声明的 emitter_id 列表

    std::mt19937 rng_{std::random_device{}()};
};

class TargetTypeReporting
{
public:
    // 添加一条目标平台报告规则
    void AddRule(const TargetReportRule& rule);

    // 检测到发射器时调用
    void OnDetection(const std::string& target_id, const std::string& emitter_id,
                     double sim_time);

    // 每帧评估
    void Evaluate(double sim_time);

    // 查询声明结果
    bool GetDeclaration(const std::string& target_id,
                        std::string& declared_type, double& confidence) const;

    // 检查本次 Evaluate 是否刚刚完成首次声明
    bool JustDeclared(const std::string& target_id) const;

    void Reset();

private:
    std::string evaluate_rule(const TargetReportRule& rule,
                              const std::vector<std::string>& detected_emitters);

    std::map<std::string, TargetReportState> states_;
    std::map<std::string, TargetReportRule>  rules_;
    std::vector<std::string> just_declared_;

    std::mt19937 rng_{std::random_device{}()};
};
