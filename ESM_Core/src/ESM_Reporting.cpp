// ****************************************************************************
// ESM_Reporting.cpp - 报告引擎实现
// ****************************************************************************

#include "ESM_Reporting.hpp"
#include <algorithm>

// ============================================================================
// EmitterTypeReporting
// ============================================================================

void EmitterTypeReporting::AddRule(const EmitterReportRule& rule)
{
    rules_[rule.emitter_truth_type] = rule;
}

void EmitterTypeReporting::OnDetection(const std::string& emitter_id,
                                       const std::string& truth_type,
                                       double sim_time)
{
    auto it = states_.find(emitter_id);
    if (it == states_.end())
    {
        // 首次检测到该发射器，创建报告状态
        EmitterReportState state;
        state.emitter_id       = emitter_id;
        state.truth_id         = truth_type;
        state.first_detect_time = sim_time;

        // 按 truth_type 匹配规则
        auto rule_it = rules_.find(truth_type);
        if (rule_it != rules_.end())
        {
            state.rule = rule_it->second;
        }
        // 未匹配到规则则使用默认：始终报告真实类型
        else
        {
            state.rule.emitter_truth_type = truth_type;
            state.rule.report_truth = true;
        }

        states_[emitter_id] = state;
    }
}

void EmitterTypeReporting::Evaluate(double sim_time)
{
    just_declared_.clear();

    for (auto& kv : states_)
    {
        EmitterReportState& state = kv.second;

        // 已声明则按重新评估间隔检查
        if (state.declared && sim_time >= state.next_evaluate_time)
        {
            state.declared_id = evaluate_rule(state.rule);
            state.next_evaluate_time = sim_time + state.rule.time_to_reevaluate;
        }
        // 未声明且达到首次声明时间
        else if (!state.declared &&
                 sim_time >= state.first_detect_time + state.rule.time_to_declare)
        {
            state.declared_id = evaluate_rule(state.rule);
            state.declared_confidence = state.rule.report_truth ? 1.0 : 0.5;
            state.declared = true;
            state.next_evaluate_time = sim_time + state.rule.time_to_reevaluate;
            just_declared_.push_back(state.emitter_id);
        }
    }
}

std::string EmitterTypeReporting::evaluate_rule(const EmitterReportRule& rule)
{
    if (rule.report_truth)
        return rule.emitter_truth_type;

    if (rule.report_table.empty())
        return rule.emitter_truth_type;

    // 概率抽取
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    double roll = dist(rng_);

    double cumulative = 0.0;
    for (const auto& entry : rule.report_table)
    {
        double conf = entry.second;
        // confidence == 0 表示 remainder（剩余概率）
        if (conf == 0.0)
            return entry.first;

        cumulative += conf;
        if (roll <= cumulative)
            return entry.first;
    }

    // 兜底：返回真实类型
    return rule.emitter_truth_type;
}

bool EmitterTypeReporting::GetDeclaration(const std::string& emitter_id,
                                          std::string& declared_type,
                                          double& confidence) const
{
    auto it = states_.find(emitter_id);
    if (it == states_.end() || !it->second.declared)
        return false;
    declared_type = it->second.declared_id;
    confidence    = it->second.declared_confidence;
    return true;
}

bool EmitterTypeReporting::JustDeclared(const std::string& emitter_id) const
{
    return std::find(just_declared_.begin(), just_declared_.end(), emitter_id)
           != just_declared_.end();
}

void EmitterTypeReporting::Reset()
{
    states_.clear();
    just_declared_.clear();
}

// ============================================================================
// TargetTypeReporting
// ============================================================================

void TargetTypeReporting::AddRule(const TargetReportRule& rule)
{
    rules_[rule.target_truth_type] = rule;
}

void TargetTypeReporting::OnDetection(const std::string& target_id,
                                      const std::string& emitter_id,
                                      double sim_time)
{
    auto it = states_.find(target_id);
    if (it == states_.end())
    {
        TargetReportState state;
        state.target_id = target_id;
        state.detected_emitters.push_back(emitter_id);

        // 尝试匹配规则（按真实类型匹配，这里使用 target_id 作为 truth）
        auto rule_it = rules_.find(target_id);
        if (rule_it != rules_.end())
        {
            state.rule = rule_it->second;
        }
        else
        {
            state.rule.target_truth_type = target_id;
            state.rule.report_truth = true;
        }

        states_[target_id] = state;
    }
    else
    {
        // 添加新发现的发射器（去重）
        auto& emitters = it->second.detected_emitters;
        if (std::find(emitters.begin(), emitters.end(), emitter_id) == emitters.end())
            emitters.push_back(emitter_id);
    }
}

void TargetTypeReporting::Evaluate(double sim_time)
{
    just_declared_.clear();

    for (auto& kv : states_)
    {
        TargetReportState& state = kv.second;

        if (state.declared_type.empty() &&
            sim_time >= state.next_evaluate_time)
        {
            state.declared_type = evaluate_rule(state.rule, state.detected_emitters);
            state.declared_confidence = state.rule.report_truth ? 1.0 : 0.5;
            state.next_evaluate_time = sim_time + state.rule.time_to_reevaluate;
            just_declared_.push_back(state.target_id);
        }
        else if (!state.declared_type.empty() &&
                 sim_time >= state.next_evaluate_time)
        {
            state.declared_type = evaluate_rule(state.rule, state.detected_emitters);
            state.next_evaluate_time = sim_time + state.rule.time_to_reevaluate;
        }
    }
}

std::string TargetTypeReporting::evaluate_rule(
    const TargetReportRule& rule,
    const std::vector<std::string>& detected_emitters)
{
    if (rule.report_truth)
        return rule.target_truth_type;

    // cEMITTERS 模式：按检测到的发射器组合匹配
    if (!rule.emitter_to_target.empty())
    {
        for (const auto& entry : rule.emitter_to_target)
        {
            const std::vector<std::string>& required = entry.first;
            bool all_found = true;
            for (const auto& req : required)
            {
                if (std::find(detected_emitters.begin(),
                              detected_emitters.end(), req) == detected_emitters.end())
                {
                    all_found = false;
                    break;
                }
            }
            if (all_found)
                return entry.second;
        }
    }

    // cTABLE 模式：概率抽取
    if (!rule.report_table.empty())
    {
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        double roll = dist(rng_);
        double cumulative = 0.0;
        for (const auto& entry : rule.report_table)
        {
            if (entry.second == 0.0)
                return entry.first;
            cumulative += entry.second;
            if (roll <= cumulative)
                return entry.first;
        }
    }

    return rule.target_truth_type;
}

bool TargetTypeReporting::GetDeclaration(const std::string& target_id,
                                         std::string& declared_type,
                                         double& confidence) const
{
    auto it = states_.find(target_id);
    if (it == states_.end() || it->second.declared_type.empty())
        return false;
    declared_type = it->second.declared_type;
    confidence    = it->second.declared_confidence;
    return true;
}

bool TargetTypeReporting::JustDeclared(const std::string& target_id) const
{
    return std::find(just_declared_.begin(), just_declared_.end(), target_id)
           != just_declared_.end();
}

void TargetTypeReporting::Reset()
{
    states_.clear();
    just_declared_.clear();
}
