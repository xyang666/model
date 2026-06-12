#ifndef FREQ_TABLE_HPP
#define FREQ_TABLE_HPP

#include <algorithm>
#include <vector>

// 频率索引查找表，支持分段线性插值。
//
// 典型用途：
//   - detection_threshold_table : 频率 (Hz) → 最小 SNR (dB)
//   - sensitivity_table         : 频率 (Hz) → 最小可检测功率 (dBm)
//
// 用法：
//   FreqTable t;
//   t.add(1e9, 10.0);
//   t.add(3e9,  8.0);
//   t.add(10e9, 12.0);
//   t.build();                // 按频率升序排序；所有 add() 之后调用一次
//   double v = t.lookup(2e9); // 返回 9.0（插值结果）

struct FreqTableEntry
{
    double frequency_hz;
    double value;
};

class FreqTable
{
public:
    void add(double frequency_hz, double value)
    {
        entries_.push_back({frequency_hz, value});
    }

    // 按频率升序排序。
    // 在所有 add() 调用之后调用一次。若条目已按顺序添加则非必须。
    void build()
    {
        std::sort(entries_.begin(), entries_.end(),
                  [](const FreqTableEntry &a, const FreqTableEntry &b)
                  {
                      return a.frequency_hz < b.frequency_hz;
                  });
    }

    bool empty() const { return entries_.empty(); }

    // 返回 frequency_hz 处的线性插值。
    // 超出表范围时钳位到边界值。
    double lookup(double frequency_hz) const
    {
        if (entries_.empty())
            return 0.0;

        if (entries_.size() == 1 || frequency_hz <= entries_.front().frequency_hz)
            return entries_.front().value;

        if (frequency_hz >= entries_.back().frequency_hz)
            return entries_.back().value;

        // 找到包围 frequency_hz 的区间 [lo, hi]。
        auto it = std::lower_bound(
            entries_.begin(), entries_.end(), frequency_hz,
            [](const FreqTableEntry &e, double f)
            { return e.frequency_hz < f; });

        if (it->frequency_hz == frequency_hz)
            return it->value;

        const FreqTableEntry &hi = *it;
        const FreqTableEntry &lo = *std::prev(it);
        double t = (frequency_hz - lo.frequency_hz) / (hi.frequency_hz - lo.frequency_hz);
        return lo.value + t * (hi.value - lo.value);
    }

private:
    std::vector<FreqTableEntry> entries_;
};

#endif // FREQ_TABLE_HPP
