// #include "AstGlobal.h"
// #include "AstUtil/Constants.h"
// #include "AstCore/CelestialBody.hpp"
#include <string>
#include <vector>

class HPOPEquation; 
class ODEIntegrator;
class HPOPForceModel;

/// @brief 高精度轨道预报接口类
class HPOP
{
public:
    HPOP() = default;
    ~HPOP();
public:
    /// @brief 设置力模型
    errc_t setForceModel(const HPOPForceModel& forcemodel);

    /// @brief 设置预报坐标系
    errc_t setPropagationFrame(Frame* frame);

    /// @brief 设置积分器
    void setIntegrator(ODEIntegrator* integrator);

    /// @brief 获取积分器
    ODEIntegrator* getIntegrator() const;

    /// @brief 轨道预报
    /// 考虑到有停止条件，所以预报结束时间同时也是一个输出参数
    /// @param[in]      startTime   预报起始时间
    /// @param[in,out]  targetTime  预报结束时间
    /// @param[in,out]  position    输出位置向量
    /// @param[in,out]  velocity    输出速度向量
    /// @return errc_t  错误码
    errc_t propagate(const TimePoint& startTime, TimePoint& targetTime, Vector3d& position, Vector3d& velocity);

    /// @brief 初始化
    errc_t initialize();
protected:
    HPOPEquation* equation_{nullptr};               ///< 高精度轨道预报方程
    mutable ODEIntegrator* integrator_{nullptr};    ///< 高精度轨道预报积分器
};


/*! @} */

AST_NAMESPACE_END
