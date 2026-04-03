#include "HPOP.h"
#include "HPOPEquation.hpp"
#include "AstMath/ODE.hpp"
#include "AstMath/Vector.hpp"
#include "AstUtil/Logger.hpp"


HPOP::~HPOP()
{
    if (equation_)
        delete equation_;
    if (integrator_)
        delete integrator_;
}

int HPOP::setForceModel(const HPOPForceModel& forcemodel)
{
    if(!equation_){
        equation_ = new HPOPEquation();
    }
    return equation_->setForceModel(forcemodel);
}

int HPOP::setPropagationFrame(Frame *frame)
{
    return equation_->setPropagationFrame(frame);
}

ODEIntegrator *HPOP::getIntegrator() const
{
    if(A_UNLIKELY(!integrator_)){
        // const_cast<HPOP*>
        (this)->integrator_ = new RKF78();
    }
    return integrator_;
}

int HPOP::initialize()
{
    if (!equation_){
        equation_ = new HPOPEquation();
    }
    if (!integrator_){
        integrator_ = new RKF78();
    }
    equation_->initialize();
    // err |= integrator_->initialize(equation_);
    return 0;
}




int HPOP::propagate(const TimePoint &startTime, TimePoint &targetTime, Vector3d &position, Vector3d &velocity)
{
    int err = this->initialize();
    if (err)
        return err;
    int dim = equation_->getDimension();
    if (dim != 6){
        aError("dimension of equation is not 6");
        return -1;
    }
    equation_->setEpoch(startTime);
    array6d y = {position.x(), position.y(), position.z(), velocity.x(), velocity.y(), velocity.z()};
    double duration = targetTime - startTime;
    double t = 0;
    err = integrator_->integrate(*equation_,  y.data(), t, duration);
    if(t != duration && !err){
        targetTime = startTime + t;
    }
    position = {y[0], y[1], y[2]};
    velocity = {y[3], y[4], y[5]};
    return err;
}

AST_NAMESPACE_END

