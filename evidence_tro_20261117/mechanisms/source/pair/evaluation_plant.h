#ifndef EVALUATION_PLANT_H
#define EVALUATION_PLANT_H
#include "runtime_motion_primitives.h"
#include <array>
#include <stdexcept>

// Independently integrated evaluation plant. The nominal ODE matches the
// project's point-mass aircraft; RK4 and perturbed coefficients are confined
// to evaluation and are never exposed to the planner.
class EvaluationPlant {
public:
    struct Parameters {
        double dt = .005, actuator_scale = 1., lift_scale = 1.;
        double drag_scale = 1., thrust_scale = 1., wind_x = 0., wind_y = 0.;
    };
    struct Result {
        std::vector<RuntimeMotionPrimitives::s_t> states;
        bool valid = true;
    };
    EvaluationPlant() = default;
    explicit EvaluationPlant(const Parameters& parameters) : p(parameters) {
        if (!(p.dt > 0 && p.dt <= .1 && p.actuator_scale > 0 &&
              p.lift_scale > 0 && p.drag_scale > 0 && p.thrust_scale > 0))
            throw std::invalid_argument("Invalid evaluation plant parameters");
    }
    Result advance(const RuntimeMotionPrimitives::s_t& initial,
                   const RuntimeMotionPrimitives::pr_t& control, double duration) const {
        Result out;
        out.states.push_back(initial);
        Vector y = {initial.pos[0], initial.pos[1], initial.h, initial.V,
                    initial.gamma, initial.sigma, initial.roll, initial.attack, initial.T};
        if (!valid(y) || !std::isfinite(duration) || duration < 0) {out.valid=false;return out;}
        for (double t=0; t < duration-1e-12;) {
            const double h=std::min(p.dt,duration-t);
            const Vector k1=derivative(y,control);
            const Vector k2=derivative(add(y,k1,h/2),control);
            const Vector k3=derivative(add(y,k2,h/2),control);
            const Vector k4=derivative(add(y,k3,h),control);
            for (size_t i=0;i<y.size();++i) y[i]+=h*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6;
            t+=h;
            if(!valid(y)){out.valid=false;break;}
            RuntimeMotionPrimitives::s_t state;
            state.pos[0]=y[0];state.pos[1]=y[1];state.h=y[2];state.V=y[3];
            state.gamma=y[4];state.sigma=std::atan2(std::sin(y[5]),std::cos(y[5]));
            state.roll=y[6];state.attack=y[7];state.T=y[8];
            state.d_sigma=derivative(y,control)[5];
            out.states.push_back(state);
        }
        return out;
    }
private:
    using Vector=std::array<double,9>;
    Parameters p;
    static Vector add(const Vector& y,const Vector& d,double h){
        Vector out=y;for(size_t i=0;i<y.size();++i)out[i]+=h*d[i];return out;
    }
    static bool valid(const Vector& y){
        for(double v:y)if(!std::isfinite(v))return false;
        return y[3]>.5 && std::abs(y[4])<1.45;
    }
    Vector derivative(const Vector& y,const RuntimeMotionPrimitives::pr_t& u) const {
        const double V=y[3],gamma=y[4],sigma=y[5],roll=y[6],alpha=y[7],T=y[8];
        const double CL=p.lift_scale*(.03*std::sin(alpha)+.028);
        const double CD=p.drag_scale*(.001*alpha*alpha+.002);
        const double thrust=p.thrust_scale*T;
        const double normal=thrust*std::sin(alpha)+1.86*V*V*CL;
        return {V*std::cos(gamma)*std::cos(sigma)+p.wind_x,
                V*std::cos(gamma)*std::sin(sigma)+p.wind_y,V*std::sin(gamma),
                thrust*std::cos(alpha)-1.86*V*V*CD-9.81*std::sin(gamma),
                (normal*std::cos(roll)-9.81*std::cos(gamma))/V,
                normal*std::sin(roll)/(V*std::cos(gamma)),
                (u.w_roll_c-roll)/(.5*p.actuator_scale),
                (u.attack_c-alpha)/p.actuator_scale,
                (u.T_c-T)/(.1*p.actuator_scale)};
    }
};
#endif
