// Deterministic, model-in-the-loop mechanism experiment. The plant deliberately
// shares the planning model; these results are not independent flight validation.
#include "motion_primitives.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

struct Cylinder { double x, y, r; };
struct Scenario { std::string name; std::vector<Cylinder> obstacles; };

int main(int argc, char** argv) {
    ros::init(argc, argv, "recovery_benchmark");
    ros::NodeHandle args("~");
    std::string prefix;
    args.param<std::string>("output_prefix", prefix, "/tmp/recovery_benchmark");
    std::ofstream episodes(prefix + "_episodes.csv"), plans(prefix + "_plans.csv"),
                  paths(prefix + "_paths.csv"), geometry(prefix + "_scenarios.csv");
    if (!episodes || !plans || !paths || !geometry) {
        std::cerr << "Cannot open output files; create parent directory first\n";
        return 2;
    }
    episodes << "scenario,variant,initial_y,outcome,steps,sim_seconds,goal_distance\n";
    plans << "scenario,variant,initial_y,step,planning_ms,status,primary_evaluated,primary_feasible,recovery_checked,backup_evaluated,verified,budget_exhausted\n";
    paths << "scenario,variant,initial_y,step,time,x,y,z\n";
    geometry << "scenario,x,y,r,inflated_r,z,height\n";
    const std::vector<Scenario> scenarios = {
        {"open", {}},
        {"offset_pillars", {{75, 5, 12}, {125, -18, 12}, {175, 12, 12}}},
        {"late_wall", {{80, -24, 12}, {80, 0, 12}, {80, 24, 12}}},
        // A deliberately difficult cul-de-sac: report failures without filtering.
        {"cul_de_sac", {{45,-25,8},{65,-25,8},{85,-25,8},{105,-25,8},
                        {45,25,8},{65,25,8},{85,25,8},{105,25,8},
                        {115,-16,8},{115,0,8},{115,16,8}}}
    };
    for (const auto& scene : scenarios) for (const auto& obs : scene.obstacles)
        geometry << scene.name << ',' << obs.x << ',' << obs.y << ',' << obs.r
                 << ',' << obs.r * 1.2 << ",20,100\n";
    const std::vector<std::string> variants = {
        "baseline_3s", "dense_3s", "long_6s", "recovery_3plus3s", "recovery_full"};
    int run = 0;
    for (const auto& scene : scenarios) for (const auto& variant : variants)
    for (double initial_y : {-2.0, 0.0, 2.0}) {
        const std::string ns = "/recovery_benchmark/run_" + std::to_string(++run);
        ros::NodeHandle nh(ns);
        nh.setParam("motion_primitive/horizon_time", variant == "long_6s" ? 6.0 : 3.0);
        nh.setParam("motion_primitive/num", variant == "dense_3s" ? 57 : 29);
        nh.setParam("motion_primitive/vertical_num", variant == "dense_3s" ? 9 : 7);
        // A narrow altitude band makes turning-space failure observable without
        // pretending the vehicle can always escape over a full-height wall.
        nh.setParam("motion_primitive/min_altitude", 16.0);
        nh.setParam("motion_primitive/max_altitude", 24.0);
        nh.setParam("recovery/enabled", variant.find("recovery_") == 0);
        nh.setParam("recovery/horizon_time", 3.0);
        nh.setParam("recovery/max_candidates", variant == "recovery_full" ? 203 : 12);
        nh.setParam("recovery/known_space_radius", 0.0);
        nh.setParam("waypoints/num", 1);
        nh.setParam("waypoints/wp1/x", 250.0);
        nh.setParam("waypoints/wp1/y", 0.0);
        nh.setParam("waypoints/wp1/h", 20.0);
        nh.setParam("static_obstacles/num", static_cast<int>(scene.obstacles.size()));
        for (size_t i = 0; i < scene.obstacles.size(); ++i) {
            const auto& obs = scene.obstacles[i];
            const std::string key = "static_obstacles/obs" + std::to_string(i+1);
            nh.setParam(key + "/x", obs.x); nh.setParam(key + "/y", obs.y);
            nh.setParam(key + "/r", obs.r); nh.setParam(key + "/z", 20.0);
            nh.setParam(key + "/height", 100.0);
        }
        {
            MotionPrimitives planner(nh);
            MotionPrimitives::s_t state;
            state.V = 15.5; state.h = 20; state.pos[1] = initial_y;
            state.T = 0.93; state.attack = -0.20;
            std::string outcome = "timeout";
            int executed = 0;
            for (int step = 0; step < 150; ++step) {
                paths << scene.name << ',' << variant << ',' << initial_y << ',' << step
                      << ',' << step * 0.2 << ',' << state.pos[0] << ',' << state.pos[1]
                      << ',' << state.h << '\n';
                if (std::hypot(250.0-state.pos[0], state.pos[1]) < 15.0 &&
                    std::abs(state.h-20.0) < 3.0) { outcome = "goal"; break; }
                auto begin = std::chrono::steady_clock::now();
                auto decision = planner.select_motion_primitive(state);
                const double elapsed = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now()-begin).count();
                plans << scene.name << ',' << variant << ',' << initial_y << ',' << step
                      << ',' << elapsed << ',' << decision.status << ','
                      << decision.primary_evaluated << ',' << decision.primary_feasible_count
                      << ',' << decision.recovery_checked << ',' << decision.backup_evaluated
                      << ',' << decision.recovery_verified << ',' << decision.budget_exhausted << '\n';
                // Stop on absence of a feasible primary. Do not interpret this
                // offline termination as a safe physical stop for a fixed-wing UAV.
                if (!decision.primary_feasible) { outcome = "no_feasible_primary"; break; }
                auto applied = decision.command;
                planner.remember_command(applied);
                applied.t_way = 0.2;
                auto plant = planner.rollout_motion_primitive(state, applied);
                ++executed;
                state = plant.states.back();
                if (!plant.valid) { outcome = "invalid_dynamics"; break; }
                if (plant.collision) { outcome = "collision"; break; }
                if (std::hypot(250.0-state.pos[0], state.pos[1]) < 15.0 &&
                    std::abs(state.h-20.0) < 3.0) { outcome = "goal"; break; }
            }
            paths << scene.name << ',' << variant << ',' << initial_y << ',' << executed
                  << ',' << executed * 0.2 << ',' << state.pos[0] << ',' << state.pos[1]
                  << ',' << state.h << '\n';
            episodes << scene.name << ',' << variant << ',' << initial_y << ',' << outcome
                     << ',' << executed << ',' << executed * 0.2 << ','
                     << std::hypot(250.0-state.pos[0], state.pos[1]) << '\n';
            std::cout << run << "/60 " << scene.name << ' ' << variant << ' '
                      << initial_y << " -> " << outcome << std::endl;
        }
        ros::param::del(ns);
    }
    return 0;
}
