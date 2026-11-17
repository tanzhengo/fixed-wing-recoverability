// Gazebo runtime planner. Forked once from the frozen paper implementation
// (motion_primitives.h, SHA256 a08773a2f3c79a9f3f481590b8d197ff933f01b1670ccf0e5de31293b588fc73).
// Maintain runtime fixes here. Historical benchmark and training executables
// continue to include motion_primitives.h so their data remain reproducible.
#ifndef RUNTIME_MOTION_PRIMITIVES_H
#define RUNTIME_MOTION_PRIMITIVES_H

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include "learned_recovery.h"
#include "urban_box_geometry.h"
#include "runtime_swept_geometry.h"
#include "runtime_sensor_map.h"

#define Rad2Deg 0.017453292519943295f

class RuntimeMotionPrimitives {
public:
    struct waypoint {
        float d = 0, theata = 0, h = 0, V = 0;
    };
    typedef waypoint wp_t;

    struct primitive {
        float w_roll_c = 0, attack_c = 0, T_c = 0, t_way = 0;
        wp_t wp;
    };
    typedef primitive pr_t;

    // The Gazebo plugin uses custom odometry fields: V, gamma, sigma, T,
    // yaw rate. Its command fields are roll, attack angle and thrust.
    struct state {
        float V = 0, gamma = 0, sigma = 0, d_sigma = 0, T = 0, h = 0;
        float pos[2] = {0, 0};
        float q[4] = {1, 0, 0, 0};
        float roll = 0, attack = 0;
    };
    typedef state s_t;

    struct target {
        float pos[2] = {0, 0};
        float h = 5;
    };
    typedef target target_t;

    struct obstacle {
        float pos[2] = {0, 0};
        float r = 10, z = 0, height = 0;
    };
    typedef obstacle obs_t;

    struct sphere_obstacle {
        float x = 0, y = 0, z = 0, r = 5;
    };

    struct horizontal_obstacle {
        float x = 0, y = 0, z = 0, r = 4, length = 200, yaw = 0;
        float axis_x = 1, axis_y = 0;
    };

    struct box_obstacle_t {
        urban_geometry::Box geometry;
        double clearance = urban_geometry::face_margin;
    };

    // Install complete physical geometry atomically. The same per-face margin
    // is used by hard collision checks and the learned geometry features.
    void set_box_obstacles(const std::vector<box_obstacle_t>& boxes) {
        if (sensor_map_enabled && !boxes.empty())
            throw std::invalid_argument("Sensor-only planning cannot install truth boxes");
        for (const auto& box : boxes)
            if (!urban_geometry::valid(box.geometry) ||
                !std::isfinite(box.clearance) || box.clearance < 0.0)
                throw std::invalid_argument("Invalid box obstacle geometry or clearance");
        box_obstacles_ = boxes;
    }
    const std::vector<box_obstacle_t>& box_obstacles() const { return box_obstacles_; }

    struct dynamic_obstacle {
        float pos[2] = {0, 0}, current_pos[2] = {0, 0}, center[2] = {0, 0};
        float r = 5, z = 5, height = 10;
        std::string type;
        float velocity = 0, direction = 0, radius = 0, angular_velocity = 0, angle = 0;
        ros::Time last_update_time;
    };
    typedef dynamic_obstacle dyn_obs_t;

    enum class rejection_reason_t { NONE, ENVELOPE, COLLISION, KNOWN_SPACE };

    struct rollout_t {
        std::vector<s_t> states;
        bool valid = true;
        bool collision = false;
        bool work_exhausted = false;
        bool early_rejected = false;
        rejection_reason_t early_rejection_reason = rejection_reason_t::NONE;
    };

    // A verified continuation is only a finite, sampled model rollout. It is
    // neither an invariant safe set nor a guarantee about later replanning.
    struct recovery_result_t {
        bool verified = false;
        rollout_t backup;
        pr_t command;
        int checked = 0, integration_steps = 0;
        int rejected_collision = 0, rejected_envelope = 0;
        int rejected_known_space = 0, rejected_invalid = 0, truncated = 0;
        bool work_budget_exhausted = false;
        int early_terminated = 0;
    };

    // Primary hard checks precede all schedulers. Learned scheduling uses the
    // terminal state, cost, and local geometry through recovery_features().
    struct recovery_candidate_t {
        s_t terminal;
        float cost = std::numeric_limits<float>::infinity();
    };

    struct selection_t {
        pr_t command, backup_command;
        rollout_t rollout, backup_rollout;
        std::vector<pr_t> execution_plan;
        float cost = std::numeric_limits<float>::infinity();
        bool primary_feasible = false, recovery_verified = false;
        bool reused_verified_plan = false;
        float plan_remaining_time = 0.0f;
        bool budget_exhausted = false, backup_work_budget_exhausted = false;
        int backup_rejected_collision = 0, backup_rejected_envelope = 0;
        int backup_rejected_known_space = 0, backup_rejected_invalid = 0, backup_truncated = 0;
        int backup_early_terminated = 0;
        int primary_evaluated = 0, primary_feasible_count = 0;
        int recovery_checked = 0, backup_evaluated = 0;
        int primary_integration_steps = 0, backup_integration_steps = 0;
        int cached_integration_steps = 0, cached_segments_checked = 0;
        // Rank is one-based among feasible primaries sorted by original cost.
        // scheduled_count is the planned budget; recovery_checked is actual work.
        int selected_cost_rank = -1, scheduled_count = 0;
        std::string candidate_order = "cost";
        std::string status = "NO_FEASIBLE_PRIMARY";
    };

    explicit RuntimeMotionPrimitives(ros::NodeHandle node) : nh(node) {
        nh.param<bool>("sensor_map/enabled", sensor_map_enabled, false);
        nh.param<bool>("recovery/state_only_features", state_only_features, false);
        nh.param<double>("sensor_map/planning_reserve", sensor_planning_reserve, 0.0);
        nh.param<double>("execution/planning_lookahead", planning_latency_estimate, sensor_map_enabled ? 0.05 : 0.0);
        if (!std::isfinite(sensor_planning_reserve) || sensor_planning_reserve<0 ||
            !std::isfinite(planning_latency_estimate) || planning_latency_estimate<0 || planning_latency_estimate>0.15)
            throw std::invalid_argument("Invalid sensor planning reserve or lookahead");
        planning_lookahead_enabled = planning_latency_estimate>0;
        if (sensor_map_enabled) sensor_map.reset(new RuntimeSensorMap(nh));
        nh.param<float>("model/CL/k1", CL_k1, 0.03f);
        nh.param<float>("model/CL/k2", CL_k2, 0.028f);
        nh.param<float>("model/CD/k1", CD_k1, 0.001f);
        nh.param<float>("model/CD/k2", CD_k2, 0.002f);
        nh.param<float>("motion_primitive/horizon_time", horizon_time, 3.0f);
        nh.param<float>("motion_primitive/V", mp_V, 15.5f);
        nh.param<int>("motion_primitive/num", mp_num, 29);
        nh.param<int>("motion_primitive/vertical_num", vertical_num, 7);
        nh.param<bool>("motion_primitive/state_relative_altitudes", state_relative_altitudes, true);
        load_angle("theata", mp_theata, 45);
        load_angle("max_roll_angle", mp_max_roll, 35);
        load_angle("max_attack_angle", mp_max_attack, 25);
        load_angle("min_attack_angle", mp_min_attack, -20);
        load_angle("max_climb_angle", max_climb_angle, 20);
        load_angle("max_descent_angle", max_descent_angle, 20);
        nh.param<float>("motion_primitive/compensate_tau", tau_a, 0.8f);
        nh.param<float>("motion_primitive/K_T", mp_k_T, 2.2f);
        nh.param<float>("motion_primitive/min_altitude", min_altitude, 1.0f);
        nh.param<float>("motion_primitive/max_altitude", max_altitude, 80.0f);
        nh.param<float>("motion_primitive/vertical_clearance", vertical_clearance, 1.0f);
        nh.param<float>("motion_primitive/integration_dt", integration_dt, 0.02f);
        nh.param<float>("motion_primitive/min_speed", min_speed, 3.0f);
        nh.param<float>("motion_primitive/max_speed", max_speed, 45.0f);
        nh.param<float>("motion_primitive/max_thrust", max_thrust, 20.0f);
        nh.param<float>("motion_primitive/altitude_weight", altitude_weight, 0.15f);
        nh.param<float>("motion_primitive/distance_weight", distance_weight, 0.02f);
        nh.param<float>("motion_primitive/heading_weight", heading_weight, 3.0f);
        nh.param<float>("motion_primitive/waypoint_altitude_tolerance", altitude_tolerance, 3.0f);
        mp_num = std::max(1, mp_num);
        vertical_num = std::max(1, vertical_num);
        if (vertical_num > 1 && vertical_num % 2 == 0) ++vertical_num;
        horizon_time = constraint(horizon_time, 0.1f, 10.0f);
        integration_dt = constraint(integration_dt, 0.005f, 0.05f);
        min_speed = std::max(1.0f, min_speed);
        max_speed = std::max(min_speed + 1.0f, max_speed);
        max_altitude = std::max(min_altitude + 1.0f, max_altitude);
        max_thrust = std::max(0.0f, max_thrust);
        vertical_clearance = std::max(0.0f, vertical_clearance);
        max_climb_angle = constraint(max_climb_angle, 0.0f, 45.0f * Rad2Deg);
        max_descent_angle = constraint(max_descent_angle, 0.0f, 45.0f * Rad2Deg);
        mp_d = horizon_time * mp_V;
        nh.param<float>("motion_primitive/waypoint_horizontal_tolerance", horizontal_tolerance, mp_d + 0.15f * mp_V);
        horizontal_tolerance = std::max(0.1f, horizontal_tolerance);
        nh.param<bool>("recovery/enabled", recovery_enabled, false);
        // The primary-only ablation uses IDENTICAL hard primary checks while
        // omitting terminal validation; disabled preserves the legacy baseline.
        nh.param<bool>("recovery/require_terminal", require_terminal, true);
        // Supplemental fail-fast validation; false preserves V3 full rollouts.
        nh.param<bool>("recovery/stop_on_rejection", stop_on_rejection, false);
        nh.param<int>("recovery/max_backup_integration_steps", max_backup_integration_steps, 0);
        max_backup_integration_steps = std::max(0, max_backup_integration_steps);
        nh.param<bool>("recovery/reuse_verified_plan", reuse_verified_plan, false);
        nh.param<bool>("recovery/strict_execution", strict_execution, false);
        if (strict_execution && (!recovery_enabled || !require_terminal))
            throw std::invalid_argument("recovery/strict_execution requires enabled terminal recovery");
        if (strict_execution) reuse_verified_plan = true;
        nh.param<bool>("execution/compensate_latency", compensate_latency, strict_execution);
        nh.param<float>("execution/max_state_age", max_state_age, 0.5f);
        nh.param<float>("execution/max_validation_age", max_validation_age, 0.02f);
        if (!std::isfinite(max_state_age) || max_state_age <= 0 || max_state_age > 2.0f ||
            !std::isfinite(max_validation_age) || max_validation_age <= 0 || max_validation_age > max_state_age)
            throw std::invalid_argument("Invalid execution state/validation age bounds");
        nh.param<float>("recovery/min_execution_time", min_execution_time, 0.1f);
        if (!std::isfinite(min_execution_time) || min_execution_time <= 0) min_execution_time = 0.1f;
        nh.param<float>("recovery/horizon_time", recovery_horizon, 3.0f);
        nh.param<int>("recovery/max_candidates", recovery_max_candidates, 12);
        nh.param<std::string>("recovery/candidate_order", recovery_order_mode, "cost");
        if (recovery_order_mode != "cost" && recovery_order_mode != "diverse" &&
            recovery_order_mode != "learned" && recovery_order_mode != "learned_diverse") {
            ROS_WARN("Unknown recovery/candidate_order '%s'; using cost", recovery_order_mode.c_str());
            recovery_order_mode = "cost";
        }
        nh.param<float>("recovery/learned_work_power", learned_work_power, 1.0f);
        if (!std::isfinite(learned_work_power)) learned_work_power = 1.0f;
        learned_work_power = constraint(learned_work_power, 0.0f, 4.0f);
        if (recovery_order_mode == "learned" || recovery_order_mode == "learned_diverse") {
            std::string path;
            nh.param<std::string>("recovery/model_path", path, "");
            if (!learned_network.load(path)) {
                ROS_WARN("Could not load recovery network '%s'; using diverse scheduling", path.c_str());
                recovery_order_mode = "diverse";
            }
        }
        nh.param<float>("recovery/known_space_radius", known_space_radius, 0.0f);
        if (!std::isfinite(recovery_horizon)) recovery_horizon = 3.0f;
        recovery_horizon = constraint(recovery_horizon, 0.1f, 10.0f);
        recovery_max_candidates = std::max(1, recovery_max_candidates);
        if (!std::isfinite(known_space_radius)) known_space_radius = 0.0f;
        known_space_radius = std::max(0.0f, known_space_radius);

        nh.param<int>("waypoints/num", wp_n, 1);
        wp_n = std::max(1, wp_n);
        wp_vec.resize(wp_n);
        for (int i = 0; i < wp_n; ++i) {
            const std::string prefix = "waypoints/wp" + std::to_string(i + 1);
            nh.param<float>(prefix + "/x", wp_vec[i].pos[0], 300.0f);
            nh.param<float>(prefix + "/y", wp_vec[i].pos[1], 0.0f);
            nh.param<float>(prefix + "/h", wp_vec[i].h, 5.0f);
        }
        global_target = wp_vec.front();

        load_box_obstacles();
        int count;
        float default_height;
        nh.param<int>("static_obstacles/num", count, 0);
        nh.param<float>("static_obstacles/height", default_height, 36.2743f);
        static_obs_vec.resize(std::max(0, count));
        for (int i = 0; i < count; ++i) {
            const std::string prefix = "static_obstacles/obs" + std::to_string(i + 1);
            obs_t& obs = static_obs_vec[i];
            nh.param<float>(prefix + "/x", obs.pos[0], 110.0f);
            nh.param<float>(prefix + "/y", obs.pos[1], 10.0f);
            nh.param<float>(prefix + "/r", obs.r, 10.0f);
            nh.param<float>(prefix + "/height", obs.height, default_height);
            nh.param<float>(prefix + "/z", obs.z, obs.height / 2.0f);
        }
        nh.param<int>("sphere_obstacles/num", count, 0);
        sphere_obs_vec.resize(std::max(0, count));
        for (int i = 0; i < count; ++i) {
            const std::string prefix = "sphere_obstacles/sphere" + std::to_string(i + 1);
            auto& obs = sphere_obs_vec[i];
            nh.param<float>(prefix + "/x", obs.x, 0.0f);
            nh.param<float>(prefix + "/y", obs.y, 0.0f);
            nh.param<float>(prefix + "/z", obs.z, 10.0f);
            nh.param<float>(prefix + "/r", obs.r, 5.0f);
        }
        nh.param<int>("horizontal_obstacles/num", count, 0);
        horizontal_obs_vec.resize(std::max(0, count));
        for (int i = 0; i < count; ++i) {
            const std::string prefix = "horizontal_obstacles/cylinder" + std::to_string(i + 1);
            auto& obs = horizontal_obs_vec[i];
            nh.param<float>(prefix + "/x", obs.x, 0.0f);
            nh.param<float>(prefix + "/y", obs.y, 0.0f);
            nh.param<float>(prefix + "/z", obs.z, 10.0f);
            nh.param<float>(prefix + "/r", obs.r, 4.0f);
            nh.param<float>(prefix + "/length", obs.length, 200.0f);
            nh.param<float>(prefix + "/axis_yaw", obs.yaw, 0.0f);
            obs.yaw *= Rad2Deg;
            obs.axis_x = std::cos(obs.yaw);
            obs.axis_y = std::sin(obs.yaw);
        }
        nh.param<int>("dynamic_obstacles/num", count, 0);
        nh.param<float>("dynamic_obstacles/height", default_height, 10.0f);
        dyn_obs_vec.resize(std::max(0, count));
        for (int i = 0; i < count; ++i) {
            const std::string prefix = "dynamic_obstacles/dyn_obs" + std::to_string(i + 1);
            dyn_obs_t& obs = dyn_obs_vec[i];
            nh.param<float>(prefix + "/x", obs.pos[0], 0.0f);
            nh.param<float>(prefix + "/y", obs.pos[1], 0.0f);
            nh.param<float>(prefix + "/r", obs.r, 5.0f);
            nh.param<float>(prefix + "/height", obs.height, default_height);
            nh.param<float>(prefix + "/z", obs.z, obs.height / 2.0f);
            nh.param<std::string>(prefix + "/type", obs.type, "linear");
            nh.param<float>(prefix + "/velocity", obs.velocity, 0.0f);
            float direction;
            nh.param<float>(prefix + "/direction", direction, 0.0f);
            obs.direction = direction * Rad2Deg;
            nh.param<float>(prefix + "/angular_velocity", obs.angular_velocity, 0.0f);
            nh.param<float>(prefix + "/radius", obs.radius, 0.0f);
            nh.param<float>(prefix + "/center/x", obs.center[0], 0.0f);
            nh.param<float>(prefix + "/center/y", obs.center[1], 0.0f);
            obs.current_pos[0] = obs.pos[0];
            obs.current_pos[1] = obs.pos[1];
            obs.last_update_time = ros::Time::now();
        }
        if (sensor_map_enabled && (!strict_execution || !box_obstacles_.empty() ||
                !static_obs_vec.empty() || !sphere_obs_vec.empty() ||
                !horizontal_obs_vec.empty() || !dyn_obs_vec.empty()))
            throw std::invalid_argument("Sensor-only planning requires strict execution and no truth obstacles");
        odom_sub = nh.subscribe("odom", 1, &RuntimeMotionPrimitives::odom_cb, this);
        cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        traj_visual_pub = nh.advertise<visualization_msgs::MarkerArray>("traj", 1);
        obs_visual_pub = nh.advertise<visualization_msgs::MarkerArray>("obs", 1);
        goal_visual_pub = nh.advertise<visualization_msgs::Marker>("goal", 1);
        planning_status_pub = nh.advertise<std_msgs::String>("planning_status", 1);
        planning_timer = nh.createTimer(ros::Duration(0.1), &RuntimeMotionPrimitives::planningCallback, this);
    }

    ~RuntimeMotionPrimitives() {
        delete state_ptr;
        delete last_pr_ptr;
    }

    std::vector<pr_t> generate_library(const s_t& s) {
        // Split both sides of the reachable height interval around the goal.
        // This includes goal-height tracking without collapsing all levels onto
        // the ground when starting close to the minimum flight altitude.
        const float low = std::max(min_altitude, s.h - mp_d * std::tan(max_descent_angle));
        const float high = std::min(max_altitude, s.h + mp_d * std::tan(max_climb_angle));
        const float center = constraint(global_target.h, low, high);
        std::vector<float> heights;
        const int half = vertical_num / 2;
        for (int j = -half; j <= half; ++j) {
            const float h = half == 0 ? center : j < 0
                ? center + (center - low) * j / half
                : center + (high - center) * j / half;
            if (heights.empty() || std::abs(h - heights.back()) > 1e-4f)
                heights.push_back(h);
        }
        // A constant-control primitive aimed at a coarse global height layer can
        // exceed the climb-angle envelope over the prediction horizon. Retain
        // the global layers, and fill in gentle state-relative height changes.
        // The fractions are target-direction angles, not relaxed flight limits;
        // every resulting rollout still passes the original hard checks.
        if (vertical_num > 1 && state_relative_altitudes) {
            const auto add_height = [&](float height) {
                height = constraint(height, low, high);
                for (float existing : heights)
                    if (std::abs(existing - height) <= 1e-4f) return;
                heights.push_back(height);
            };
            add_height(s.h);
            for (float fraction : {1.0f / 16.0f, 1.0f / 8.0f, 1.0f / 4.0f}) {
                add_height(s.h + mp_d * std::tan(fraction * max_climb_angle));
                add_height(s.h - mp_d * std::tan(fraction * max_descent_angle));
            }
        }
        std::vector<pr_t> library;
        library.reserve(mp_num * heights.size());
        for (float h : heights) {
            for (int i = 0; i < mp_num; ++i) {
                wp_t wp;
                wp.d = mp_d;
                wp.theata = mp_num == 1 ? 0.0f : -mp_theata + i * 2.0f * mp_theata / (mp_num - 1);
                wp.h = h;
                wp.V = mp_V;
                pr_t pr = generate_unlimited_agility_cmd(wp, s);
                if (tau_a > 0.1f) pr = compensate_cmd(pr, s);
                library.push_back(pr);
            }
        }
        return library;
    }

    // Goal-independent continuation controls. This small library deliberately
    // makes no claim to exhaust all possible recovery maneuvers.
    std::vector<pr_t> generate_recovery_library(const s_t& terminal) {
        std::vector<pr_t> library;
        const float distance = recovery_horizon * std::max(min_speed, terminal.V);
        const float heights[] = {
            constraint(terminal.h, min_altitude, max_altitude),
            constraint(terminal.h + distance * std::tan(max_climb_angle), min_altitude, max_altitude),
            constraint(terminal.h - distance * std::tan(max_descent_angle), min_altitude, max_altitude)
        };
        const float headings[] = {0.0f, -0.5f, 0.5f, -1.0f, 1.0f};
        for (float height : heights) {
            for (float heading : headings) {
                wp_t wp;
                wp.d = distance;
                wp.theata = heading * mp_theata;
                wp.h = height;
                wp.V = mp_V;
                pr_t pr = generate_unlimited_agility_cmd(wp, terminal);
                pr.t_way = recovery_horizon;
                if (tau_a > 0.1f) pr = compensate_cmd(pr, terminal);
                library.push_back(pr);
            }
        }
        return library;
    }

    bool state_within_recovery_envelope(const s_t& state) const {
        return valid_flight_state(state) && state.gamma <= max_climb_angle + 1e-5f &&
            state.gamma >= -max_descent_angle - 1e-5f;
    }

    // Applies the recovery-mode primary filters to an already computed rollout;
    // it does not regenerate geometry or trust an incomplete work-limited path.
    bool check_primary_feasibility(const s_t& observed_start, const rollout_t& primary) const {
        return valid_flight_state(observed_start) && primary.valid && !primary.work_exhausted &&
            !primary.collision && within_recovery_envelope(primary) && within_known_space(observed_start, primary);
    }

    // start_time is relative to the current obstacle snapshot. The optional
    // budget is an exact integration-step cap: -1 means unlimited; 0 permits
    // no backup work. The ROS parameter uses 0 for unlimited at the SELECT level.
    recovery_result_t check_terminal_recovery(const s_t& observed_start,
                                               const rollout_t& primary,
                                               float start_time,
                                               int max_integration_steps = -1) {
        recovery_result_t result;
        if (primary.states.size() < 2 || !check_primary_feasibility(observed_start, primary) ||
            !std::isfinite(start_time) || start_time < 0) return result;
        const s_t& terminal = primary.states.back();
        for (const auto& command : generate_recovery_library(terminal)) {
            const int remaining = max_integration_steps < 0 ? -1 :
                std::max(0, max_integration_steps - result.integration_steps);
            if (remaining == 0) {
                result.work_budget_exhausted = true;
                break;
            }
            ++result.checked;
            const auto backup = rollout_motion_primitive(terminal, command, start_time, remaining,
                stop_on_rejection ? &observed_start : nullptr);
            result.integration_steps += backup.states.empty() ? 0 : static_cast<int>(backup.states.size()) - 1;
            if (backup.work_exhausted) {
                ++result.truncated;
                result.work_budget_exhausted = true;
            }
            // Mutually exclusive rejection reasons, in this priority order.
            // Invalid means the underlying rollout failed (e.g. model envelope),
            // while envelope means its stricter recovery flight limits failed.
            // A truncated path may reveal a definite failure; otherwise only its
            // truncation is counted. It can NEVER provide verification evidence.
            if (backup.early_rejected) {
                ++result.early_terminated;
                // In fail-fast mode classify the FIRST violating sample. A tie
                // at that sample is resolved envelope, collision, known-space.
                if (backup.early_rejection_reason == rejection_reason_t::ENVELOPE) ++result.rejected_envelope;
                else if (backup.early_rejection_reason == rejection_reason_t::COLLISION) ++result.rejected_collision;
                else if (backup.early_rejection_reason == rejection_reason_t::KNOWN_SPACE) ++result.rejected_known_space;
            }
            else if (!backup.valid && !backup.work_exhausted) ++result.rejected_invalid;
            else if (backup.collision) ++result.rejected_collision;
            else if (!within_recovery_envelope(backup)) ++result.rejected_envelope;
            else if (!within_known_space(observed_start, backup)) ++result.rejected_known_space;
            else if (!backup.work_exhausted) {
                result.verified = true;
                result.command = command;
                result.backup = backup;
                break;
            }
            if (backup.work_exhausted) break;
        }
        return result;
    }

    // Shared training/deployment feature encoder, version 1 (103 scalars).
    // No fitted normalization or validation labels enter this representation.
    std::vector<float> recovery_features(const s_t& initial,
                                         const recovery_candidate_t& candidate) const {
        const s_t& terminal = candidate.terminal;
        const float ci = std::cos(initial.sigma), si = std::sin(initial.sigma);
        const float ct = std::cos(terminal.sigma), st = std::sin(terminal.sigma);
        const float dx = terminal.pos[0] - initial.pos[0];
        const float dy = terminal.pos[1] - initial.pos[1];
        const float gx = global_target.pos[0] - terminal.pos[0];
        const float gy = global_target.pos[1] - terminal.pos[1];
        const float delta = terminal.sigma - initial.sigma;
        std::vector<float> feature = {
            terminal.V / 20.0f, terminal.gamma / 0.35f, terminal.roll / 0.61f,
            terminal.attack / 0.44f, terminal.T / 20.0f, terminal.d_sigma,
            initial.V / 20.0f, initial.gamma / 0.35f, initial.roll / 0.61f,
            initial.attack / 0.44f, initial.T / 20.0f, initial.d_sigma,
            (dx * ci + dy * si) / 100.0f, (-dx * si + dy * ci) / 100.0f,
            (terminal.h - initial.h) / 30.0f, std::sin(delta), std::cos(delta),
            (gx * ct + gy * st) / 250.0f, (-gx * st + gy * ct) / 250.0f,
            (global_target.h - terminal.h) / 50.0f,
            (terminal.h - min_altitude) / 50.0f, (max_altitude - terminal.h) / 50.0f,
            horizon_time / 10.0f, recovery_horizon / 10.0f, known_space_radius / 200.0f,
            known_space_radius > 0.0f ?
                std::sqrt(dx * dx + dy * dy + (terminal.h - initial.h) * (terminal.h - initial.h)) / known_space_radius : 0.0f,
            candidate.cost / 20.0f, vertical_clearance / 10.0f
        };
        feature.reserve(LearnedRecoveryNetwork::input_size);
        if (state_only_features) {
            feature.resize(LearnedRecoveryNetwork::input_size, 0.0f);
            return feature;
        }
        // These spatial probes describe geometry at primary terminal time. The
        // explicit validator retains its time-varying dynamic-obstacle checks.
        for (float forward : {0.0f, 22.5f, 45.0f, 67.5f, 90.0f})
            for (float lateral : {-60.0f, -30.0f, 0.0f, 30.0f, 60.0f})
                for (float height : {-15.0f, 0.0f, 15.0f}) {
                    const float x = terminal.pos[0] + ct * forward - st * lateral;
                    const float y = terminal.pos[1] + st * forward + ct * lateral;
                    const float clearance = obstacle_signed_distance(x, y, terminal.h + height, horizon_time);
                    feature.push_back(std::max(-1.0f, std::min(1.0f, clearance / 30.0f)));
                }
        return feature;
    }

    // Signed distance to the union of the same inflated geometric obstacles
    // used by collision_check(). Altitude limits are encoded separately above.
    float obstacle_signed_distance(float x, float y, float z,
                                   float prediction_time = 0.0f) const {
        if (sensor_map_enabled) return sensor_map->distance(x,y,z);
        float distance = 30.0f;
        for (const auto& box : box_obstacles_)
            distance = std::min(distance, static_cast<float>(urban_geometry::signed_distance(
                box.geometry, x, y, z, box.clearance)));
        const auto capped_cylinder = [](float radial, float axial) {
            return std::hypot(std::max(radial, 0.0f), std::max(axial, 0.0f)) +
                std::min(std::max(radial, axial), 0.0f);
        };
        const auto vertical = [&](float ox, float oy, float oz, float radius, float height) {
            return capped_cylinder(std::hypot(x - ox, y - oy) - 1.2f * radius,
                std::abs(z - oz) - height / 2.0f - vertical_clearance);
        };
        for (const auto& o : static_obs_vec)
            distance = std::min(distance, vertical(o.pos[0], o.pos[1], o.z, o.r, o.height));
        for (const auto& o : sphere_obs_vec) {
            const float margin = std::max(0.2f * o.r, vertical_clearance);
            distance = std::min(distance, std::sqrt((x-o.x)*(x-o.x) +
                (y-o.y)*(y-o.y) + (z-o.z)*(z-o.z)) - o.r - margin);
        }
        for (const auto& o : horizontal_obs_vec) {
            const float dx = x - o.x, dy = y - o.y;
            const float axial = dx * o.axis_x + dy * o.axis_y;
            const float transverse = -dx * o.axis_y + dy * o.axis_x;
            const float margin = std::max(0.2f * o.r, vertical_clearance);
            distance = std::min(distance, capped_cylinder(std::hypot(transverse, z - o.z) - o.r - margin,
                std::abs(axial) - o.length / 2.0f - margin));
        }
        for (const auto& o : dyn_obs_vec) {
            float ox = o.current_pos[0], oy = o.current_pos[1];
            if (prediction_time > 0 && o.type == "linear") {
                ox += o.velocity * std::cos(o.direction) * prediction_time;
                oy += o.velocity * std::sin(o.direction) * prediction_time;
            } else if (prediction_time > 0 && o.type == "circular") {
                const float angle = o.angle + o.angular_velocity * prediction_time;
                ox = o.center[0] + o.radius * std::cos(angle);
                oy = o.center[1] + o.radius * std::sin(angle);
            }
            distance = std::min(distance, vertical(ox, oy, o.z, o.r, o.height));
        }
        return distance;
    }

    // Return original input indices under the configured validation budget.
    // Deterministic farthest-point coverage supplements, but cannot establish,
    // recoverability: every scheduled primary still needs explicit validation.
    std::vector<size_t> recovery_candidate_order(const s_t& initial,
            const std::vector<recovery_candidate_t>& candidates) const {
        std::vector<size_t> cost_order;
        if (!finite_state(initial) || initial.V <= 0) return cost_order;
        for (size_t i = 0; i < candidates.size(); ++i) {
            const auto& candidate = candidates[i];
            if (std::isfinite(candidate.cost) && std::isfinite(candidate.terminal.sigma) &&
                std::isfinite(candidate.terminal.h)) cost_order.push_back(i);
        }
        std::stable_sort(cost_order.begin(), cost_order.end(),
            [&](size_t a, size_t b) { return candidates[a].cost < candidates[b].cost; });
        const size_t limit = std::min(cost_order.size(), static_cast<size_t>(recovery_max_candidates));
        if (recovery_order_mode == "cost" || limit < 2) {
            cost_order.resize(limit);
            return cost_order;
        }
        // Normalize by the nominal physical maneuver span, NOT the observed
        // candidate spread. In a narrow altitude band, millimeter differences
        // consequently remain small instead of becoming a full extra dimension.
        const float speed = std::max(min_speed, initial.V);
        const float heading_span = std::min(static_cast<float>(M_PI),
            gravity * std::abs(std::tan(mp_max_roll)) * horizon_time / speed);
        const float height_span = speed * horizon_time *
            std::max(std::tan(max_climb_angle), std::tan(max_descent_angle));
        const auto distance_squared = [&](size_t a, size_t b) {
            const auto& first = candidates[a].terminal;
            const auto& second = candidates[b].terminal;
            // Circular difference prevents a false split at heading +/-pi.
            const float heading = heading_span > 1e-5f
                ? abs_err_angle(first.sigma, second.sigma) / heading_span : 0.0f;
            const float height = height_span > 1e-5f
                ? (first.h - second.h) / height_span : 0.0f;
            return heading * heading + height * height;
        };
        std::vector<size_t> learned_order = cost_order;
        bool use_learned = (recovery_order_mode == "learned" || recovery_order_mode == "learned_diverse") &&
            learned_network.ready();
        if (use_learned) {
            std::vector<float> utility(candidates.size(), 0.0f);
            for (size_t i : cost_order) {
                std::array<float, 2> output;
                if (!learned_network.predict(recovery_features(initial, candidates[i]), output)) {
                    use_learned = false;
                    ROS_WARN_THROTTLE(5.0, "Nonfinite recovery network inference; using diverse scheduling");
                    break;
                }
                utility[i] = LearnedRecoveryNetwork::probability(output[0]) /
                    std::pow(1.0f + LearnedRecoveryNetwork::predicted_steps(output[1]) / 1000.0f,
                             learned_work_power);
            }
            if (use_learned) std::stable_sort(learned_order.begin(), learned_order.end(),
                [&](size_t a, size_t b) { return utility[a] > utility[b]; });
        }
        std::vector<size_t> selected;
        selected.reserve(limit);
        std::vector<bool> used(candidates.size(), false);
        std::vector<float> closest(candidates.size(), std::numeric_limits<float>::infinity());
        selected.push_back(cost_order.front());
        used[selected.back()] = true;
        while (selected.size() < limit) {
            size_t next = candidates.size();
            float farthest = -1.0f;
            for (size_t i : cost_order) {
                if (used[i]) continue;
                closest[i] = std::min(closest[i], distance_squared(i, selected.back()));
                // Iteration follows stable cost order, resolving equal-distance
                // ties by cost and then the original input index.
                if (closest[i] > farthest + 1e-7f) {
                    farthest = closest[i];
                    next = i;
                }
            }
            // After the common cheapest-first seed, the hybrid alternates
            // learned ranking and farthest-first exploration. Distances above
            // are updated against every selected member in either mode.
            if (use_learned && (recovery_order_mode == "learned" || selected.size() % 2 == 1)) {
                for (size_t i : learned_order) if (!used[i]) { next = i; break; }
            }
            if (next == candidates.size()) break;
            selected.push_back(next);
            used[next] = true;
        }
        return selected;
    }

    // Nominal search uses a larger observed-free margin. Cached controls retain
    // their minimum-clearance witness and are checked at actual dispatch time,
    // not at a forecast state with unconsumed durations.
    selection_t select_planning_proposal(const s_t& initial,
                                         std::vector<rollout_t>* predictions = nullptr) {
        FlagScope scope(planning_guard_active, sensor_map_enabled || planning_lookahead_enabled);
        return select_motion_primitive(initial,predictions);
    }

    double sensor_reserve_at(float prediction_time) const {
        return sensor_planning_reserve * std::max(0.0, std::min(1.0,
            (static_cast<double>(prediction_time)-0.2)/0.3));
    }

    // The optional predictions output avoids recomputing primary rollouts for
    // visualization. Calling this method alone does not publish or execute.
    selection_t select_motion_primitive(const s_t& initial,
                                         std::vector<rollout_t>* predictions = nullptr) {
        selection_t result;
        result.candidate_order = recovery_order_mode;
        if (predictions) predictions->clear();
        if (!valid_flight_state(initial)) {
            cached_execution_plan.clear();
            result.status = "INVALID_STATE";
            result.rollout.valid = false;
            return result;
        }
        struct candidate_t { pr_t command; rollout_t rollout; float cost; };
        std::vector<candidate_t> feasible;
        const auto library = generate_library(initial);
        for (const auto& command : library) {
            const auto prediction = rollout_motion_primitive(initial, command);
            ++result.primary_evaluated;
            result.primary_integration_steps += prediction.states.empty() ? 0 : static_cast<int>(prediction.states.size()) - 1;
            if (predictions) predictions->push_back(prediction);
            const float cost = evaluate_motion_primitive(initial, command, prediction);
            const bool is_feasible = prediction.valid && !prediction.collision &&
                std::isfinite(cost) && (!recovery_enabled ||
                    (within_recovery_envelope(prediction) && within_known_space(initial, prediction)));
            if (is_feasible) {
                ++result.primary_feasible_count;
                feasible.push_back({command, prediction, cost});
            }
            // Preserve the original baseline's soft collision penalty when the
            // experiment switch is off, including its explicitly unsafe fallback.
            if (!recovery_enabled && cost < result.cost) {
                result.command = command;
                result.rollout = prediction;
                result.cost = cost;
            }
        }
        // As in the baseline, only try the previous control after the sampled
        // library has no usable primary. It must pass the same hard filters.
        if (last_pr_ptr && (recovery_enabled ? feasible.empty() : result.cost >= collision_cost)) {
            // A reused witness may have a very short remaining front segment.
            // Reusing its control as a NEW primary must still cover the full
            // configured horizon; only the cache path may execute a short tail.
            pr_t previous = *last_pr_ptr;
            previous.t_way = horizon_time;
            const auto prediction = rollout_motion_primitive(initial, previous);
            ++result.primary_evaluated;
            result.primary_integration_steps += prediction.states.empty() ? 0 : static_cast<int>(prediction.states.size()) - 1;
            const float cost = evaluate_motion_primitive(initial, previous, prediction);
            if (prediction.valid && !prediction.collision && std::isfinite(cost) &&
                (!recovery_enabled ||
                    (within_recovery_envelope(prediction) && within_known_space(initial, prediction)))) {
                ++result.primary_feasible_count;
                feasible.push_back({previous, prediction, cost});
            }
            if (!recovery_enabled && cost < result.cost) {
                result.command = previous;
                result.rollout = prediction;
                result.cost = cost;
            }
        }
        if (recovery_enabled && !feasible.empty()) {
            std::stable_sort(feasible.begin(), feasible.end(),
                [](const candidate_t& a, const candidate_t& b) { return a.cost < b.cost; });
            result.command = feasible.front().command;
            result.rollout = feasible.front().rollout;
            result.cost = feasible.front().cost;
            result.primary_feasible = true;
            result.selected_cost_rank = 1;
            if (!require_terminal) {
                result.status = "PRIMARY_ONLY_FEASIBLE";
                return result;
            }
            std::vector<recovery_candidate_t> summaries;
            summaries.reserve(feasible.size());
            for (const auto& candidate : feasible) {
                recovery_candidate_t summary;
                summary.terminal = candidate.rollout.states.back();
                summary.cost = candidate.cost;
                summaries.push_back(summary);
            }
            const auto schedule = recovery_candidate_order(initial, summaries);
            result.scheduled_count = static_cast<int>(schedule.size());
            for (size_t i : schedule) {
                const auto& candidate = feasible[i];
                const int remaining_work = max_backup_integration_steps == 0 ? -1 :
                    std::max(0, max_backup_integration_steps - result.backup_integration_steps);
                if (remaining_work == 0) {
                    result.backup_work_budget_exhausted = true;
                    break;
                }
                ++result.recovery_checked;
                const auto recovery = check_terminal_recovery(initial, candidate.rollout, candidate.command.t_way, remaining_work);
                result.backup_evaluated += recovery.checked;
                result.backup_integration_steps += recovery.integration_steps;
                result.backup_rejected_collision += recovery.rejected_collision;
                result.backup_rejected_envelope += recovery.rejected_envelope;
                result.backup_rejected_known_space += recovery.rejected_known_space;
                result.backup_rejected_invalid += recovery.rejected_invalid;
                result.backup_truncated += recovery.truncated;
                result.backup_early_terminated += recovery.early_terminated;
                result.backup_work_budget_exhausted = recovery.work_budget_exhausted;
                if (!recovery.verified) {
                    if (recovery.work_budget_exhausted) break;
                    continue;
                }
                // A coverage schedule is not cost-ordered. Keep the cheapest
                // verified member instead of committing to its first success.
                if (!result.recovery_verified || candidate.cost < result.cost ||
                    (candidate.cost == result.cost && static_cast<int>(i) + 1 < result.selected_cost_rank)) {
                    result.command = candidate.command;
                    result.rollout = candidate.rollout;
                    result.cost = candidate.cost;
                    result.backup_command = recovery.command;
                    result.backup_rollout = recovery.backup;
                    result.execution_plan = {candidate.command, recovery.command};
                    result.plan_remaining_time = candidate.command.t_way + recovery.command.t_way;
                    result.selected_cost_rank = static_cast<int>(i) + 1;
                    result.recovery_verified = true;
                    result.status = "RECOVERY_VERIFIED";
                }
                // Cost mode retains its previous first-success behavior. All
                // modes may stop if the globally lowest-cost primary passes.
                if (recovery_order_mode == "cost" || i == 0) return result;
            }
            if (result.recovery_verified) {
                result.budget_exhausted = result.backup_work_budget_exhausted;
                return result;
            }
            result.budget_exhausted = result.backup_work_budget_exhausted || schedule.size() < feasible.size();
            result.status = result.backup_work_budget_exhausted ? "BACKUP_WORK_BUDGET_EXHAUSTED" :
                (result.budget_exhausted ? "BUDGET_EXHAUSTED" : "DEGRADED_NO_CONTINUATION");
            try_reuse_cached_plan(initial, result);
            return result;
        }
        if (!recovery_enabled && std::isfinite(result.cost)) {
            result.primary_feasible = result.rollout.valid && !result.rollout.collision;
            result.selected_cost_rank = result.primary_feasible ? 1 : -1;
            result.status = result.primary_feasible ? "BASELINE_FEASIBLE" : "BASELINE_UNSAFE";
            return result;
        }
        if (recovery_enabled && try_reuse_cached_plan(initial, result)) return result;
        // Fixed-wing aircraft cannot stop in place. Retain the existing bounded
        // climb command as an UNSAFE best-effort fallback, and expose the failure.
        // This is not a verified recovery and must be counted as an intervention.
        wp_t emergency;
        emergency.d = mp_d;
        emergency.V = mp_V;
        emergency.h = constraint(initial.h + 5.0f, min_altitude, max_altitude);
        result.command = generate_unlimited_agility_cmd(emergency, initial);
        result.rollout = rollout_motion_primitive(initial, result.command);
        result.primary_integration_steps += result.rollout.states.empty() ? 0 : static_cast<int>(result.rollout.states.size()) - 1;
        result.status = recovery_enabled ? "NO_FEASIBLE_PRIMARY" : "BASELINE_UNSAFE";
        return result;
    }

    // Explicit external control history invalidates any cached execution
    // witness. Existing baselines and matched-state experiments use this API.
    void remember_command(const pr_t& command) {
        cached_execution_plan.clear();
        if (!last_pr_ptr) last_pr_ptr = new pr_t;
        *last_pr_ptr = command;
    }

    // Record a selection only when its first control is accepted for execution.
    // This stores an earlier witness; it does NOT verify it in the current map.
    // Every reuse starts from the then-current measured state and revalidates.
    void accept_selection(const selection_t& selection) {
        // A rejected proposal must not erase the still-running verified plan.
        if (!selection_is_executable(selection)) return;
        remember_command(selection.command);
        if (!recovery_enabled || !require_terminal || !reuse_verified_plan || !selection.primary_feasible ||
            selection.execution_plan.empty()) return;
        const bool fresh = selection.status == "RECOVERY_VERIFIED" && selection.recovery_verified;
        const bool reused = selection.status == "REVALIDATED_PLAN" && selection.reused_verified_plan &&
            !selection.recovery_verified;
        if (!fresh && !reused) return;
        if (fresh && selection.execution_plan.size() < 2) return;
        const auto& first = selection.execution_plan.front();
        if (!same_control(first, selection.command)) return;
        for (const auto& command : selection.execution_plan)
            if (!valid_cached_command(command)) return;
        cached_execution_plan = selection.execution_plan;
    }

    // Account for actual elapsed execution time, never the nominal timer period.
    // A nonfinite/negative duration invalidates the witness; zero is a no-op.
    void advance_execution(float elapsed) {
        if (!std::isfinite(elapsed) || elapsed < 0) {
            cached_execution_plan.clear();
            return;
        }
        while (!cached_execution_plan.empty() && elapsed > 0) {
            auto& first = cached_execution_plan.front();
            if (!valid_cached_command(first)) {
                cached_execution_plan.clear();
                return;
            }
            if (elapsed + 1e-6f >= first.t_way) {
                elapsed = std::max(0.0f, elapsed - first.t_way);
                cached_execution_plan.erase(cached_execution_plan.begin());
            } else {
                first.t_way -= elapsed;
                elapsed = 0;
            }
        }
    }

    float remaining_execution_time() const {
        float total = 0.0f;
        for (const auto& command : cached_execution_plan) total += command.t_way;
        return total;
    }

    std::vector<pr_t> remaining_execution_plan() const { return cached_execution_plan; }

    bool selection_is_executable(const selection_t& selection) const {
        if (selection.status == "INVALID_STATE" || selection.status == "EXECUTION_REJECTED") return false;
        if (!strict_execution) return true;
        const bool fresh = selection.status == "RECOVERY_VERIFIED" && selection.recovery_verified &&
            selection.execution_plan.size() >= 2;
        const bool reused = selection.status == "REVALIDATED_PLAN" && selection.reused_verified_plan &&
            !selection.execution_plan.empty();
        return selection.primary_feasible && (fresh || reused) &&
            same_control(selection.command, selection.execution_plan.front());
    }

    // Keep the controls actually sent to the plant, including their timestamps.
    // A delayed odometry sample can straddle more than one control update.
    void record_applied_command(const pr_t& command, const ros::Time& stamp) {
        if (!applied_commands.empty() && stamp < applied_commands.back().stamp) applied_commands.clear();
        // The paired Gazebo plugin initializes all desired actuator controls
        // to zero. Keep that interval when odometry predates the first publish.
        if (applied_commands.empty()) applied_commands.push_back({ros::Time(0), pr_t()});
        applied_commands.push_back({stamp, command});
        const double cutoff = stamp.toSec() - max_state_age - 1.0;
        while (applied_commands.size() > 1 && applied_commands[1].stamp.toSec() < cutoff)
            applied_commands.erase(applied_commands.begin());
    }

    // Model propagation only: collision validation is performed on the plan
    // starting at the propagated state against the current obstacle snapshot.
    bool predict_state_at(const s_t& observed, const ros::Time& observed_time,
                          const ros::Time& target_time, s_t& predicted) const {
        const double elapsed = (target_time - observed_time).toSec();
        if (!valid_flight_state(observed) || observed_time.isZero() ||
            !std::isfinite(elapsed) || elapsed < -1e-6 || elapsed > max_state_age) return false;
        predicted = observed;
        if (elapsed <= 1e-6) return true;
        pr_t active;
        bool covered = applied_commands.empty();  // Paired plugin's zero startup control.
        for (const auto& entry : applied_commands) {
            if (entry.stamp > observed_time) break;
            active = entry.command;
            covered = true;
        }
        if (!covered) return false;  // Do not invent an unrecorded earlier command.
        ros::Time cursor = observed_time;
        const auto propagate = [&](const ros::Time& until) {
            const float duration = static_cast<float>((until - cursor).toSec());
            if (duration <= 1e-6f) { cursor = until; return true; }
            active.t_way = duration;
            const auto segment = rollout_motion_primitive(predicted, active);
            if (!segment.valid || segment.states.empty() || !within_recovery_envelope(segment) ||
                (sensor_map_enabled && segment.collision)) return false;
            predicted = segment.states.back();
            cursor = until;
            return true;
        };
        for (const auto& entry : applied_commands) {
            if (entry.stamp <= observed_time) continue;
            if (entry.stamp >= target_time) break;
            if (!propagate(entry.stamp)) return false;
            active = entry.command;
        }
        return propagate(target_time);
    }

    // Recheck the complete command sequence from its dispatch state. Cached
    // durations have already been consumed up to that same timestamp.
    bool prepare_execution(const s_t& dispatch_state, selection_t& selection) {
        if (selection.reused_verified_plan) {
            if (try_reuse_cached_plan(dispatch_state, selection)) return true;
        } else if (selection_is_executable(selection) && revalidate_selection(dispatch_state, selection)) {
            return true;
        }
        if (try_reuse_cached_plan(dispatch_state, selection)) return true;
        selection.status = "EXECUTION_REJECTED";
        selection.primary_feasible = false;
        selection.recovery_verified = false;
        selection.reused_verified_plan = false;
        selection.execution_plan.clear();
        return false;
    }

    // Independently callable for diagnostics/tests. REVALIDATED_PLAN reports
    // only a shrinking, rechecked remainder; it is not a fresh terminal witness
    // and cannot support an invariant-set or indefinite-flight claim.
    selection_t revalidate_cached_plan(const s_t& actual) {
        FlagScope scope(planning_guard_active,false);
        selection_t result;
        result.candidate_order = recovery_order_mode;
        result.status = "NO_CACHED_PLAN";
        if (!recovery_enabled || !require_terminal || !reuse_verified_plan || cached_execution_plan.empty()) return result;
        result.status = "CACHED_PLAN_REJECTED";
        // A short front tail cannot cover another control interval. Try an
        // early transition to the next control, rechecking its entire remainder
        // from the actual state rather than assuming the old terminal state.
        auto remaining = cached_execution_plan;
        while (remaining.size() > 1 && remaining.front().t_way + 1e-5f < min_execution_time)
            remaining.erase(remaining.begin());
        if (!valid_flight_state(actual) || remaining.front().t_way + 1e-5f < min_execution_time) {
            cached_execution_plan.clear();
            return result;
        }
        s_t state = actual;
        float start_time = 0.0f;
        std::vector<rollout_t> predictions;
        for (const auto& command : remaining) {
            if (!valid_cached_command(command)) {
                cached_execution_plan.clear();
                return result;
            }
            ++result.cached_segments_checked;
            const auto predicted = rollout_motion_primitive(state, command, start_time);
            result.cached_integration_steps += predicted.states.empty() ? 0 :
                static_cast<int>(predicted.states.size()) - 1;
            if (!predicted.valid || predicted.collision || !within_recovery_envelope(predicted) ||
                !within_known_space(actual, predicted)) {
                cached_execution_plan.clear();
                return result;
            }
            predictions.push_back(predicted);
            state = predicted.states.back();
            start_time += command.t_way;
        }
        cached_execution_plan = remaining;
        result.command = remaining.front();
        result.rollout = predictions.front();
        for (size_t i = 1; i < predictions.size(); ++i) {
            const auto& states = predictions[i].states;
            const size_t skip = result.backup_rollout.states.empty() ? 0 : 1;
            result.backup_rollout.states.insert(result.backup_rollout.states.end(), states.begin() + skip, states.end());
        }
        if (remaining.size() > 1) result.backup_command = remaining[1];
        result.cost = evaluate_motion_primitive(actual, result.command, result.rollout);
        result.primary_feasible = true;
        result.reused_verified_plan = true;
        result.execution_plan = remaining;
        result.plan_remaining_time = start_time;
        result.status = "REVALIDATED_PLAN";
        return result;
    }

    void planningCallback(const ros::TimerEvent&) {
        if (!state_ptr) return;
        const ros::Time planning_time = ros::Time::now();
        const s_t observed = *state_ptr;
        const ros::Time observed_stamp = state_stamp;
        account_execution_until(planning_time);
        updateDynamicObstacles();
        if (sensor_map_enabled && !sensor_map->ready(planning_time.toSec())) {
            std::ostringstream json;json<<std::setprecision(17)
                <<"{\"status\":\""<<(have_sensor_execution ? "EXECUTION_REJECTED" : "MAP_WAITING")
                <<"\",\"primary_feasible\":false,\"command_published\":false,\"sensor_map_enabled\":true,\"truth_obstacle_count\":0"
                <<",\"map_ready\":false,\"map_stamp\":"<<sensor_map->stamp()
                <<",\"map_age\":"<<(planning_time.toSec()-sensor_map->stamp())
                <<",\"strict_execution\":true,\"execution_rejection_reason\":\"MAP_UNAVAILABLE_OR_STALE\"}";
            std_msgs::String status;status.data=json.str();planning_status_pub.publish(status);return;
        }
        check_target(observed);
        s_t planning_state = observed;
        double lookahead = compensate_latency && planning_lookahead_enabled ? planning_latency_estimate : 0.0;
        ros::Time proposal_state_stamp = compensate_latency ? planning_time+ros::Duration(lookahead) : observed_stamp;
        bool input_valid = !compensate_latency ||
            predict_state_at(observed, observed_stamp, proposal_state_stamp, planning_state);
        bool forecast_fallback = false;
        if (!input_valid && compensate_latency && lookahead>0) {
            // A speculative old-control bridge can leave observed space even
            // when a different command is feasible now. Keep current-time
            // planning available; an invalid past bridge still rejects input.
            input_valid = predict_state_at(observed,observed_stamp,planning_time,planning_state);
            if (input_valid) { lookahead=0;proposal_state_stamp=planning_time;forecast_fallback=true; }
        }
        std::vector<rollout_t> predictions;
        selection_t selected;
        if (input_valid) selected = select_planning_proposal(planning_state, &predictions);
        else selected.status = "EXECUTION_REJECTED";
        const double generation_duration = input_valid ? std::max(0.0,(ros::Time::now()-planning_time).toSec()) : 0.0;
        if (planning_lookahead_enabled && input_valid) {
            const double measured = std::min(0.15,generation_duration);
            planning_latency_estimate = 0.5*planning_latency_estimate+0.5*measured;
        }
        const std::string proposal_status = selected.status;
        std::string execution_rejection_reason = input_valid ? "" : "STATE_ESTIMATE_INVALID_OR_STALE";
        ros::Time dispatch_state_stamp = compensate_latency ? planning_time : observed_stamp;
        bool executable = input_valid && selection_is_executable(selected);
        if (input_valid && (strict_execution || compensate_latency)) {
            // Revalidation is much cheaper than generating every candidate.
            // Retry once if its own computation exceeds the configured age.
            for (int attempt = 0; attempt < 2; ++attempt) {
                const ros::Time dispatch_time = ros::Time::now();
                account_execution_until(dispatch_time);
                updateDynamicObstacles();  // Same epoch as the dispatch-state check.
                s_t dispatch_state = observed;
                dispatch_state_stamp = compensate_latency ? dispatch_time : observed_stamp;
                if (compensate_latency && !predict_state_at(observed, observed_stamp, dispatch_time, dispatch_state)) {
                    executable = false;
                    execution_rejection_reason = "STATE_ESTIMATE_INVALID_OR_STALE";
                    break;
                }
                executable = prepare_execution(dispatch_state, selected);
                if (!executable) {
                    execution_rejection_reason = "NO_REVALIDATED_EXECUTION_PLAN";
                    break;
                }
                if (!compensate_latency || (ros::Time::now() - dispatch_time).toSec() <= max_validation_age) break;
                executable = false;
                execution_rejection_reason = "DISPATCH_VALIDATION_TOO_OLD";
            }
        }
        if (!executable && (strict_execution || compensate_latency)) {
            selected.status = sensor_map_enabled && !have_sensor_execution ? "MAP_WAITING" : "EXECUTION_REJECTED";
            selected.primary_feasible = false;
            selected.recovery_verified = false;
            selected.reused_verified_plan = false;
            selected.execution_plan.clear();
            if (selected.status == "MAP_WAITING")
                ROS_INFO_THROTTLE(1.0, "Collecting observed free space before first validated control");
            else ROS_ERROR_THROTTLE(1.0, "Execution rejected (%s); no new control published, supervisor intervention required",
                                   execution_rejection_reason.c_str());
        }
        // Send the checked control before rendering/publishing hundreds of
        // visualization markers; that work must not age the dispatch state.
        ros::Time command_stamp;
        if (executable) {
            accept_selection(selected);
            geometry_msgs::Twist command;
            command.linear.x = selected.command.w_roll_c;
            command.linear.y = selected.command.attack_c;
            command.linear.z = selected.command.T_c;
            command_stamp = ros::Time::now();
            cmd_pub.publish(command);
            record_applied_command(selected.command, command_stamp);
            if (sensor_map_enabled) have_sensor_execution = true;
            // A compensated plan was checked from dispatch_state_stamp. Charge
            // its validation duration too, for fresh and reused plans alike.
            last_command_time = compensate_latency ? dispatch_state_stamp :
                (selected.reused_verified_plan ? last_command_time : command_stamp);
            have_command_time = true;
            execution_rejection_reason.clear();
        }
        visualization_msgs::MarkerArray markers;
        int id = 1;
        for (const auto& prediction : predictions) {
            auto marker = visual_motion_primitive(prediction);
            marker.ns = "traj" + std::to_string(id);
            marker.id = id++;
            markers.markers.push_back(marker);
        }
        if (!selected.primary_feasible && selected.status != "EXECUTION_REJECTED")
            ROS_WARN_THROTTLE(2.0, "No feasible 3D primitive; publishing best-effort emergency only if state is valid");
        else if (selected.primary_feasible && recovery_enabled && require_terminal && !selected.recovery_verified && !selected.reused_verified_plan)
            ROS_WARN_THROTTLE(2.0, "Primary trajectory is feasible but finite continuation is not verified: %s", selected.status.c_str());
        auto optimal = visual_motion_primitive(selected.rollout);
        optimal.ns = "traj_optimal";
        optimal.id = 1;
        optimal.color.r = 1.0f;
        optimal.color.g = 0.0f;
        optimal.color.b = !selected.primary_feasible ? 1.0f : 0.0f;
        optimal.color.a = 1.0f;
        optimal.scale.x = 0.45f;
        markers.markers.push_back(optimal);
        auto backup = visual_motion_primitive(selected.backup_rollout);
        backup.ns = "traj_continuation";
        backup.id = 1;
        backup.color.r = 0.0f;
        backup.color.g = backup.color.b = 1.0f;
        backup.color.a = 1.0f;
        backup.scale.x = 0.35f;
        if ((!selected.recovery_verified && !selected.reused_verified_plan) ||
            selected.backup_rollout.states.empty()) backup.action = visualization_msgs::Marker::DELETE;
        markers.markers.push_back(backup);
        for (int old_id = id; old_id <= previous_marker_count; ++old_id) {
            visualization_msgs::Marker removed;
            removed.ns = "traj" + std::to_string(old_id);
            removed.id = old_id;
            removed.action = visualization_msgs::Marker::DELETE;
            markers.markers.push_back(removed);
        }
        previous_marker_count = id - 1;
        traj_visual_pub.publish(markers);
        obs_visual_pub.publish(collision_visual());
        goal_visual_pub.publish(goal_visual());
        std::ostringstream json;
        json << std::setprecision(17);
        json << "{\"status\":\"" << selected.status << "\",\"primary_feasible\":"
             << (selected.primary_feasible ? "true" : "false")
             << ",\"proposal_status\":\"" << proposal_status << "\""
             << ",\"strict_execution\":" << (strict_execution ? "true" : "false")
             << ",\"sensor_map_enabled\":" << (sensor_map_enabled ? "true" : "false")
             << ",\"truth_obstacle_count\":" << (box_obstacles_.size()+static_obs_vec.size()+sphere_obs_vec.size()+horizontal_obs_vec.size()+dyn_obs_vec.size())
             << ",\"map_ready\":" << ((!sensor_map_enabled || sensor_map->ready(planning_time.toSec())) ? "true" : "false")
             << ",\"map_stamp\":" << (sensor_map_enabled ? sensor_map->stamp() : 0)
             << ",\"map_age\":" << (sensor_map_enabled ? planning_time.toSec()-sensor_map->stamp() : 0)
             << ",\"sensor_map_current_state_free\":" << ((!sensor_map_enabled || sensor_map->free_point(observed.pos[0],observed.pos[1],observed.h)) ? "true" : "false")
             << ",\"command_published\":" << (executable ? "true" : "false")
             << ",\"execution_rejection_reason\":\"" << execution_rejection_reason << "\""
             << ",\"observation_stamp\":" << observed_stamp.toSec()
             << ",\"planning_stamp\":" << planning_time.toSec()
             << ",\"proposal_state_stamp\":" << proposal_state_stamp.toSec()
             << ",\"planning_lookahead\":" << lookahead
             << ",\"generation_duration\":" << generation_duration
             << ",\"forecast_fallback_to_current\":" << (forecast_fallback ? "true" : "false")
             << ",\"sensor_planning_reserve\":" << sensor_planning_reserve
             << ",\"dispatch_state_stamp\":" << dispatch_state_stamp.toSec()
             << ",\"command_stamp\":" << command_stamp.toSec()
             << ",\"observation_age\":" << (planning_time - observed_stamp).toSec()
             << ",\"dispatch_validation_age\":" << (command_stamp.isZero() ? 0.0 : (command_stamp - dispatch_state_stamp).toSec())
             << ",\"recovery_verified\":" << (selected.recovery_verified ? "true" : "false")
             << ",\"reused_verified_plan\":" << (selected.reused_verified_plan ? "true" : "false")
             << ",\"plan_remaining_time\":" << selected.plan_remaining_time
             << ",\"cached_integration_steps\":" << selected.cached_integration_steps
             << ",\"cached_segments_checked\":" << selected.cached_segments_checked
             << ",\"budget_exhausted\":" << (selected.budget_exhausted ? "true" : "false")
             << ",\"primary_evaluated\":" << selected.primary_evaluated
             << ",\"primary_feasible_count\":" << selected.primary_feasible_count
             << ",\"recovery_checked\":" << selected.recovery_checked
             << ",\"backup_evaluated\":" << selected.backup_evaluated
             << ",\"primary_integration_steps\":" << selected.primary_integration_steps
             << ",\"backup_integration_steps\":" << selected.backup_integration_steps
             << ",\"backup_work_budget_exhausted\":" << (selected.backup_work_budget_exhausted ? "true" : "false")
             << ",\"backup_rejected_collision\":" << selected.backup_rejected_collision
             << ",\"backup_rejected_envelope\":" << selected.backup_rejected_envelope
             << ",\"backup_rejected_known_space\":" << selected.backup_rejected_known_space
             << ",\"backup_rejected_invalid\":" << selected.backup_rejected_invalid
             << ",\"backup_truncated\":" << selected.backup_truncated
             << ",\"backup_early_terminated\":" << selected.backup_early_terminated
             << ",\"selected_cost_rank\":" << selected.selected_cost_rank
             << ",\"scheduled_count\":" << selected.scheduled_count
             << ",\"candidate_order\":\"" << selected.candidate_order << "\"}";
        std_msgs::String status;
        status.data = json.str();
        planning_status_pub.publish(status);
    }

    void updateDynamicObstacles() {
        const ros::Time now = ros::Time::now();
        for (auto& obs : dyn_obs_vec) {
            const float dt = (now - obs.last_update_time).toSec();
            obs.last_update_time = now;
            if (dt <= 0) continue;
            if (obs.type == "linear") {
                obs.current_pos[0] += obs.velocity * std::cos(obs.direction) * dt;
                obs.current_pos[1] += obs.velocity * std::sin(obs.direction) * dt;
            } else if (obs.type == "circular") {
                obs.angle = wrap_yaw(obs.angle + obs.angular_velocity * dt);
                obs.current_pos[0] = obs.center[0] + obs.radius * std::cos(obs.angle);
                obs.current_pos[1] = obs.center[1] + obs.radius * std::sin(obs.angle);
            }
        }
    }

    pr_t compensate_cmd(const pr_t& pr, const s_t& s) {
        const float nu = s.d_sigma * tau_a / 2.0f;
        const float delay_distance = s.V * tau_a;
        const float distance = std::sqrt(std::max(0.01f, pr.wp.d * pr.wp.d +
            delay_distance * delay_distance - 2.0f * pr.wp.d * delay_distance * std::cos(pr.wp.theata - nu)));
        float angle = std::acos(constraint((delay_distance - pr.wp.d * std::cos(pr.wp.theata - nu)) / distance, -1.0f, 1.0f));
        if (pr.wp.theata - nu < 0) angle = -angle;
        wp_t wp = pr.wp;
        wp.d = distance;
        wp.theata = constraint(wrap_yaw(M_PI - angle - nu), -mp_theata, mp_theata);
        pr_t result = generate_unlimited_agility_cmd(wp, s);
        result.wp = pr.wp;
        result.t_way = pr.t_way;
        return result;
    }

    void check_target(const s_t& s) {
        if (std::hypot(s.pos[0] - global_target.pos[0], s.pos[1] - global_target.pos[1]) <= horizontal_tolerance &&
            std::abs(s.h - global_target.h) <= altitude_tolerance) {
            wp_i = (wp_i + 1) % wp_n;
            global_target = wp_vec[wp_i];
        }
    }

    void odom_cb(const nav_msgs::Odometry& msg) {
        s_t s;
        s.V = msg.twist.twist.linear.x;
        s.gamma = msg.twist.twist.linear.y;
        s.sigma = msg.twist.twist.linear.z;
        s.T = msg.twist.twist.angular.x;
        s.d_sigma = msg.twist.twist.angular.y;
        s.pos[0] = msg.pose.pose.position.x;
        s.pos[1] = msg.pose.pose.position.y;
        s.h = msg.pose.pose.position.z;
        const auto& q = msg.pose.pose.orientation;
        s.q[0] = q.w; s.q[1] = q.x; s.q[2] = q.y; s.q[3] = q.z;
        if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
            q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z < 1e-8) return;
        tf::Quaternion orientation(q.x, q.y, q.z, q.w);
        orientation.normalize();
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        s.roll = -roll;
        s.attack = -pitch - s.gamma;
        if (!finite_state(s)) {
            ROS_WARN_THROTTLE(2.0, "Ignoring non-finite flight odometry");
            return;
        }
        if (!state_ptr) state_ptr = new s_t;
        *state_ptr = s;
        state_stamp = msg.header.stamp;
        if (sensor_map_enabled) sensor_map->add_pose(msg);
    }

    float C_D(float attack) const { return CD_k1 * attack * attack + CD_k2; }
    float C_L(float attack) const { return CL_k1 * std::sin(attack) + CL_k2; }
    static float wrap_yaw(float angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
    static float abs_err_angle(float a, float b) { return std::abs(wrap_yaw(a - b)); }
    static float constraint(float x, float low, float high) { return std::max(low, std::min(x, high)); }

    pr_t generate_unlimited_agility_cmd(const wp_t& wp, const s_t& s) {
        pr_t result;
        const float speed = std::max(min_speed, s.V);
        const float distance = std::max(0.1f, wp.d);
        const float gain = 2.0f * speed / distance;
        const float gamma_c = std::atan2(wp.h - s.h, distance);
        const float gamma_dot = gain * (gamma_c - s.gamma);
        const float y = 2.0f * speed * speed * std::sin(wp.theata) * std::pow(std::cos(s.gamma), 2);
        const float x = distance * (gravity * std::cos(s.gamma) + speed * gamma_dot);
        result.w_roll_c = constraint(std::atan2(y, x), -mp_max_roll, mp_max_roll);
        const float lift_scale = k * speed * speed * std::max(0.1f, std::cos(result.w_roll_c));
        const float a = (gamma_dot * speed + gravity * std::cos(gamma_c)) / lift_scale;
        const float b = s.T / lift_scale;
        const float denominator = std::max(1e-5f, CL_k1 + b);
        result.attack_c = constraint(std::asin(constraint((a - CL_k2) / denominator, -1.0f, 1.0f)),
                                     mp_min_attack, mp_max_attack);
        result.T_c = constraint((k * speed * speed * C_D(result.attack_c) + gravity * std::sin(gamma_c)) /
            std::max(0.1f, std::cos(result.attack_c)) + mp_k_T * gain * (wp.V - speed), 0.0f, max_thrust);
        result.t_way = horizon_time;
        result.wp = wp;
        return result;
    }

    bool collision_check(float x, float y, float z, float prediction_time = 0.0f) const {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            z < min_altitude || z > max_altitude) return true;
        if (sensor_map_enabled && !sensor_map->free_point(x,y,z,
                planning_guard_active ? sensor_reserve_at(prediction_time) : 0.0)) return true;
        for (const auto& box : box_obstacles_)
            if (urban_geometry::signed_distance(box.geometry, x, y, z, box.clearance) <= 0.0)
                return true;
        for (const auto& obs : static_obs_vec) {
            if (inside_cylinder(x, y, z, obs.pos[0], obs.pos[1], obs.z, obs.r, obs.height))
                return true;
        }
        for (const auto& obs : sphere_obs_vec) {
            // Inflate in every direction equally; a floating sphere has no
            // cylindrical footprint extending to its top and bottom corners.
            const float radius = obs.r + std::max(0.2f * obs.r, vertical_clearance);
            const float dx = x - obs.x, dy = y - obs.y, dz = z - obs.z;
            if (dx * dx + dy * dy + dz * dz <= radius * radius) return true;
        }
        for (const auto& obs : horizontal_obs_vec) {
            const float dx = x - obs.x, dy = y - obs.y, dz = z - obs.z;
            const float axial = dx * obs.axis_x + dy * obs.axis_y;
            const float transverse = -dx * obs.axis_y + dy * obs.axis_x;
            const float margin = std::max(0.2f * obs.r, vertical_clearance);
            const float radius = obs.r + margin;
            // Transform into the cylinder's frame: finite flat caps along its
            // horizontal axis, circular cross-section in transverse/height.
            if (std::abs(axial) <= obs.length / 2.0f + margin &&
                transverse * transverse + dz * dz <= radius * radius) return true;
        }
        for (const auto& obs : dyn_obs_vec) {
            float ox = obs.current_pos[0], oy = obs.current_pos[1];
            if (prediction_time > 0 && obs.type == "linear") {
                ox += obs.velocity * std::cos(obs.direction) * prediction_time;
                oy += obs.velocity * std::sin(obs.direction) * prediction_time;
            } else if (prediction_time > 0 && obs.type == "circular") {
                const float angle = obs.angle + obs.angular_velocity * prediction_time;
                ox = obs.center[0] + obs.radius * std::cos(angle);
                oy = obs.center[1] + obs.radius * std::sin(angle);
            }
            if (inside_cylinder(x, y, z, ox, oy, obs.z, obs.r, obs.height)) return true;
        }
        return false;
    }

    // Check the entire position segment, rather than only its endpoints. The
    // geometry/margins match collision_check(); the swept path is the runtime
    // integrator's piecewise-linear prediction, not a tracking-error tube.
    bool collision_check_segment(const s_t& from, const s_t& to,
                                 float start_time = 0.0f, float end_time = 0.0f) const {
        using runtime_swept_geometry::Point;
        const Point a{from.pos[0], from.pos[1], from.h}, b{to.pos[0], to.pos[1], to.h};
        if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
            !std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(b.z) ||
            !std::isfinite(start_time) || !std::isfinite(end_time) ||
            start_time < 0 || end_time < start_time ||
            a.z < min_altitude || a.z > max_altitude ||
            b.z < min_altitude || b.z > max_altitude) return true;
        if (sensor_map_enabled) return !sensor_map->free_segment({a.x,a.y,a.z},{b.x,b.y,b.z},
            planning_guard_active ? sensor_reserve_at(end_time) : 0.0);
        for (const auto& entry : box_obstacles_)
            if (runtime_swept_geometry::box(a, b, entry.geometry, entry.clearance)) return true;
        for (const auto& obs : static_obs_vec)
            if (runtime_swept_geometry::cylinder(
                    {a.x - obs.pos[0], a.y - obs.pos[1], a.z - obs.z},
                    {b.x - obs.pos[0], b.y - obs.pos[1], b.z - obs.z},
                    obs.r * 1.2f, obs.height / 2.0f + vertical_clearance)) return true;
        for (const auto& obs : sphere_obs_vec) {
            const float radius = obs.r + std::max(0.2f * obs.r, vertical_clearance);
            if (runtime_swept_geometry::sphere(
                    {a.x - obs.x, a.y - obs.y, a.z - obs.z},
                    {b.x - obs.x, b.y - obs.y, b.z - obs.z}, radius)) return true;
        }
        for (const auto& obs : horizontal_obs_vec) {
            const auto local = [&](const Point& p) {
                const double dx = p.x - obs.x, dy = p.y - obs.y;
                return Point{-dx * obs.axis_y + dy * obs.axis_x, p.z - obs.z,
                             dx * obs.axis_x + dy * obs.axis_y};
            };
            const float margin = std::max(0.2f * obs.r, vertical_clearance);
            if (runtime_swept_geometry::cylinder(local(a), local(b), obs.r + margin,
                                                obs.length / 2.0f + margin)) return true;
        }
        for (const auto& obs : dyn_obs_vec) {
            const auto relative = [&](const Point& p, float time) {
                double x = obs.current_pos[0], y = obs.current_pos[1];
                if (time > 0 && obs.type == "linear") {
                    x += obs.velocity * std::cos(obs.direction) * time;
                    y += obs.velocity * std::sin(obs.direction) * time;
                } else if (time > 0 && obs.type == "circular") {
                    const double angle = obs.angle + obs.angular_velocity * time;
                    x = obs.center[0] + obs.radius * std::cos(angle);
                    y = obs.center[1] + obs.radius * std::sin(angle);
                }
                return Point{p.x - x, p.y - y, p.z - obs.z};
            };
            // Relative motion is linear for a constant-velocity obstacle. For
            // a circular obstacle, enlarge by the arc/chord sagitta over this
            // step; large angular steps use the full diameter bound.
            double extra_radius = 0.0;
            if (obs.type == "circular") {
                const double angle = std::abs(obs.angular_velocity * (end_time - start_time));
                extra_radius = std::abs(obs.radius) * (angle < 3.141592653589793
                    ? 1.0 - std::cos(angle / 2.0) : 2.0);
                // The initial parameter position may not yet lie on its orbit.
                if (start_time == 0.0f)
                    extra_radius += std::hypot(obs.current_pos[0] - obs.center[0] - obs.radius * std::cos(obs.angle),
                                               obs.current_pos[1] - obs.center[1] - obs.radius * std::sin(obs.angle));
            }
            if (runtime_swept_geometry::cylinder(relative(a, start_time), relative(b, end_time),
                    obs.r * 1.2f + extra_radius, obs.height / 2.0f + vertical_clearance)) return true;
        }
        return false;
    }

    // A non-null validation_center enables fail-fast backup checks at each
    // existing integration sample. Primaries pass nullptr and remain unchanged.
    // Early-rejected prefixes are invalid, not complete witnesses or budget cuts.
    rollout_t rollout_motion_primitive(const s_t& initial, const pr_t& pr, float start_time = 0.0f,
                                       int max_integration_steps = -1,
                                       const s_t* validation_center = nullptr) const {
        rollout_t result;
        result.states.push_back(initial);
        s_t s = initial;
        if (!valid_flight_state(s) || !std::isfinite(pr.T_c) || !std::isfinite(pr.attack_c) ||
            !std::isfinite(pr.w_roll_c) || !std::isfinite(pr.t_way) || pr.t_way <= 0 ||
            !std::isfinite(start_time) || start_time < 0 ||
            pr.T_c < 0 || pr.T_c > max_thrust || std::abs(pr.w_roll_c) > mp_max_roll + 1e-5f ||
            pr.attack_c < mp_min_attack - 1e-5f || pr.attack_c > mp_max_attack + 1e-5f) {
            result.valid = false;
            return result;
        }
        const auto reject_sample = [&]() {
            if (!validation_center) return false;
            if (!state_within_recovery_envelope(s)) result.early_rejection_reason = rejection_reason_t::ENVELOPE;
            else if (result.collision) result.early_rejection_reason = rejection_reason_t::COLLISION;
            else if (!state_within_known_space(*validation_center, s)) result.early_rejection_reason = rejection_reason_t::KNOWN_SPACE;
            else return false;
            result.valid = false;
            result.early_rejected = true;
            return true;
        };
        result.collision = collision_check(s.pos[0], s.pos[1], s.h, start_time);
        if (reject_sample()) return result;
        for (float t = 0; t < pr.t_way - 1e-6f;) {
            if (max_integration_steps >= 0 &&
                static_cast<int>(result.states.size()) - 1 >= max_integration_steps) {
                result.valid = false;
                result.work_exhausted = true;
                break;
            }
            const float dt = std::min(integration_dt, pr.t_way - t);
            const s_t previous = s;
            const float segment_start = start_time + t;
            const float speed = std::max(1.0f, s.V);
            const float lift = s.T * std::sin(s.attack) / speed + k * speed * C_L(s.attack);
            const float dV = s.T * std::cos(s.attack) - k * speed * speed * C_D(s.attack) - gravity * std::sin(s.gamma);
            const float dgamma = lift * std::cos(s.roll) - gravity * std::cos(s.gamma) / speed;
            const float dsigma = lift * std::sin(s.roll) / std::max(0.1f, std::cos(s.gamma));
            s.pos[0] += speed * std::cos(s.gamma) * std::cos(s.sigma) * dt;
            s.pos[1] += speed * std::cos(s.gamma) * std::sin(s.sigma) * dt;
            s.h += speed * std::sin(s.gamma) * dt;
            s.V += dV * dt;
            s.gamma += dgamma * dt;
            s.sigma = wrap_yaw(s.sigma + dsigma * dt);
            s.d_sigma = dsigma;
            // Match the three first-order actuators in the existing Gazebo plugin.
            s.attack += (pr.attack_c - s.attack) * dt;
            s.roll += 2.0f * (pr.w_roll_c - s.roll) * dt;
            s.T += 10.0f * (pr.T_c - s.T) * dt;
            t += dt;
            result.states.push_back(s);
            if (!finite_state(s) || s.V < min_speed || s.V > max_speed || std::abs(s.gamma) > 80.0f * Rad2Deg) {
                result.valid = false;
                break;
            }
            if (!result.collision && collision_check_segment(previous, s, segment_start, start_time + t))
                result.collision = true;
            if (reject_sample()) break;
        }
        return result;
    }

    float evaluate_motion_primitive(const s_t& s, const pr_t& pr) const {
        return evaluate_motion_primitive(s, pr, rollout_motion_primitive(s, pr));
    }

    float evaluate_motion_primitive(const s_t&, const pr_t& pr, const rollout_t& prediction) const {
        if (!prediction.valid || prediction.states.empty()) return std::numeric_limits<float>::infinity();
        const s_t& end = prediction.states.back();
        const float ex = global_target.pos[0] - end.pos[0], ey = global_target.pos[1] - end.pos[1];
        const float ez = global_target.h - end.h;
        const float horizontal_distance = std::hypot(ex, ey);
        float cost = distance_weight * std::sqrt(ex*ex + ey*ey + ez*ez) + altitude_weight * std::abs(ez);
        if (horizontal_distance > 1.0f) cost += heading_weight * abs_err_angle(std::atan2(ey, ex), end.sigma);
        if (last_pr_ptr) {
            cost += 0.1f * std::abs(pr.attack_c - last_pr_ptr->attack_c);
            cost += 1.0f * std::abs(pr.w_roll_c - last_pr_ptr->w_roll_c);
            cost += 0.05f * std::abs(pr.T_c - last_pr_ptr->T_c);
        }
        if (prediction.collision) cost += collision_cost;
        return cost;
    }

    visualization_msgs::Marker visual_motion_primitive(const s_t& s, const pr_t& pr) const {
        return visual_motion_primitive(rollout_motion_primitive(s, pr));
    }

    visualization_msgs::Marker visual_motion_primitive(const rollout_t& prediction) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.12;
        marker.color.g = 1;
        marker.color.a = prediction.valid && !prediction.collision ? 0.65 : 0.15;
        for (const auto& s : prediction.states) {
            if (!finite_state(s)) break;
            geometry_msgs::Point p;
            p.x = s.pos[0]; p.y = s.pos[1]; p.z = s.h;
            marker.points.push_back(p);
        }
        return marker;
    }

    visualization_msgs::Marker goal_visual() const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "goal";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = global_target.pos[0];
        marker.pose.position.y = global_target.pos[1];
        marker.pose.position.z = global_target.h;
        marker.pose.orientation.w = 1;
        marker.scale.x = marker.scale.y = marker.scale.z = 5;
        marker.color.g = marker.color.b = marker.color.a = 1;
        return marker;
    }

    visualization_msgs::MarkerArray collision_visual() const {
        visualization_msgs::MarkerArray result;
        int id = 1;
        for (const auto& box : box_obstacles_) {
            const auto& geometry = box.geometry;
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "box_obstacles";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = geometry.x;
            marker.pose.position.y = geometry.y;
            marker.pose.position.z = geometry.z;
            marker.pose.orientation.w = 1;
            marker.scale.x = geometry.size_x;
            marker.scale.y = geometry.size_y;
            marker.scale.z = geometry.size_z;
            marker.color.b = 1;
            marker.color.a = 0.5;
            result.markers.push_back(marker);
        }
        for (size_t i = 0; i < static_obs_vec.size(); ++i) {
            const auto& obs = static_obs_vec[i];
            auto marker = cylinder_marker("static_obs" + std::to_string(i + 1), id++, obs.pos[0], obs.pos[1], obs.z, obs.r, obs.height);
            marker.color.b = 1;
            result.markers.push_back(marker);
        }
        for (size_t i = 0; i < dyn_obs_vec.size(); ++i) {
            const auto& obs = dyn_obs_vec[i];
            auto marker = cylinder_marker("dyn_obs" + std::to_string(i + 1), id++, obs.current_pos[0], obs.current_pos[1], obs.z, obs.r, obs.height);
            marker.color.r = 1;
            result.markers.push_back(marker);
        }
        for (size_t i = 0; i < sphere_obs_vec.size(); ++i) {
            const auto& obs = sphere_obs_vec[i];
            auto marker = cylinder_marker("sphere_obs" + std::to_string(i + 1), id++,
                                          obs.x, obs.y, obs.z, obs.r, 2 * obs.r);
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.color.b = 1.0f;
            result.markers.push_back(marker);
        }
        for (size_t i = 0; i < horizontal_obs_vec.size(); ++i) {
            const auto& obs = horizontal_obs_vec[i];
            auto marker = cylinder_marker("horizontal_obs" + std::to_string(i + 1), id++,
                                          obs.x, obs.y, obs.z, obs.r, obs.length);
            marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, M_PI / 2.0, obs.yaw);
            marker.color.b = 1.0f;
            result.markers.push_back(marker);
        }
        return result;
    }

public: // Isolated experiment access.
    struct FlagScope {
        bool& flag;bool previous;
        FlagScope(bool& value,bool next):flag(value),previous(value){flag=next;}
        ~FlagScope(){flag=previous;}
    };
    bool revalidate_selection(const s_t& initial, selection_t& selection) {
        std::vector<pr_t> commands = selection.execution_plan;
        if (commands.empty()) commands.push_back(selection.command);
        if (strict_execution && commands.front().t_way < min_execution_time) return false;
        s_t state = initial;
        float offset = 0.0f;
        std::vector<rollout_t> checked;
        for (const auto& command : commands) {
            if (!valid_cached_command(command)) return false;
            const auto prediction = rollout_motion_primitive(state, command, offset);
            if (!check_primary_feasibility(initial, prediction)) return false;
            checked.push_back(prediction);
            state = prediction.states.back();
            offset += command.t_way;
        }
        selection.rollout = checked.front();
        selection.backup_rollout = rollout_t();
        for (size_t i = 1; i < checked.size(); ++i) {
            const auto& states = checked[i].states;
            const size_t skip = selection.backup_rollout.states.empty() ? 0 : 1;
            selection.backup_rollout.states.insert(selection.backup_rollout.states.end(), states.begin() + skip, states.end());
        }
        selection.cost = evaluate_motion_primitive(initial, selection.command, selection.rollout);
        selection.plan_remaining_time = selection.execution_plan.empty() ? 0.0f : offset;
        return true;
    }

    void account_execution_until(const ros::Time& now) {
        if (have_command_time) advance_execution(static_cast<float>((now - last_command_time).toSec()));
        last_command_time = now;
        have_command_time = true;
    }

    void load_box_obstacles() {
        XmlRpc::XmlRpcValue config;
        if (!nh.getParam("box_obstacles", config)) return;
        const std::string prefix = nh.resolveName("box_obstacles");
        if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !config.hasMember("num") || config["num"].getType() != XmlRpc::XmlRpcValue::TypeInt)
            throw std::invalid_argument(prefix + "/num must be a nonnegative integer");
        const int count = static_cast<int>(config["num"]);
        if (count < 0)
            throw std::invalid_argument(prefix + "/num must be a nonnegative integer");
        std::vector<box_obstacle_t> boxes;
        for (int i = 0; i < count; ++i) {
            const std::string key = "box" + std::to_string(i + 1);
            if (!config.hasMember(key) || config[key].getType() != XmlRpc::XmlRpcValue::TypeStruct)
                throw std::invalid_argument(prefix + "/" + key + " is missing or invalid");
            auto& fields = config[key];
            const auto number = [&](const std::string& field) {
                if (!fields.hasMember(field))
                    throw std::invalid_argument(prefix + "/" + key + "/" + field + " is missing");
                auto& value = fields[field];
                double result;
                if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
                    result = static_cast<int>(value);
                else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    result = static_cast<double>(value);
                else
                    throw std::invalid_argument(prefix + "/" + key + "/" + field + " must be numeric");
                if (!std::isfinite(result))
                    throw std::invalid_argument(prefix + "/" + key + "/" + field + " must be finite");
                return result;
            };
            box_obstacle_t box;
            box.geometry = {number("x"), number("y"), number("z"),
                            number("size_x"), number("size_y"), number("size_z")};
            if (fields.hasMember("clearance")) box.clearance = number("clearance");
            if (!urban_geometry::valid(box.geometry) || box.clearance < 0.0)
                throw std::invalid_argument(prefix + "/" + key + " requires positive sizes and nonnegative clearance");
            boxes.push_back(box);
        }
        set_box_obstacles(boxes);
    }

    static bool same_control(const pr_t& a, const pr_t& b) {
        return a.w_roll_c == b.w_roll_c && a.attack_c == b.attack_c &&
            a.T_c == b.T_c && a.t_way == b.t_way;
    }
    bool valid_cached_command(const pr_t& command) const {
        return std::isfinite(command.t_way) && command.t_way > 0 && command.t_way <= 10.0f + 1e-5f &&
            std::isfinite(command.w_roll_c) && std::abs(command.w_roll_c) <= mp_max_roll + 1e-5f &&
            std::isfinite(command.attack_c) && command.attack_c >= mp_min_attack - 1e-5f &&
            command.attack_c <= mp_max_attack + 1e-5f && std::isfinite(command.T_c) &&
            command.T_c >= 0 && command.T_c <= max_thrust + 1e-5f;
    }
    bool try_reuse_cached_plan(const s_t& actual, selection_t& result) {
        if (planning_guard_active) return false;
        if (!recovery_enabled || !require_terminal || !reuse_verified_plan) return false;
        const auto reused = revalidate_cached_plan(actual);
        result.cached_integration_steps += reused.cached_integration_steps;
        result.cached_segments_checked += reused.cached_segments_checked;
        if (!reused.reused_verified_plan) return false;
        result.command = reused.command;
        result.backup_command = reused.backup_command;
        result.rollout = reused.rollout;
        result.backup_rollout = reused.backup_rollout;
        result.cost = reused.cost;
        result.primary_feasible = true;
        result.recovery_verified = false;
        result.reused_verified_plan = true;
        result.execution_plan = reused.execution_plan;
        result.plan_remaining_time = reused.plan_remaining_time;
        result.selected_cost_rank = -1;  // Not a member of this cycle's sampled library.
        result.status = "REVALIDATED_PLAN";
        return true;
    }
    void load_angle(const std::string& name, float& value, float degrees) {
        nh.param<float>("motion_primitive/" + name, value, degrees);
        value *= Rad2Deg;
    }
    static bool finite_state(const s_t& s) {
        return std::isfinite(s.V) && std::isfinite(s.gamma) && std::isfinite(s.sigma) &&
            std::isfinite(s.T) && std::isfinite(s.h) && std::isfinite(s.pos[0]) &&
            std::isfinite(s.pos[1]) && std::isfinite(s.roll) && std::isfinite(s.attack) && std::isfinite(s.d_sigma);
    }
    bool valid_flight_state(const s_t& s) const {
        return finite_state(s) && s.V >= min_speed && s.V <= max_speed &&
            s.h >= min_altitude && s.h <= max_altitude && std::abs(s.gamma) <= 80.0f * Rad2Deg &&
            std::abs(s.roll) <= mp_max_roll + 1e-5f &&
            s.attack >= mp_min_attack - 1e-5f && s.attack <= mp_max_attack + 1e-5f &&
            s.T >= 0 && s.T <= max_thrust + 1e-5f;
    }
    bool within_recovery_envelope(const rollout_t& rollout) const {
        // The baseline uses climb/descent angles to sample waypoint heights.
        // Continuation verification additionally enforces those limits on every
        // predicted state, including the current and primary terminal states.
        // Recheck states rather than trusting a caller-supplied valid flag.
        for (const auto& s : rollout.states) {
            if (!state_within_recovery_envelope(s)) return false;
        }
        return !rollout.states.empty();
    }
    bool state_within_known_space(const s_t& observed_start, const s_t& state) const {
        if (sensor_map_enabled && sensor_map->map().state(sensor_occupancy::Point(state.pos[0],state.pos[1],state.h)) != sensor_occupancy::Cell::FREE)
            return false;
        if (!finite_state(state)) return false;
        if (known_space_radius <= 0) return true;
        const float dx = state.pos[0] - observed_start.pos[0];
        const float dy = state.pos[1] - observed_start.pos[1];
        const float dz = state.h - observed_start.h;
        return dx*dx + dy*dy + dz*dz <= known_space_radius * known_space_radius;
    }
    bool within_known_space(const s_t& observed_start, const rollout_t& rollout) const {
        // Idealized spherical observation footprint, frozen at planning start.
        // It is not a camera FOV, occlusion model, or measured free-space map.
        for (const auto& state : rollout.states)
            if (!state_within_known_space(observed_start, state)) return false;
        return !rollout.states.empty();
    }
    bool inside_cylinder(float x, float y, float z, float ox, float oy, float center_z, float radius, float height) const {
        return std::abs(z - center_z) <= height / 2.0f + vertical_clearance &&
            std::hypot(x - ox, y - oy) <= radius * 1.2f;
    }
    visualization_msgs::Marker cylinder_marker(const std::string& name, int id, float x, float y, float z, float radius, float height) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = name;
        marker.id = id;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x; marker.pose.position.y = y; marker.pose.position.z = z;
        marker.pose.orientation.w = 1;
        marker.scale.x = marker.scale.y = 2 * radius;
        marker.scale.z = height;
        marker.color.a = 0.5;
        return marker;
    }

    const float gravity = 9.81f, k = 1.86f, collision_cost = 10000.0f;
    float mp_d, mp_V, mp_theata, mp_max_roll, mp_max_attack, mp_min_attack, mp_k_T;
    float horizon_time, max_climb_angle, max_descent_angle, min_altitude, max_altitude;
    float integration_dt, min_speed, max_speed, max_thrust, vertical_clearance;
    float altitude_weight, distance_weight, heading_weight, altitude_tolerance, horizontal_tolerance;
    bool state_relative_altitudes = true;
    float CL_k1, CL_k2, CD_k1, CD_k2, tau_a;
    bool recovery_enabled, reuse_verified_plan, require_terminal, stop_on_rejection;
    bool strict_execution = false, compensate_latency = false;
    float max_state_age = 0.5f, max_validation_age = 0.02f;
    struct applied_command_t { ros::Time stamp; pr_t command; };
    std::vector<applied_command_t> applied_commands;
    ros::Time state_stamp;
    int max_backup_integration_steps;
    float recovery_horizon, known_space_radius, min_execution_time;
    std::vector<pr_t> cached_execution_plan;
    ros::Time last_command_time;
    bool have_command_time = false;
    int recovery_max_candidates;
    std::string recovery_order_mode;
    float learned_work_power = 1.0f;
    LearnedRecoveryNetwork learned_network;
    int mp_num, vertical_num, wp_i = 0, wp_n, previous_marker_count = 0;
    s_t* state_ptr = nullptr;
    pr_t* last_pr_ptr = nullptr;
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher cmd_pub, traj_visual_pub, obs_visual_pub, goal_visual_pub, planning_status_pub;
    ros::Timer planning_timer;
    target_t global_target;
    bool sensor_map_enabled = false, state_only_features = false, have_sensor_execution = false;
    bool planning_guard_active = false, planning_lookahead_enabled = false;
    double sensor_planning_reserve = 0.0, planning_latency_estimate = 0.0;
    std::unique_ptr<RuntimeSensorMap> sensor_map;
    std::vector<target_t> wp_vec;
    std::vector<obs_t> static_obs_vec;
    std::vector<box_obstacle_t> box_obstacles_;
    std::vector<sphere_obstacle> sphere_obs_vec;
    std::vector<horizontal_obstacle> horizontal_obs_vec;
    std::vector<dyn_obs_t> dyn_obs_vec;
};
#endif
