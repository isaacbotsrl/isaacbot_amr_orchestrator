// Copyright 2025 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ISAACBOT_AMR_MISSION_MANAGER_DOMAIN_MISSION_POLICY_HPP_
#define ISAACBOT_AMR_MISSION_MANAGER_DOMAIN_MISSION_POLICY_HPP_

namespace isaacbot_amr_mission_manager::domain
{

/**
 * @brief Health of a sensor the mission depends on.
 *
 * kUnknown is a third state on purpose: "no lidar fitted" and "lidar fitted but broken" must
 * lead to different decisions, and a bool cannot tell them apart.
 */
enum class SensorHealth
{
    kUnknown,    ///< Never reported. No sensor fitted, or it has not started yet.
    kHealthy,    ///< Reporting usable data.
    kUnhealthy,  ///< Reporting a fault, or its report has gone stale.
};

/** @brief Everything the policy is allowed to look at when deciding what to do next. */
struct IsaacbotConditions
{
    /**
     * @brief True when the isaacbot's driving mode lets Nav 2 drive (isaacbot_amr_drive_mode AUTOMATIC).
     *
     * False until the mode is known: the manager must never start driving on a isaacbot whose
     * operator has not handed it over.
     */
    bool autonomy_allowed = false;

    /** @brief True when isaacbot_amr_twist_mux's motion lock is engaged, or its topic is stale. */
    bool motion_locked = true;

    /** @brief Battery state of charge, 0..1. Negative means "not known yet". */
    double battery_fraction = -1.0;

    /** @brief Health of isaacbot_amr_rs16_lidar, from its diagnostic_updater task. */
    SensorHealth lidar_health = SensorHealth::kUnknown;
};

/** @brief What the manager should do with the active mission this tick. */
enum class MissionAction
{
    kProceed,  ///< Keep driving to the current waypoint.
    kHold,     ///< Suspend; conditions are temporarily unsafe.
    kCancel,   ///< The operator took the isaacbot back (driving mode left AUTOMATIC).
    kAbort,    ///< Give up on the mission.
};

/**
 * @brief Decides whether a mission may continue, given the isaacbot's current conditions.
 *
 * Kept separate from Mission so the "when is it safe to drive" rules can be unit-tested and
 * changed without touching the mission's own bookkeeping.
 *
 * The motion lock is a *hold*, not an abort: it is how an operator pauses the isaacbot, and the
 * mission must survive being paused. A battery below the abort threshold is an abort, since
 * no amount of waiting fixes it while driving.
 *
 * A dead lidar is also a *hold*, for the same reason as the lock: Nav 2's costmaps stop being
 * trustworthy without it, but the sensor can come back, and the mission should resume on the
 * same waypoint when it does. kUnknown only holds when @p require_lidar is set, so a isaacbot
 * running with ISAACBOT_AMR_SYSTEM_USE_LIDAR=false is not permanently held.
 *
 * Leaving AUTOMATIC is a *cancel*: the operator switched modes or took over with the
 * joystick, and the isaacbot must not resume the mission on its own when AUTOMATIC comes back.
 */
class MissionPolicy
{
public:
    explicit MissionPolicy(double abort_battery_fraction = 0.10, bool require_lidar = false);

    MissionAction decide(const IsaacbotConditions & conditions) const;

    /** @brief Whether a new mission may start under @p conditions. */
    bool mayAccept(const IsaacbotConditions & conditions) const;

    double abortBatteryFraction() const { return abort_battery_fraction_; }

    bool requireLidar() const { return require_lidar_; }

private:
    double abort_battery_fraction_;
    bool require_lidar_;
};

}  // namespace isaacbot_amr_mission_manager::domain

#endif  // ISAACBOT_AMR_MISSION_MANAGER_DOMAIN_MISSION_POLICY_HPP_
