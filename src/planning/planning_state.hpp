// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

namespace dog_ego_planner
{

enum class PlanningState
{
  kWaitingForData,
  kActive,
  kGoalReached,
  kDegradedStop,
};

inline const char * planningStateName(PlanningState state) noexcept
{
  switch (state) {
    case PlanningState::kWaitingForData:
      return "waiting_for_data";
    case PlanningState::kActive:
      return "active";
    case PlanningState::kGoalReached:
      return "goal_reached";
    case PlanningState::kDegradedStop:
      return "degraded_stop";
  }
  return "unknown";
}

}  // namespace dog_ego_planner
