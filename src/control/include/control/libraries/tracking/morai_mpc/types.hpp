#pragma once

namespace control { namespace morai_mpc {
// Kept separate from the source PID header: no PID controller is imported.
enum class LongitudinalState { kAccel, kCoast, kBrake };
}}  // namespace control::morai_mpc
