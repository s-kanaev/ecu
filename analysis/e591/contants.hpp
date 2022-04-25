#pragma once

#include <types.hpp>

namespace Const {
// Synchronization disc teeth numbers start from 0
static constexpr byte SyncDiscTeethCount = 60;
static constexpr byte SyncDiscLastTooth = SyncDiscTeethCount - 1; // 59
static constexpr byte SyncDiscMissingTeethCount = 2;
static constexpr byte SyncDiscLastRealTooth =
    SyncDiscTeethCount - SyncDiscMissingTeethCount - 1; // 57
static constexpr byte SyncDiscLastRealToothMinus1 =
    SyncDiscLastRealTooth - 1; // 56
} // namespace Const
