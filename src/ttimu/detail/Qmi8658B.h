// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cstdint>

namespace ttimu {
class RegisterBus;
struct RawSample;
namespace detail {
enum class QmiReadResult : uint8_t { NoData, Sample, BusError };
bool qmi8658bConfigure(RegisterBus& bus,uint8_t odr);
QmiReadResult qmi8658bRead(RegisterBus& bus,RawSample& sample);
}}
