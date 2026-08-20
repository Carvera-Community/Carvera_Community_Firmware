#pragma once

#include <cstdint>

enum class Machine : uint8_t {
  unknown = 0,
  carvera = 1,
  carvera_air = 2,
  makera_z1 = 3,
  makera_z1_pro = 4,
};

inline constexpr Machine CARVERA = Machine::carvera;
inline constexpr Machine CARVERA_AIR = Machine::carvera_air;
inline constexpr Machine Z1 = Machine::makera_z1;
inline constexpr Machine Z1PRO = Machine::makera_z1_pro;
