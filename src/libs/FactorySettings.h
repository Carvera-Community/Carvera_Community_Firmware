#pragma once

#include <cstdint>
#include <string_view>

struct FACTORY_SET {
  char MachineModel;
  char FuncSetting;
  char reserve1;
  char reserve2;
};

static_assert(sizeof(FACTORY_SET) == 4, "Unexpected factory settings record size");

class FactorySettings {
 public:
  enum class LineResult : uint8_t { ignored, applied, missing_pair, missing_value };

  explicit FactorySettings(FACTORY_SET& values) : values_(values) {}

  LineResult apply_line(std::string_view line);

 private:
  void set_bit(uint8_t bit, bool enabled);

  FACTORY_SET& values_;
};
