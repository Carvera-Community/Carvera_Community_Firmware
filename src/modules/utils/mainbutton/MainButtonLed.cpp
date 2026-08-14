#include "MainButtonLed.h"

namespace {

inline uint32_t read_primask() {
  uint32_t value;
  __asm volatile("mrs %0, primask" : "=r"(value));
  return value;
}

inline uint32_t read_basepri() {
  uint32_t value;
  __asm volatile("mrs %0, basepri" : "=r"(value));
  return value;
}

inline void write_basepri(uint32_t value) { __asm volatile("msr basepri, %0" : : "r"(value) : "memory"); }

#define LED_NOP1 __NOP();
#define LED_NOP2 LED_NOP1 LED_NOP1
#define LED_NOP4 LED_NOP2 LED_NOP2
#define LED_NOP16 LED_NOP4 LED_NOP4 LED_NOP4 LED_NOP4
#define LED_NOP32 LED_NOP16 LED_NOP16
#define LED_NOP64 LED_NOP32 LED_NOP32
#define LED_HIGH_ONE LED_NOP64 LED_NOP16
#define LED_HIGH_ZERO LED_NOP32
#define LED_LOW LED_NOP64 LED_NOP4 LED_NOP2

__attribute__((always_inline)) inline void write_bit(LPC_GPIO_TypeDef* port, uint32_t pin_mask, bool one) {
#if defined(MACHINE_FAMILY_Z1)
  const uint32_t interrupt_mask = read_primask();
  __disable_irq();
#endif
  port->FIOSET = pin_mask;
  if (one) {
    LED_HIGH_ONE
  } else {
    LED_HIGH_ZERO
  }
  port->FIOCLR = pin_mask;
#if defined(MACHINE_FAMILY_Z1)
  if (interrupt_mask == 0U) __enable_irq();
#endif
  LED_LOW
}

__attribute__((always_inline)) inline void write_byte(LPC_GPIO_TypeDef* port, uint32_t pin_mask, uint8_t value) {
  for (uint8_t bit = 0; bit < 8; ++bit) write_bit(port, pin_mask, (value & (0x80U >> bit)) != 0U);
}

}  // namespace

void MainButtonLed::set_all(Color color) const {
  Colors colors;
  colors.fill(color);
  write(colors, false);
}

void MainButtonLed::set_number(Color front, Color back, uint8_t number, bool row) const {
  const uint8_t maximum = row ? 5 : 3;
  if (number == 0 || number > maximum) return;

  Colors colors;
  colors.fill(back);
  for (uint8_t index = 0; index < colors.size(); ++index) {
    const bool selected = row ? index < number : index % 2 == 0 && index / 2 < number;
    if (selected) colors[index] = front;
  }
  write(colors, true);
}

void MainButtonLed::write(const Colors& colors, bool exclusive) const {
  if (port_ == nullptr || pin_mask_ == 0) return;
#if defined(MACHINE_FAMILY_Z1)
  (void)exclusive;
  constexpr uint32_t frame_priority_mask = 3U << (8U - __NVIC_PRIO_BITS);
  const uint32_t previous_priority_mask = read_basepri();
  if (previous_priority_mask == 0U || previous_priority_mask > frame_priority_mask) write_basepri(frame_priority_mask);

  constexpr uint8_t group_order[] = {0, 1, 2, 3, 4, 1};
  for (uint8_t group : group_order) {
    for (uint8_t led = 0; led < 3; ++led) {
      write_byte(port_, pin_mask_, colors[group].green);
      write_byte(port_, pin_mask_, colors[group].red);
      write_byte(port_, pin_mask_, colors[group].blue);
    }
  }

  write_basepri(previous_priority_mask);
#else
  if (exclusive) {
    __disable_irq();
  } else {
    NVIC_DisableIRQ(TIMER0_IRQn);
    NVIC_DisableIRQ(TIMER1_IRQn);
  }

  for (const Color& color : colors) {
    write_byte(port_, pin_mask_, color.red);
    write_byte(port_, pin_mask_, color.green);
    write_byte(port_, pin_mask_, color.blue);
  }

  if (exclusive) {
    __enable_irq();
  } else {
    NVIC_EnableIRQ(TIMER0_IRQn);
    NVIC_EnableIRQ(TIMER1_IRQn);
  }
#endif
}
void MainButtonLed::set_pin(const Pin& pin) {
  port_ = pin.port;
  pin_mask_ = pin.port != nullptr && pin.pin < 32 ? 1UL << pin.pin : 0;
}
