#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace cat9554 {

/// Modes for CAT9554 pins
//enum CAT9554GPIOMode : uint8_t {
//  CAT9554_INPUT = FLAG_INPUT,
//  CAT9554_OUTPUT = FLAG_OUTPUT,
//};

enum CAT9554Commands {
  INPUT_REG = 0x00,
  OUTPUT_REG = 0x01,
  CONFIG_REG = 0x03,
};

class CAT9554Component : public Component, public i2c::I2CDevice {
 public:
  CAT9554Component() { enable_irq_ = false; };

  /// Check i2c availability and setup masks
  void setup() override;
  /// Helper function to read the value of a pin.
  bool digital_read(uint8_t pin);
  /// Helper function to write the value of a pin.
  void digital_write(uint8_t pin, bool value);
  /// Helper function to set the pin mode of a pin.
  void pin_mode(uint8_t pin, gpio::Flags flags);
  /// Setup irq pin.
  void set_irq_pin(GPIOPin *irq_pin) {
    enable_irq_ = true;
    irq_pin_ = irq_pin;
  };

  float get_setup_priority() const override;

  void dump_config() override;

 protected:
  bool read_gpio_();

  bool write_gpio_();
  bool config_gpio_();
  bool read_config_();

  /// Mask for the pin mode - 1 means output, 0 means input
  uint16_t mode_mask_{0x00};
  /// The mask to write as output state - 1 means HIGH, 0 means LOW
  uint16_t output_mask_{0x00};
  /// The state read in read_gpio_ - 1 means HIGH, 0 means LOW
  uint16_t input_mask_{0x00};
  /// IRQ is enabled.
  bool enable_irq_;
  /// IRQ pin.
  GPIOPin *irq_pin_;
  /// Interrupt handler
  ISRInternalGPIOPin *isr_;
  /// Need update GPIO
  bool update_gpio_;
};

/// Helper class to expose a CAT9554 pin as an internal input GPIO pin.
class CAT9554GPIOPin : public GPIOPin {
 public:
  //CAT9554GPIOPin(CAT9554Component *parent, uint8_t pin, uint8_t mode, bool inverted = false);

  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  std::string dump_summary() const override;

  void set_parent(CAT9554Component *parent) { parent_ = parent; }
  void set_pin(uint8_t pin) { pin_ = pin; }
  void set_inverted(bool inverted) { inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { flags_ = flags; }

 protected:
  CAT9554Component *parent_;
  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
};

}  // namespace cat9554
}  // namespace esphome