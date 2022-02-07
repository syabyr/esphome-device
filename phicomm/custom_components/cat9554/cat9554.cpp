#include "cat9554.h"
#include "esphome/core/log.h"

namespace esphome {
namespace cat9554 {

static const char *const TAG = "cat9554";

//static void ICACHE_RAM_ATTR HOT gpio_intr(bool *need_update_gpio) { *need_update_gpio = true; }

void CAT9554Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up CAT9554...");
  if (!this->read_gpio_()) {
    ESP_LOGE(TAG, "CAT9554 not available under 0x%02X", this->address_);
    this->mark_failed();
    return;
  }

  if (this->enable_irq_) {
    this->irq_pin_->setup();
	ESP_LOGCONFIG(TAG, "Eable CAT9554 interrupt");
    //this->isr_ = this->irq_pin_->to_isr();
    //this->irq_pin_->attach_interrupt(gpio_intr, &this->update_gpio_, FALLING);
    this->update_gpio_ = false;
  }
  this->read_gpio_();
  this->read_config_();
}
void CAT9554Component::dump_config() {
  ESP_LOGCONFIG(TAG, "CAT9554:");
  LOG_I2C_DEVICE(this)
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with CAT9554 failed!");
  }
}
bool CAT9554Component::digital_read(uint8_t pin) {
  if (!this->enable_irq_ || this->update_gpio_) {
    this->read_gpio_();
    this->update_gpio_ = false;
  }
  return this->input_mask_ & (1 << pin);
}
void CAT9554Component::digital_write(uint8_t pin, bool value) {
  if (value) {
    this->output_mask_ |= (1 << pin);
  } else {
    this->output_mask_ &= ~(1 << pin);
  }

  this->write_gpio_();
}
void CAT9554Component::pin_mode(uint8_t pin, gpio::Flags flags) {
  switch (flags) {
    case gpio::FLAG_INPUT:
      // Clear mode mask bit
      this->mode_mask_ |= (1 << pin);
      break;
    case gpio::FLAG_OUTPUT:
      // Set mode mask bit
      this->mode_mask_ &= ~(1 << pin);
      break;
    default:
      break;
  }
  this->config_gpio_();
}
bool CAT9554Component::read_gpio_() {
  if (this->is_failed())
    return false;
  bool success;
  uint8_t data;
  success = this->read_byte(INPUT_REG, &data);
  if (!success) {
    this->status_set_warning();
    return false;
  }
  this->input_mask_ = data;

  this->status_clear_warning();
  return true;
}
bool CAT9554Component::write_gpio_() {
  if (this->is_failed())
    return false;

  if (!this->write_byte(OUTPUT_REG, this->output_mask_)) {
    this->status_set_warning();
    return false;
  }

  this->status_clear_warning();
  return true;
}
bool CAT9554Component::config_gpio_() {
  if (this->is_failed())
    return false;

  if (!this->write_byte(INPUT_REG, this->mode_mask_)) {
    this->status_set_warning();
    return false;
  }
  if (!this->write_byte(CONFIG_REG, this->mode_mask_)) {
    this->status_set_warning();
    return false;
  }
  if (!this->write_byte(INPUT_REG, 0x00)) {
    this->status_set_warning();
    return false;
  }

  this->status_clear_warning();
  return true;
}
bool CAT9554Component::read_config_() {
  uint8_t data;

  if (this->is_failed())
    return false;

  if (!this->read_byte(CONFIG_REG, &data)) {
    this->status_set_warning();
    return false;
  }
  this->mode_mask_ = data;

  this->status_clear_warning();
  return true;
}
float CAT9554Component::get_setup_priority() const { return setup_priority::IO; }

void CAT9554GPIOPin::setup() { pin_mode(flags_); }
void CAT9554GPIOPin::pin_mode(gpio::Flags flags) { this->parent_->pin_mode(this->pin_, flags); }
bool CAT9554GPIOPin::digital_read() { return this->parent_->digital_read(this->pin_) != this->inverted_; }
void CAT9554GPIOPin::digital_write(bool value) { this->parent_->digital_write(this->pin_, value != this->inverted_); }
std::string CAT9554GPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via PCF8574", pin_);
  return buffer;
}

}  // namespace cat9554
}  // namespace esphome