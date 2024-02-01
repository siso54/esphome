#include "waveshare_epaper.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace waveshare_epaper {

static const char *const TAG = "waveshare_4.20v2";

static const uint8_t FULL_LUT[] = {
    0x32,  // CMD
    0x01,	0x0A,	0x1B,	0x0F,	0x03,	0x01,
    0x01, 0x05,	0x0A,	0x01,	0x0A,	0x01,
    0x01,	0x01, 0x05,	0x08,	0x03,	0x02,
    0x04,	0x01,	0x01, 0x01,	0x04,	0x04,
    0x02,	0x00,	0x01,	0x01, 0x01,	0x00,	0x00,	0x00,	0x00,	0x01,	0x01,
0x01,	0x00,	0x00,	0x00,	0x00,	0x01,	0x01,
0x01,	0x0A,	0x1B,	0x0F,	0x03,	0x01,	0x01,
0x05,	0x4A,	0x01,	0x8A,	0x01,	0x01,	0x01,
0x05,	0x48,	0x03,	0x82,	0x84,	0x01,	0x01,
0x01,	0x84,	0x84,	0x82,	0x00,	0x01,	0x01,
0x01,	0x00,	0x00,	0x00,	0x00,	0x01,	0x01,
0x01,	0x00,	0x00,	0x00,	0x00,	0x01,	0x01,
0x01,	0x0A,	0x1B,	0x8F,	0x03,	0x01,	0x01,
0x05,	0x4A,	0x01,	0x8A,	0x01,	0x01,	0x01,
0x05,	0x48,	0x83,	0x82,	0x04,	0x01,	0x01,
0x01,	0x04,	0x04,	0x02,	0x00,	0x01,	0x01,
0x01,	0x00,	0x00,	0x00,	0x00,	0x01,	0x01,
0x01,	0x00,	0x00,	0x00,	0x00,	0x01,	0x01,
0x01,	0x8A,	0x1B,	0x8F,	0x03,	0x01,	0x01,
0x05,	0x4A,	0x01,	0x8A,	0x01,	0x01,	0x01,
0x05,	0x48,	0x83,	0x02,	0x04,	0x01,	0x01,
0x01,	0x04,	0x04,	0x02,	0x00,	0x01,	0x01,
0x01,	0x00,	0x00,	0x00,	0x00,	0x01,	0x01,
0x01,	0x00,	0x00,	0x00,	0x00,	0x01,	0x01,
0x01,	0x8A,	0x9B,	0x8F,	0x03,	0x01,	0x01,
0x05,	0x4A,	0x01,	0x8A,	0x01,	0x01,	0x01,
0x05,	0x48,	0x03,	0x42,	0x04,	0x01,	0x01,
0x01,	0x04,	0x04,	0x42,	0x00,	0x01,	0x01,
0x01,	0x00,	0x00,	0x00,	0x00,	0x01,	0x01,
0x01,	0x00,	0x00,	0x00,	0x00,	0x01,	0x01,
0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
0x02,	0x00,	0x00
};

// 	0x07,	0x17,	0x41,	0xA8, 0x32,	0x30,

static const uint8_t SW_RESET = 0x12;  // yes 420
static const uint8_t ACTIVATE = 0x20;  // yes 420
static const uint8_t WRITE_BUFFER = 0x24; // yes 420
static const uint8_t WRITE_BASE = 0x26;

static const uint8_t DRV_OUT_CTL[] = {0x21, 0x00, 0x00};  // DISPLAY update control
static const uint8_t GATEV[] = {0x03, 0x17};
static const uint8_t SRCV[] = {0x04, 0x41, 0xA8, 0x32};
static const uint8_t SLEEP[] = {0x10, 0x01};
static const uint8_t DATA_ENTRY[] = {0x11, 0x03};            // data entry mode
static const uint8_t TEMP_SENS[] = {0x18, 0x80};             // Temp sensor
static const uint8_t DISPLAY_UPDATE[] = {0x21, 0x00, 0x80};  // Display update control
static const uint8_t UPSEQ[] = {0x22, 0xC0};
static const uint8_t ON_FULL[] = {0x22, 0xCF};    // yes 420
static const uint8_t ON_PARTIAL[] = {0x22, 0xFF};
static const uint8_t VCOM[] = {0x2C, 0x30};
static const uint8_t CMD5[] = {0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00};
static const uint8_t BORDER_PART[] = {0x3C, 0x80};  // border waveform
static const uint8_t BORDER_FULL[] = {0x3C, 0x05};  // border waveform
static const uint8_t CMD1[] = {0x3F, 0x07};
static const uint8_t RAM_X_START[] = {0x44, (0x00>>3) & 0xFF, (399>>3) & 0xFF};           // set ram_x_address_start_end
static const uint8_t RAM_Y_START[] = {0x45, 0x00 & 0xFF, ((0x00 >> 8) & 0xFF), 299 & 0xFF , ((299 >> 8) & 0xFF)};  // set ram_y_address_start_end
static const uint8_t RAM_X_POS[] = {0x4E, 0x00};                      // set ram_x_address_counter
// static const uint8_t RAM_Y_POS[] = {0x4F, 0x00, 0x00};        // set ram_y_address_counter
#define SEND(x) this->cmd_data(x, sizeof(x))

void WaveshareEPaper4P2InV2::write_lut_(const uint8_t *lut) {
  this->wait_until_idle_();
  // this->cmd_data(lut, sizeof(FULL_LUT));
  SEND(lut);
  this->command(0x3F);
  this->data(0x07);

  this->command(0x03);
  this->data(0x17);

  this->command(0x04);
  this->data(0x41);
  this->data(0xA8);
  this->data(0x32);

  this->command(0x2c);
  this->data(0x30);
  SEND(CMD1);
  SEND(GATEV);
  SEND(SRCV);
  SEND(VCOM);
}

// write the buffer starting on line top, up to line bottom.
void WaveshareEPaper4P2InV2::write_buffer_(uint8_t cmd, int top, int bottom) {
  this->wait_until_idle_();
  this->set_window_(top, bottom);
  this->command(cmd);
  this->start_data_();
  auto width_bytes = this->get_width_internal() / 8;
  this->write_array(this->buffer_ + top * width_bytes, (bottom - top) * width_bytes);
  this->end_data_();
}

void WaveshareEPaper4P2InV2::send_reset_() {
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->digital_write(false);
    delay(2);
    this->reset_pin_->digital_write(true);
  }
}

void WaveshareEPaper4P2InV2::setup() {
  ESP_LOGI(TAG, "Setting up pins.");
  setup_pins_();
  delay(20);
  ESP_LOGI(TAG, "HW reset.");
  this->reset_pin_->digital_write(true);
  delay(100);
  this->reset_pin_->digital_write(false);
  delay(2);
  this->reset_pin_->digital_write(true);
  delay(100);  // NOLINT
  this->wait_until_idle_();
  ESP_LOGI(TAG, "SW reset.");
  // SW reset
  this->command(0x12);
  this->wait_until_idle_();
  ESP_LOGI(TAG, "Setting up.");
  SEND(DRV_OUT_CTL);
  SEND(BORDER_FULL);
  this->command(0x1A);
  this->data(0x5A);
  this->command(0x22);
  this->data(0x91);
  this->command(0x20);
  this->wait_until_idle_();
  SEND(DATA_ENTRY);
  this->set_window_(0, this->get_height_internal());
  this->wait_until_idle_();
  ESP_LOGI(TAG, "Setup complete.");
}

// t and b are y positions, i.e. line numbers.
void WaveshareEPaper4P2InV2::set_window_(int t, int b) {
  uint8_t buffer[3];
  SEND(RAM_X_START);
  SEND(RAM_Y_START);
  SEND(RAM_X_POS);
  buffer[0] = 0x4F;
  buffer[1] = (uint8_t) 0 & 0xFF;
  buffer[2] = (uint8_t) (0 >> 8) & 0xFF;
  SEND(buffer);
}

// must implement, but we override setup to have more control
void WaveshareEPaper4P2InV2::initialize() {
 ESP_LOGI(TAG, "Setting up pins.");
  setup_pins_();
  delay(20);
  ESP_LOGI(TAG, "HW reset.");
  this->reset_pin_->digital_write(true);
  delay(100);
  this->reset_pin_->digital_write(false);
  delay(2);
  this->reset_pin_->digital_write(true);
  delay(100);  // NOLINT
  this->wait_until_idle_();
  ESP_LOGI(TAG, "SW reset.");
  // SW reset
  this->command(0x12);
  this->wait_until_idle_();
  ESP_LOGI(TAG, "Setting up.");
  SEND(DRV_OUT_CTL);
  SEND(BORDER_FULL);
  SEND(DATA_ENTRY);
  this->set_window_(0, this->get_height_internal());
  this->wait_until_idle_();
  // this->write_lut_(FULL_LUT);
  ESP_LOGI(TAG, "Setup complete.");
}

void WaveshareEPaper4P2InV2::partial_update_() {
  ESP_LOGI(TAG, "Performing partial e-paper update.");
  this->set_timeout(100, [this] {
    // this->write_lut_(FULL_LUT);
    SEND(BORDER_PART);
    SEND(UPSEQ);
    this->command(ACTIVATE);
    this->set_timeout(100, [this] {
      this->wait_until_idle_();
      this->write_buffer_(WRITE_BUFFER, 0, this->get_height_internal());
      SEND(ON_PARTIAL);
      this->command(ACTIVATE);  // don't wait here
      this->is_busy_ = false;
    });
  });
}

void WaveshareEPaper4P2InV2::full_update_() {
  ESP_LOGI(TAG, "Performing full e-paper update.");
  // this->write_lut_(FULL_LUT);
  this->write_buffer_(WRITE_BUFFER, 0, this->get_height_internal());
  SEND(ON_FULL);
  this->command(ACTIVATE);  // don't wait here
  this->is_busy_ = false;
}

void WaveshareEPaper4P2InV2::display() {
  if (this->is_busy_ || (this->busy_pin_ != nullptr && this->busy_pin_->digital_read()))
    return;
  this->is_busy_ = true;
  const bool partial = this->at_update_ != 0;
  this->at_update_ = (this->at_update_ + 1) % this->full_update_every_;
  if (partial) {
    // this->full_update_();
    this->partial_update_();
  } else {
    this->full_update_();
  }
}

int WaveshareEPaper4P2InV2::get_width_internal() { return 400; }

int WaveshareEPaper4P2InV2::get_height_internal() { return 300; }

uint32_t WaveshareEPaper4P2InV2::idle_timeout_() { return 5000; }

void WaveshareEPaper4P2InV2::dump_config() {
  LOG_DISPLAY("", "Waveshare E-Paper", this)
  ESP_LOGCONFIG(TAG, "  Model: 4.20inV2");
  LOG_PIN("  CS Pin: ", this->cs_)
  LOG_PIN("  Reset Pin: ", this->reset_pin_)
  LOG_PIN("  DC Pin: ", this->dc_pin_)
  LOG_PIN("  Busy Pin: ", this->busy_pin_)
  LOG_UPDATE_INTERVAL(this)
}

void WaveshareEPaper4P2InV2::set_full_update_every(uint32_t full_update_every) {
  this->full_update_every_ = full_update_every;
}

}  // namespace waveshare_epaper
}  // namespace esphome
