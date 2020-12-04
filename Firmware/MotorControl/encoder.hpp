#ifndef __ENCODER_HPP
#define __ENCODER_HPP

#include <arm_math.h>
#include <Drivers/STM32/stm32_spi_arbiter.hpp>
#include "utils.hpp"
#include <autogen/interfaces.hpp>
#include "component.hpp"


class Encoder : public ODriveIntf::EncoderIntf {
public:
    static constexpr uint32_t MODE_FLAG_ABS = 0x100;

    struct Config_t {
        Mode mode = MODE_INCREMENTAL;
        bool use_index = false;
        bool pre_calibrated = false; // If true, this means the offset stored in
                                    // configuration is valid and does not need
                                    // be determined by run_offset_calibration.
                                    // In this case the encoder will enter ready
                                    // state as soon as the index is found.
        bool zero_count_on_find_idx = true;
        int32_t cpr = (2048 * 4);   // Default resolution of CUI-AMT102 encoder,
        int32_t phase_offset = 0;        // Offset between encoder count and rotor electrical phase
        float phase_offset_float = 0.0f; // Sub-count phase alignment offset
        int32_t direction = 0.0f; // direction with respect to motor
        bool enable_phase_interpolation = true; // Use velocity to interpolate inside the count state
        float calib_range = 0.02f; // Accuracy required to pass encoder cpr check
        float calib_scan_distance = 16.0f * M_PI; // rad electrical
        float calib_scan_omega = 4.0f * M_PI; // rad/s electrical
        float bandwidth = 1000.0f;
        bool find_idx_on_lockin_only = false; // Only be sensitive during lockin scan constant vel state
        bool ignore_illegal_hall_state = false; // dont error on bad states like 000 or 111
        uint16_t abs_spi_cs_gpio_pin = 1;
        uint16_t sincos_gpio_pin_sin = 3;
        uint16_t sincos_gpio_pin_cos = 4;

        // custom setters
        Encoder* parent = nullptr;
        void set_use_index(bool value) { use_index = value; parent->set_idx_subscribe(); }
        void set_find_idx_on_lockin_only(bool value) { find_idx_on_lockin_only = value; parent->set_idx_subscribe(); }
        void set_abs_spi_cs_gpio_pin(uint16_t value) { abs_spi_cs_gpio_pin = value; parent->abs_spi_cs_pin_init(); }
        void set_pre_calibrated(bool value) { pre_calibrated = value; parent->check_pre_calibrated(); }
        void set_bandwidth(float value) { bandwidth = value; parent->update_pll_gains(); }
    };

    Encoder(TIM_HandleTypeDef* timer, Stm32Gpio index_gpio,
            Stm32Gpio hallA_gpio, Stm32Gpio hallB_gpio, Stm32Gpio hallC_gpio,
            Stm32SpiArbiter* spi_arbiter);
    
    bool apply_config(ODriveIntf::MotorIntf::MotorType motor_type);
    void setup();
    void set_error(Error error);
    bool do_checks();

    void enc_index_cb();
    void set_idx_subscribe(bool override_enable = false);
    void update_pll_gains();
    void check_pre_calibrated();

    void set_linear_count(int32_t count);
    void set_circular_count(int32_t count, bool update_offset);
    bool calib_enc_offset(float voltage_magnitude);

    bool run_index_search();
    bool run_direction_find();
    bool run_offset_calibration();
    void sample_now();
    bool read_sampled_gpio(Stm32Gpio gpio);
    void decode_hall_samples();
    bool update();

    TIM_HandleTypeDef* timer_;
    Stm32Gpio index_gpio_;
    Stm32Gpio hallA_gpio_;
    Stm32Gpio hallB_gpio_;
    Stm32Gpio hallC_gpio_;
    Stm32SpiArbiter* spi_arbiter_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Config_t config_;

    Error error_ = ERROR_NONE;
    bool index_found_ = false;
    bool is_ready_ = false;
    int32_t shadow_count_ = 0;
    int32_t count_in_cpr_ = 0;
    float interpolation_ = 0.0f;
    OutputPort<float> phase_ = 0.0f;     // [rad]
    OutputPort<float> phase_vel_ = 0.0f; // [rad/s]
    float pos_estimate_counts_ = 0.0f;  // [count]
    float pos_cpr_counts_ = 0.0f;  // [count]
    float vel_estimate_counts_ = 0.0f;  // [count/s]
    float pll_kp_ = 0.0f;   // [count/s / count]
    float pll_ki_ = 0.0f;   // [(count/s^2) / count]
    float calib_scan_response_ = 0.0f; // debug report from offset calib
    int32_t pos_abs_ = 0;
    float spi_error_rate_ = 0.0f;

    OutputPort<float> pos_estimate_ = 0.0f; // [turn]
    OutputPort<float> vel_estimate_ = 0.0f; // [turn/s]
    OutputPort<float> pos_circular_ = 0.0f; // [turn]
    OutputPort<float> pos_cpr_ = 0.0f;      // [turn]

    bool pos_estimate_valid_ = false;
    bool vel_estimate_valid_ = false;

    int16_t tim_cnt_sample_ = 0; // 
    static const constexpr GPIO_TypeDef* ports_to_sample[] = { GPIOA, GPIOB, GPIOC };
    uint16_t port_samples_[sizeof(ports_to_sample) / sizeof(ports_to_sample[0])];
    // Updated by low_level pwm_adc_cb
    uint8_t hall_state_ = 0x0; // bit[0] = HallA, .., bit[2] = HallC
    float sincos_sample_s_ = 0.0f;
    float sincos_sample_c_ = 0.0f;

    bool abs_spi_start_transaction();
    void abs_spi_cb(bool success);
    void abs_spi_cs_pin_init();
    bool abs_spi_pos_updated_ = false;
    Mode mode_ = MODE_INCREMENTAL;
    Stm32Gpio abs_spi_cs_gpio_;
    uint32_t abs_spi_cr1;
    uint32_t abs_spi_cr2;
    uint16_t abs_spi_tle_dma_tx_[1] = {0x8021};
    uint16_t abs_spi_dma_tx_[1] = {0xFFFF};
    uint16_t abs_spi_dma_rx_[4] = {0xFFFF};
    Stm32SpiArbiter::SpiTask spi_task_;

    // specific to TLE encoder
    Stm32SpiArbiter::SpiTask tle_read_task_;
    Stm32SpiArbiter::SpiTask tle_write_task_;
    uint16_t tle_dma_read_tx_[1] = {0x0000};
    uint16_t tle_dma_write_tx_[2] = {0x0000, 0x0000};
    uint16_t tle_dma_rx_[10] = {0xFFFF};
    bool tle_spi_read(uint16_t command, uint16_t length);                    // send command
    bool tle_spi_write(uint16_t command, uint16_t data);    // send command and data
    void tle_spi_read_cb(bool success);                     // callback to release task
    void tle_spi_write_cb(bool success);
    uint32_t tle_spi_get_rx();                              // returns value of tle_dma_rx_
    uint8_t tle_crc(uint8_t* message, size_t length, uint8_t* tle_crc_table);
    bool tle_safety(uint16_t command, uint16_t data, uint16_t safety_word);
    // crc table from TLE5012B datasheet
    uint8_t tle_crc_table[256] = {
        //The “crc” of the position [1] (result from operation [crc ^*(message+Byteidx)])
        //is 0x00 -> 0x00 XOR 0x11D = 0x00 (1 byte).
        0x00,
        //The “crc” of the position [2] is 0x1D -> 0x01 XOR 0x11D = 0x1D (1 byte).
        0x1D,
        //The “crc” of the position [3] is 0x3A -> 0x02 XOR 0x11D = 0x3A (1 byte).
        0x3A,
        //For all the rest of the cases.
        0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB, 0xCD,
        0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B,
        0x76, 0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B,
        0x06, 0x21, 0x3C, 0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98,
        0x85, 0xD6, 0xCB, 0xEC, 0xF1, 0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB,
        0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8, 0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90,
        0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65, 0x94, 0x89, 0xAE, 0xB3, 0xE0,
        0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F, 0x59, 0x44, 0x63,
        0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2, 0x26,
        0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80,
        0x9D, 0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77,
        0x6A, 0x4D, 0x50, 0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73,
        0x6E, 0x3D, 0x20, 0x07, 0x1A, 0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84,
        0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7, 0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B,
        0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E, 0xF8, 0xE5, 0xC2, 0xDF, 0x8C,
        0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43, 0xB2, 0xAF, 0x88,
        0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09, 0x7F,
        0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFe,
        //The “crc” of the position [255] is 0xD9 -> 0xFE XOR 0x11D = 0xD9 (1 byte).
        0xD9,
        //The “crc” of the position [256] is 0xC4 -> 0xFF XOR 0x11D = 0xC4 (1 byte).
        0xC4
    };

    // odrivetool function wrappers
    void tle_write(uint32_t command, uint32_t data); // Cast to uint16_t!!!
    void tle_read(uint32_t command, uint32_t length);
    uint32_t tle_get_rx();
};

#endif // __ENCODER_HPP
