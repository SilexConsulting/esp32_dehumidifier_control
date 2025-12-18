#pragma once

#include <Arduino.h>

class DehumidifierFSM {
public:
    enum Mode : uint8_t { MODE_OFF=0, MODE_AUTO=1, MODE_ON=2 };
    enum State : uint8_t {
        ST_IDLE=0,
        ST_ANTI_SHORT,    // waiting for min off time
        ST_STARTING_COMP, // compressor on, waiting to start fan (lead)
        ST_RUNNING,       // compressor+fan running
        ST_DEFROST,       // fan only until temp >= restart
        ST_FAN_LAG        // compressor off, fan running lag time
    };

    struct Pins {
        int gpio_comp = -1;
        int gpio_fan = -1;
        int gpio_demand = -1; // dry contact to GND, use INPUT_PULLUP
        int gpio_bucket = -1; // dry contact to GND, use INPUT_PULLUP
        int gpio_temp_adc = -1; // ADC1 pin
        int led_running = -1;
        int led_defrost = -1;
        int led_bucket = -1;
    };

    struct Config {
        float stop_temp_c = 5.0f;
        float restart_temp_c = 8.0f;
        uint32_t debounce_ms = 50;
        uint32_t fan_lead_ms = 7000;
        uint32_t fan_lag_ms = 7000;
        uint32_t min_comp_off_ms = 180000; // anti-short-cycle

        // NTC model (voltage divider with R_fixed to Vcc and NTC to GND)
        float ntc_beta = 3500.0f;
        float ntc_r0 = 10000.0f;
        float ntc_t0_k = 298.15f; // 25C
        float ntc_r_fixed = 12000.0f; // ohms

        // Optional Steinhart–Hart model (overrides simple Beta when enabled)
        // 1/T = A + B*ln(R) + C*ln(R)^3, T in Kelvin, R in ohms
        bool  ntc_use_steinhart = false;
        float ntc_sh_A = 1.0403e-3f;
        float ntc_sh_B = 2.3059e-4f;
        float ntc_sh_C = 7.2345e-8f;

        // ADC calibration scale for ESP32 non-linearity/FS error
        // Effective v_ratio = (raw/4095) * ntc_adc_scale (clamped to [0.001, 0.999])
        float ntc_adc_scale = 1.070f;

        // Sensor fault/timed-defrost fallback
        // If the controller observes no (or too few) valid temperature samples within a short window,
        // it will switch to a time-based defrost cycle until the sensor becomes healthy again.
        uint16_t fault_detect_window_samples = 50;    // window length (~2.5s at 50 ms tick)
        uint16_t fault_detect_min_valid_in_window = 5; // >=K valid samples clears the fault
        uint32_t fault_run_ms = 10 * 60 * 1000UL;      // default 10 minutes run
        uint32_t fault_rest_ms = 5 * 60 * 1000UL;      // default 5 minutes defrost (fan only)
    };

    void begin(const Pins& pins, const Config& cfg);
    void setMode(Mode m);
    Mode getMode() const { return mode_; }
    State getState() const { return state_; }
    void tick(uint32_t now_ms);

    float getLastTempC() const { return last_temp_c_; }

    // runtime I/O state getters for telemetry/UI
    bool isCompressorOn() const { return comp_on_; }
    bool isFanOn() const { return fan_on_; }
    bool isBucketOk() const { return bucket_ok_; }
    bool hasDemand() const { return demand_; }

    struct Timers {
        uint32_t anti_short_remaining_ms = 0; // time until compressor allowed to start
        uint32_t fan_lead_remaining_ms = 0;   // time until fan starts after compressor
        uint32_t fan_lag_remaining_ms = 0;    // remaining fan run after compressor stops
    };

    Timers getRemainingTimers(uint32_t now_ms) const;

private:
    void setCompressor(bool on, uint32_t now_ms);
    void setFan(bool on);
    void setLEDs(bool running_on, bool defrost_on, bool bucket_on);
    float readTempC();
    bool readDemandRaw();
    bool readBucketOkRaw();

    void updateDebounced(bool raw, bool& last_raw, bool& debounced, uint32_t& last_edge_ms, uint32_t now);
    void handleStopRequest(uint32_t now_ms);

    // transitions
    void goTo(State st, uint32_t now);
    void enforceOutputsForState(uint32_t now);

private:
    Pins pins_{};
    Config cfg_{};
    Mode mode_ = MODE_AUTO;
    State state_ = ST_IDLE;

    // inputs
    bool demand_raw_ = false, demand_ = false; // debounced, true = demand present
    bool bucket_raw_ok_ = true, bucket_ok_ = true; // true = safe
    bool demand_last_raw_ = false;
    bool bucket_last_raw_ = true;
    uint32_t demand_last_edge_ = 0;
    uint32_t bucket_last_edge_ = 0;

    // outputs state
    bool comp_on_ = false;
    bool fan_on_ = false;

    // timing
    uint32_t state_since_ = 0;
    uint32_t last_comp_off_ = 0; // for anti-short-cycle

    // telemetry
    float last_temp_c_ = NAN;

    // sensor fault detection (sliding window counters)
    uint16_t window_samples_ = 0;
    uint16_t window_valid_ = 0;
    bool sensor_fault_ = false;

    // timed fallback state tracking
    uint32_t fallback_phase_since_ = 0; // start time of the current fallback phase
    bool fallback_in_defrost_ = false;  // false=RUN phase, true=DEFROST phase
};
