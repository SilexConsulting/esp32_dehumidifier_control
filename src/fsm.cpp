#include "fsm.h"
#include <math.h>

void DehumidifierFSM::begin(const Pins& pins, const Config& cfg) {
    pins_ = pins;
    cfg_ = cfg;

    if (pins_.gpio_comp >= 0) { pinMode(pins_.gpio_comp, OUTPUT); digitalWrite(pins_.gpio_comp, LOW); }
    if (pins_.gpio_fan >= 0)  { pinMode(pins_.gpio_fan, OUTPUT);  digitalWrite(pins_.gpio_fan, LOW); }
    if (pins_.led_running >= 0)  { pinMode(pins_.led_running, OUTPUT); digitalWrite(pins_.led_running, HIGH); }
    if (pins_.led_defrost >= 0)  { pinMode(pins_.led_defrost, OUTPUT); digitalWrite(pins_.led_defrost, HIGH); }
    if (pins_.led_bucket >= 0)   { pinMode(pins_.led_bucket, OUTPUT); digitalWrite(pins_.led_bucket, HIGH); }

    if (pins_.gpio_demand >= 0) { pinMode(pins_.gpio_demand, INPUT_PULLUP); }
    if (pins_.gpio_bucket >= 0) { pinMode(pins_.gpio_bucket, INPUT_PULLUP); }

    state_ = ST_IDLE;
    state_since_ = millis();
    last_comp_off_ = state_since_;

    setCompressor(false, state_since_);
    setFan(false);
    setLEDs(false, false, false);

    // init new members
    window_samples_ = 0;
    window_valid_ = 0;
    sensor_fault_ = false;
    fallback_phase_since_ = 0;
    fallback_in_defrost_ = false;
}

void DehumidifierFSM::setMode(Mode m) {
    mode_ = m;
}

void DehumidifierFSM::tick(uint32_t now_ms) {
    // sample inputs
    last_temp_c_ = readTempC();
    // sensor fault sliding window accounting
    if (window_samples_ < cfg_.fault_detect_window_samples) {
        window_samples_++;
        if (last_temp_c_ == last_temp_c_) {
            // valid reading
            if (window_valid_ < 0xFFFF) window_valid_++;
        }
    }
    if (window_samples_ >= cfg_.fault_detect_window_samples) {
        const bool had_enough_valid = (window_valid_ >= cfg_.fault_detect_min_valid_in_window);
        const bool had_none_valid = (window_valid_ == 0);
        bool prev_fault = sensor_fault_;
        if (had_none_valid) {
            sensor_fault_ = true;
        } else if (had_enough_valid) {
            sensor_fault_ = false;
        }
        // reset window
        window_samples_ = 0;
        window_valid_ = 0;
        if (sensor_fault_ && !prev_fault) {
            // entering fallback mode: start RUN phase
            fallback_in_defrost_ = false;
            fallback_phase_since_ = now_ms;
        }
    }
    demand_raw_ = readDemandRaw();
    bucket_raw_ok_ = readBucketOkRaw();

    updateDebounced(demand_raw_, demand_last_raw_, demand_, demand_last_edge_, now_ms);
    updateDebounced(bucket_raw_ok_, bucket_last_raw_, bucket_ok_, bucket_last_edge_, now_ms);

    const bool run_request = (mode_ == MODE_ON) || (mode_ == MODE_AUTO && demand_);

    // global bucket safety: if bucket not ok -> stop comp immediately, manage fan lag
    if (!bucket_ok_) {
        handleStopRequest(now_ms);
        return;
    }

    if (!run_request) {
        handleStopRequest(now_ms);
        return;
    }

    // Determine defrost state
    bool defrost_active = false;
    if (sensor_fault_) {
        // Ensure phase timing initialized
        if (fallback_phase_since_ == 0) fallback_phase_since_ = now_ms;

        // Check for phase transitions
        if (!fallback_in_defrost_) {
            if ((now_ms - fallback_phase_since_) >= cfg_.fault_run_ms) {
                fallback_in_defrost_ = true;
                fallback_phase_since_ = now_ms;
            }
        } else {
            if ((now_ms - fallback_phase_since_) >= cfg_.fault_rest_ms) {
                fallback_in_defrost_ = false;
                fallback_phase_since_ = now_ms;
            }
        }
        defrost_active = fallback_in_defrost_;
    } else {
        // Temperature-based defrost logic
        const bool defrost_needed = (last_temp_c_ == last_temp_c_) && (last_temp_c_ < cfg_.stop_temp_c);
        const bool defrost_clear = (last_temp_c_ == last_temp_c_) && (last_temp_c_ >= cfg_.restart_temp_c);

        if (state_ == ST_DEFROST) {
            defrost_active = !defrost_clear;
        } else {
            defrost_active = defrost_needed;
        }
    }

    if (defrost_active) {
        if (state_ != ST_DEFROST) {
            goTo(ST_DEFROST, now_ms);
        } else {
            enforceOutputsForState(now_ms);
        }
        return;
    }

    // Normal operation (run requested and no defrost)
    switch (state_) {
        case ST_IDLE:
        case ST_FAN_LAG:
        case ST_DEFROST:
        case ST_ANTI_SHORT: {
            const uint32_t since_off = now_ms - last_comp_off_;
            if (since_off < cfg_.min_comp_off_ms) {
                if (state_ != ST_ANTI_SHORT) {
                    goTo(ST_ANTI_SHORT, now_ms);
                }
            } else {
                goTo(ST_STARTING_COMP, now_ms);
            }
            break;
        }
        case ST_STARTING_COMP: {
            if (now_ms - state_since_ >= cfg_.fan_lead_ms) {
                goTo(ST_RUNNING, now_ms);
            } else {
                enforceOutputsForState(now_ms);
            }
            break;
        }
        case ST_RUNNING:
            enforceOutputsForState(now_ms);
            break;
    }
}

void DehumidifierFSM::handleStopRequest(uint32_t now_ms) {
    setCompressor(false, now_ms);
    if (state_ == ST_FAN_LAG) {
        if (now_ms - state_since_ >= cfg_.fan_lag_ms) {
            setFan(false);
            if (state_ != ST_IDLE) goTo(ST_IDLE, now_ms);
        }
    } else if (fan_on_) {
        goTo(ST_FAN_LAG, now_ms);
    } else {
        if (state_ != ST_IDLE) goTo(ST_IDLE, now_ms);
    }
    enforceOutputsForState(now_ms);
}

void DehumidifierFSM::setCompressor(bool on, uint32_t now_ms) {
    if (comp_on_ && !on) {
        last_comp_off_ = now_ms;
    }
    comp_on_ = on;
    if (pins_.gpio_comp >= 0) digitalWrite(pins_.gpio_comp, on ? HIGH : LOW);
}

void DehumidifierFSM::setFan(bool on) {
    fan_on_ = on;
    if (pins_.gpio_fan >= 0) digitalWrite(pins_.gpio_fan, on ? HIGH : LOW);
}

void DehumidifierFSM::setLEDs(bool running_on, bool defrost_on, bool bucket_on) {
    // LEDs are wired with common positive (active-low): LOW = on, HIGH = off
    if (pins_.led_running >= 0) digitalWrite(pins_.led_running, running_on ? LOW : HIGH);
    if (pins_.led_defrost >= 0) digitalWrite(pins_.led_defrost, defrost_on ? LOW : HIGH);
    if (pins_.led_bucket >= 0)  digitalWrite(pins_.led_bucket, bucket_on ? LOW : HIGH);
}

float DehumidifierFSM::readTempC() {
    if (pins_.gpio_temp_adc < 0) return NAN;
    int raw = analogRead(pins_.gpio_temp_adc); // 0..4095
    if (raw <= 0) return NAN; // open or 0V
    if (raw >= 4095) return NAN; // saturated

    // ESP32 ADC is not perfectly linear; apply configurable scale factor
    const float r_fixed = cfg_.ntc_r_fixed;
    float v_ratio = ((float)raw / 4095.0f) * cfg_.ntc_adc_scale;
    // Clamp to avoid div-by-zero and out-of-range
    if (v_ratio < 0.001f) v_ratio = 0.001f;
    if (v_ratio > 0.999f) v_ratio = 0.999f;
    // divider: Vout = Vcc * Rntc / (Rfixed + Rntc) -> Rntc = Rfixed * Vout / (1 - Vout)
    float r_ntc = r_fixed * (v_ratio / (1.0f - v_ratio));
    if (r_ntc <= 0) return NAN;

    float T;
    if (cfg_.ntc_use_steinhart) {
        // Steinhart–Hart: 1/T = A + B*ln(R) + C*ln(R)^3
        const float lnR = logf(r_ntc);
        const float invT = cfg_.ntc_sh_A + cfg_.ntc_sh_B * lnR + cfg_.ntc_sh_C * lnR * lnR * lnR; // 1/K
        if (invT <= 0.0f) return NAN;
        T = 1.0f / invT; // Kelvin
    } else {
        // Simple Beta model referenced to T0
        const float invT = (1.0f / cfg_.ntc_t0_k) + (1.0f / cfg_.ntc_beta) * logf(r_ntc / cfg_.ntc_r0);
        T = 1.0f / invT; // Kelvin
    }
    float celsius = T - 273.15f;

    // Sanity check: if temp is physically implausible, treat as sensor fault (NAN)
    if (celsius < -40.0f || celsius > 100.0f) return NAN;

    return celsius;
}

bool DehumidifierFSM::readDemandRaw() {
    if (pins_.gpio_demand < 0) return false;
    // active low contact
    return digitalRead(pins_.gpio_demand) == LOW;
}

bool DehumidifierFSM::readBucketOkRaw() {
    if (pins_.gpio_bucket < 0) return true;
    // active low contact: LOW means OK (present and not full)
    return digitalRead(pins_.gpio_bucket) == LOW;
}

void DehumidifierFSM::updateDebounced(bool raw, bool& last_raw, bool& debounced, uint32_t& last_edge_ms, uint32_t now) {
    if (raw != last_raw) {
        last_raw = raw;
        last_edge_ms = now;
        return;
    }
    if ((now - last_edge_ms) >= cfg_.debounce_ms && debounced != raw) {
        debounced = raw;
    }
}

void DehumidifierFSM::goTo(State st, uint32_t now) {
    state_ = st;
    state_since_ = now;
    enforceOutputsForState(now);
}




void DehumidifierFSM::enforceOutputsForState(const uint32_t now) {
    switch (state_) {
        case ST_IDLE:
            setCompressor(false, now);
            setFan(false);
            break;
        case ST_ANTI_SHORT:
            setCompressor(false, now);
            setFan(true); // keep air moving while waiting
            break;
        case ST_STARTING_COMP:
            setCompressor(true, now); // start compressor first
            // If the fan is already running (e.g. from DEFROST or FAN_LAG), keep it running.
            // Otherwise, wait for the lead timer to expire before starting it.
            if (!fan_on_) {
                setFan(false);
            } else {
                setFan(true);
            }
            break;
        case ST_RUNNING:
            setFan(true);
            setCompressor(true, now);
            break;
        case ST_DEFROST:
            setCompressor(false, now);
            setFan(true);
            break;
        case ST_FAN_LAG:
            setCompressor(false, now);
            setFan(true);
            break;
    }

    // LEDs
    const bool running_led = fan_on_ || comp_on_;
    const bool defrost_led = (state_ == ST_DEFROST);
    const bool bucket_led = !bucket_ok_;
    setLEDs(running_led, defrost_led, bucket_led);
}

DehumidifierFSM::Timers DehumidifierFSM::getRemainingTimers(uint32_t now_ms) const {
    Timers t{};

    // Anti-short: only relevant if a run is requested and bucket is OK
    const bool run_request = (mode_ == MODE_ON) || (mode_ == MODE_AUTO && demand_);
    const bool bucket_ok = bucket_ok_;
    if (run_request && bucket_ok) {
        uint32_t since_off = now_ms - last_comp_off_;
        if (since_off < cfg_.min_comp_off_ms) {
            t.anti_short_remaining_ms = cfg_.min_comp_off_ms - since_off;
        }
    }

    // Fan lead: applies while in STARTING_COMP
    if (state_ == ST_STARTING_COMP) {
        uint32_t since = now_ms - state_since_;
        if (since < cfg_.fan_lead_ms) {
            t.fan_lead_remaining_ms = cfg_.fan_lead_ms - since;
        }
    }

    // Fan lag: applies while in FAN_LAG
    if (state_ == ST_FAN_LAG) {
        uint32_t since = now_ms - state_since_;
        if (since < cfg_.fan_lag_ms) {
            t.fan_lag_remaining_ms = cfg_.fan_lag_ms - since;
        }
    }

    return t;
}
