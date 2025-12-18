#include "serial_setup.h"
#include "lfs.h"

#include <esp_wifi.h>
#include <esp_system.h>
#include <WifiManager.h>
#include <WiFi.h>

#include "fsm.h"
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Update.h>

#define FAN_GPIO 17
#define COMP_GPIO 16
#define BOOT_GPIO 0

// Chosen default GPIOs (can be rewired later if needed)
#define DEMAND_GPIO 25    // Dry contact: closed = demand present
#define TANK_GPIO   26    // Closed = bucket present & not full
#define TEMP_ADC    35    // ADC1 channel 7 (input-only GPIO)

#define RUN_LED_GPIO     15
#define DEFROST_LED_GPIO 4
#define BUCKET_LED_GPIO  19


void erase_wifi_config() {
    wifi_init_config_t wicfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wicfg);

    wifi_config_t wcfg{0};
    esp_wifi_set_config(WIFI_IF_STA, &wcfg);
}

const uint32_t MAGIC = 0x01234567;

struct configuration {
    uint32_t magic;
    uint16_t version;

    float stop_temp;
    float restart_temp;
    float timeout;
    uint8_t mode; // 0=OFF,1=AUTO,2=ON
    // Timed defrost fallback (when sensor is faulty): minutes
    float fault_run_min;   // compressor RUN duration in minutes (default 10)
    float fault_rest_min;  // DEFROST (fan-only) duration in minutes (default 5)
};

template<int N>
WiFiManagerParameter add_param(WiFiManager &wm, const char *id, const char *label, const char (&val)[N],
                               bool password = false) {
    const char *attr = password ? "type='password'" : nullptr;

    WiFiManagerParameter param(id, label, val, N - 1, attr);
    wm.addParameter(&param);
    return param;
}

WiFiManagerParameter add_param(WiFiManager &wm, const char *id, const char *label, float val, bool password = false) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", val);

    return add_param(wm, id, label, buf, password);
}

template<int N>
void param_value(WiFiManagerParameter &param, char (&cur_val)[N], bool &changed) {
    const char *val = param.getValue();

    if (strncmp(val, cur_val, N) != 0) {
        strncpy(cur_val, val, N);
        changed = true;
    }
}

void param_value(WiFiManagerParameter &param, float &cur_val, bool &changed) {
    const char *sval = param.getValue();
    float val;

    if (sscanf(sval, "%f", &val) == 1 && val != cur_val) {
        cur_val = val;
        changed = true;
    }
}

configuration conf;
static DehumidifierFSM fsm;
static WebServer server(80);

struct RuntimeStats {
    uint32_t magic = 0xABCDEF01;
    // milliseconds accumulated
    uint64_t comp_all_ms = 0;
    uint64_t fan_all_ms = 0;
    uint64_t comp_since_reset_ms = 0;
    uint64_t fan_since_reset_ms = 0;
};

static RuntimeStats stats{};
static uint32_t last_stats_tick = 0;
static bool last_comp = false, last_fan = false;
static uint64_t session_comp_ms = 0; // since power-on
static uint64_t session_fan_ms = 0;  // since power-on

static void load_stats() {
    littlefs lfs;
    auto s = lfs.load<RuntimeStats>("/stats.data");
    if (s.magic == 0xABCDEF01) {
        stats = s;
    } else {
        stats = RuntimeStats{};
    }
}

static void save_stats() {
    littlefs lfs;
    lfs.save("/stats.data", stats);
}

static void reset_stats() {
    stats.comp_since_reset_ms = 0;
    stats.fan_since_reset_ms = 0;
    save_stats();
}

static String html_page() {
    String ip = WiFi.isConnected() ? WiFi.localIP().toString() : String("not connected");
    String page;
    page.reserve(4096);
    page += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
    page += "<title>Dehumidifier Control</title><style>body{font-family:sans-serif;margin:1rem}fieldset{margin:1rem 0}label{display:block;margin:.3rem 0}</style></head><body>";
    page += "<h3>Dehumidifier Control</h3>";
    page += "<div>IP: "+ip+"</div>";
    page += "<fieldset><legend>Mode</legend><form method='POST' action='/api/mode'>";
    page += "<label><input type='radio' name='mode' value='0' "+String(fsm.getMode()==DehumidifierFSM::MODE_OFF?"checked":"")+"> OFF</label>";
    page += "<label><input type='radio' name='mode' value='1' "+String(fsm.getMode()==DehumidifierFSM::MODE_AUTO?"checked":"")+"> AUTO</label>";
    page += "<label><input type='radio' name='mode' value='2' "+String(fsm.getMode()==DehumidifierFSM::MODE_ON?"checked":"")+"> ON</label>";
    page += "<button type='submit'>Apply</button></form></fieldset>";

    page += "<fieldset><legend>Configuration</legend><form method='POST' action='/api/config'>";
    page += "<label>Defrost Stop Temp (&deg;C) <input name='stop_temp' type='number' step='0.1' value='"+String(conf.stop_temp,1)+"'></label>";
    page += "<label>Defrost Restart Temp (&deg;C) <input name='restart_temp' type='number' step='0.1' value='"+String(conf.restart_temp,1)+"'></label>";
    page += "<label>Fallback RUN (min) <input name='fault_run_min' type='number' step='0.1' value='"+String(conf.fault_run_min,1)+"'></label>";
    page += "<label>Fallback DEFROST (min) <input name='fault_rest_min' type='number' step='0.1' value='"+String(conf.fault_rest_min,1)+"'></label>";
    page += "<button type='submit'>Save</button></form></fieldset>";

    auto timers = fsm.getRemainingTimers(millis());
    page += "<fieldset><legend>Status</legend>";
    page += "<div>State: "+String((unsigned)fsm.getState())+" Mode: "+String((unsigned)fsm.getMode())+"</div>";
    page += "<div>Evap temp: "+String(fsm.getLastTempC(),2)+" &deg;C</div>";
    page += "<div>Compressor: "+String(fsm.isCompressorOn()?"ON":"OFF")+" Fan: "+String(fsm.isFanOn()?"ON":"OFF")+"</div>";
    page += "<div>Demand: "+String(fsm.hasDemand()?"YES":"NO")+" Bucket: "+String(fsm.isBucketOk()?"OK":"FULL/OPEN")+"</div>";
    page += "<div>Wait before start: "+String(timers.anti_short_remaining_ms/1000.0f,1)+" s</div>";
    page += "<div>Fan lead: "+String(timers.fan_lead_remaining_ms/1000.0f,1)+" s, Fan lag: "+String(timers.fan_lag_remaining_ms/1000.0f,1)+" s</div>";
    page += "</fieldset>";

    // OTA update link
    page += "<fieldset><legend>Firmware</legend>";
    page += "<div><a href='/update'>Upload new firmware</a></div>";
    page += "</fieldset>";

    auto fmt_dur = [](uint64_t ms){
        char buf[64];
        uint64_t s = ms/1000ULL; uint64_t m = s/60ULL; uint64_t h = m/60ULL; s%=60ULL; m%=60ULL;
        snprintf(buf,sizeof(buf),"%lluh %llum %llus", (unsigned long long)h, (unsigned long long)m, (unsigned long long)s);
        return String(buf);
    };
    page += "<fieldset><legend>Totals</legend>";
    page += "<div>Comp all-time: "+fmt_dur(stats.comp_all_ms)+" / since reset: "+fmt_dur(stats.comp_since_reset_ms)+"</div>";
    page += "<div>Fan all-time: "+fmt_dur(stats.fan_all_ms)+" / since reset: "+fmt_dur(stats.fan_since_reset_ms)+"</div>";
    page += "<form method='POST' action='/api/reset-stats'><button type='submit'>Reset Totals</button></form>";
    page += "</fieldset>";

    page += "<script>setInterval(()=>{fetch('/api/status').then(r=>r.json()).then(j=>{document.querySelectorAll('.live').forEach(e=>e.remove());let d=document.createElement('div');d.className='live';d.innerHTML=`Temp: ${j.temp_c} &deg;C, State ${j.state}, Comp ${j.comp?'ON':'OFF'}, Fan ${j.fan?'ON':'OFF'}`;document.body.insertBefore(d, document.body.firstChild);}).catch(()=>{/* swallow intermittent timeouts */});},5000);</script>";
    page += "</body></html>";
    return page;
}

static void handle_root() {
    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("Connection", "close");
    server.send(200, "text/html; charset=utf-8", html_page());
}

static void handle_status() {
    StaticJsonDocument<512> doc;
    auto t = fsm.getRemainingTimers(millis());
    doc["mode"] = (uint8_t)fsm.getMode();
    doc["state"] = (uint8_t)fsm.getState();
    doc["temp_c"] = fsm.getLastTempC();
    doc["comp"] = fsm.isCompressorOn();
    doc["fan"] = fsm.isFanOn();
    doc["demand"] = fsm.hasDemand();
    doc["bucket_ok"] = fsm.isBucketOk();
    doc["anti_s"] = t.anti_short_remaining_ms;
    doc["lead_ms"] = t.fan_lead_remaining_ms;
    doc["lag_ms"] = t.fan_lag_remaining_ms;
    doc["comp_all_ms"] = stats.comp_all_ms;
    doc["fan_all_ms"] = stats.fan_all_ms;
    doc["comp_since_reset_ms"] = stats.comp_since_reset_ms;
    doc["fan_since_reset_ms"] = stats.fan_since_reset_ms;
    doc["comp_since_boot_ms"] = session_comp_ms;
    doc["fan_since_boot_ms"] = session_fan_ms;
    String out; serializeJson(doc, out);
    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("Connection", "close");
    server.send(200, "application/json", out);
}

static void handle_mode() {
    if (!server.hasArg("mode")) { server.send(400, "text/plain", "mode missing"); return; }
    int m = server.arg("mode").toInt();
    if (m < 0) m = 0; if (m > 2) m = 2;
    conf.mode = (uint8_t)m;
    fsm.setMode(static_cast<DehumidifierFSM::Mode>(conf.mode));
    littlefs lfs; lfs.save("/config.data", conf);
    server.sendHeader("Location", "/");
    server.send(303);
}

static void handle_config() {
    auto getf = [](const String& s, float def)->float { char *end=nullptr; float v = s.toFloat(); return s.length()? v : def; };
    float stop = getf(server.arg("stop_temp"), conf.stop_temp);
    float restart = getf(server.arg("restart_temp"), conf.restart_temp);
    float frun = getf(server.arg("fault_run_min"), conf.fault_run_min);
    float frest = getf(server.arg("fault_rest_min"), conf.fault_rest_min);
    bool changed = (stop!=conf.stop_temp)||(restart!=conf.restart_temp)||(frun!=conf.fault_run_min)||(frest!=conf.fault_rest_min);
    conf.stop_temp = stop; conf.restart_temp = restart; conf.fault_run_min = frun; conf.fault_rest_min = frest;
    if (changed) {
        // apply to FSM live
        DehumidifierFSM::Config cfg{};
        cfg.stop_temp_c = conf.stop_temp;
        cfg.restart_temp_c = conf.restart_temp;
        cfg.debounce_ms = 50;
        cfg.fan_lead_ms = 7000;
        cfg.fan_lag_ms = 7000;
        cfg.min_comp_off_ms = 180000;
        cfg.ntc_beta = 3600.0f; cfg.ntc_r0 = 10000.0f; cfg.ntc_t0_k = 298.15f; cfg.ntc_r_fixed = 12000.0f;
        cfg.ntc_use_steinhart = false;
        cfg.ntc_sh_A = 1.0403e-3f; cfg.ntc_sh_B = 2.3059e-4f; cfg.ntc_sh_C = 7.2345e-8f;
        cfg.ntc_adc_scale = 1.045f;
        cfg.fault_run_ms = (uint32_t)(conf.fault_run_min * 60.0f * 1000.0f);
        cfg.fault_rest_ms = (uint32_t)(conf.fault_rest_min * 60.0f * 1000.0f);
        // Re-begin is heavy; we only update conf where used. For simplicity, just restart FSM pins unchanged.
        // Note: A richer API to update config live would be better.
        DehumidifierFSM::Pins pins_live{};
        pins_live.gpio_comp = COMP_GPIO;
        pins_live.gpio_fan = FAN_GPIO;
        pins_live.gpio_demand = DEMAND_GPIO;
        pins_live.gpio_bucket = TANK_GPIO;
        pins_live.gpio_temp_adc = TEMP_ADC;
        pins_live.led_running = RUN_LED_GPIO;
        pins_live.led_defrost = DEFROST_LED_GPIO;
        pins_live.led_bucket = BUCKET_LED_GPIO;
        fsm.begin(pins_live, cfg);
        fsm.setMode(static_cast<DehumidifierFSM::Mode>(conf.mode));
        littlefs lfs; lfs.save("/config.data", conf);
    }
    server.sendHeader("Location", "/");
    server.send(303);
}

static void handle_reset_stats() {
    reset_stats();
    server.sendHeader("Location", "/");
    server.send(303);
}

// ---------- OTA update (with basic auth) ----------
static const char* OTA_USER = "update"; // per user request
static const char* OTA_PASS = "caution"; // per user request

static bool ensure_auth() {
    if (!server.authenticate(OTA_USER, OTA_PASS)) {
        server.requestAuthentication();
        return false;
    }
    return true;
}

static void handle_update_form() {
    if (!ensure_auth()) return;
    String ip = WiFi.isConnected() ? WiFi.localIP().toString() : String("not connected");
    String html;
    html.reserve(2048);
    html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>OTA Update</title><style>body{font-family:sans-serif;margin:1rem}fieldset{margin:1rem 0}</style></head><body>";
    html += "<h3>Firmware Update</h3>";
    html += "<div>IP: "+ip+"</div>";
    html += "<p><b>Caution:</b> Make sure power is stable. Do not navigate away until the upload completes.</p>";
    html += "<form method='POST' action='/update' enctype='multipart/form-data'>";
    html += "<input type='file' name='firmware' accept='.bin' required> ";
    html += "<button type='submit'>Upload</button>";
    html += "</form>";
    html += "<p><a href='/'>Back</a></p>";
    html += "</body></html>";
    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("Connection", "close");
    server.send(200, "text/html; charset=utf-8", html);
}

static void handle_update_upload() {
    if (!ensure_auth()) return;
    if (Update.hasError()) {
        server.send(500, "text/plain", "Update failed");
        return;
    }
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "Update OK, rebooting...\n");
    delay(200);
    ESP.restart();
}

static void handle_update_stream() {
    if (!ensure_auth()) return;
    HTTPUpload &up = server.upload();
    if (up.status == UPLOAD_FILE_START) {
        Serial.printf("OTA: Update start: %s\n", up.filename.c_str());
        if (!Update.begin()) { // auto partition
            Update.printError(Serial);
        }
    } else if (up.status == UPLOAD_FILE_WRITE) {
        if (Update.write(up.buf, up.currentSize) != up.currentSize) {
            Update.printError(Serial);
        }
    } else if (up.status == UPLOAD_FILE_END) {
        if (!Update.end(true)) {
            Update.printError(Serial);
        } else {
            Serial.printf("OTA: Update success, size=%u bytes\n", up.totalSize);
        }
    } else if (up.status == UPLOAD_FILE_ABORTED) {
        Update.end();
        Serial.println("OTA: Update aborted");
    }
}

void setup() {
    serial_setup();

    // Print reset reason early to diagnose unexpected restarts
    const auto rr = esp_reset_reason();
    Serial.printf("Reset reason: %d\n", (int)rr);

    littlefs lfs;
    bool should_save = false;

    conf = lfs.load<configuration>("/config.data");
    Serial.printf("Loaded conf: magic=%08x ver=%u stop=%.1f restart=%.1f timeout=%.1f mode=%u run_min=%.1f rest_min=%.1f\n",
                  conf.magic, conf.version, conf.stop_temp, conf.restart_temp, conf.timeout, conf.mode,
                  conf.fault_run_min, conf.fault_rest_min);

    if (conf.magic != MAGIC) {
        conf = configuration{
            .magic = MAGIC,
            .version = 2,
            .stop_temp = 6,
            .restart_temp = 9,
            .timeout = 3,
            .mode = 1, // default AUTO
            .fault_run_min = 10.0f,
            .fault_rest_min = 5.0f
        };

        should_save = true;
    } else if (conf.version < 2) {
        // migrate defaults for new fields introduced in v2
        if (conf.version < 1) {
            conf.mode = 1; // default AUTO
        }
        conf.fault_run_min = (conf.fault_run_min > 0.0f) ? conf.fault_run_min : 10.0f;
        conf.fault_rest_min = (conf.fault_rest_min > 0.0f) ? conf.fault_rest_min : 5.0f;
        conf.version = 2;
        should_save = true;
    }

    WiFiManager wifiManager;

    auto stop_temp = add_param(wifiManager, "stop_temp", "Defrost Stop Temp (°C)", conf.stop_temp);
    auto restart_temp = add_param(wifiManager, "restart_temp", "Defrost Restart Temp (°C)", conf.restart_temp);
    auto timeout = add_param(wifiManager, "timeout", "WiFi Timeout (min)", conf.timeout);
    auto fault_run = add_param(wifiManager, "fault_run_min", "Fallback RUN (min)", conf.fault_run_min);
    auto fault_rest = add_param(wifiManager, "fault_rest_min", "Fallback DEFROST (min)", conf.fault_rest_min);

    char mode_buf[4];
    snprintf(mode_buf, sizeof(mode_buf), "%u", (unsigned) conf.mode);
    auto mode_param = add_param(wifiManager, "mode", "Mode 0=OFF 1=AUTO 2=ON", mode_buf);

    wifiManager.setSaveConfigCallback([&]() {
        Serial.println("WiFiManager: saveConfigCallback");
        param_value(stop_temp, conf.stop_temp, should_save);
        param_value(restart_temp, conf.restart_temp, should_save);
        param_value(timeout, conf.timeout, should_save);
        param_value(fault_run, conf.fault_run_min, should_save);
        param_value(fault_rest, conf.fault_rest_min, should_save);
        bool dummy_changed = false;
        char cur_mode[4];
        snprintf(cur_mode, sizeof(cur_mode), "%u", (unsigned) conf.mode);
        param_value(mode_param, cur_mode, dummy_changed);
        unsigned nm = conf.mode;
        sscanf(mode_param.getValue(), "%u", &nm);
        nm = (nm > 2) ? 2 : nm;
        if (conf.mode != nm) {
            conf.mode = (uint8_t) nm;
            should_save = true;
        }
        Serial.printf("Parsed: stop=%.1f restart=%.1f timeout=%.1f mode=%u run_min=%.1f rest_min=%.1f should_save=%d\n",
                      conf.stop_temp, conf.restart_temp, conf.timeout, conf.mode,
                      conf.fault_run_min, conf.fault_rest_min, should_save);
    });

    wifiManager.setSaveParamsCallback([&]() {
        Serial.println("WiFiManager: saveParamsCallback fired");
        param_value(stop_temp, conf.stop_temp, should_save);
        param_value(restart_temp, conf.restart_temp, should_save);
        param_value(timeout, conf.timeout, should_save);
        param_value(fault_run, conf.fault_run_min, should_save);
        param_value(fault_rest, conf.fault_rest_min, should_save);
        bool dummy_changed = false;
        char cur_mode[4];
        snprintf(cur_mode, sizeof(cur_mode), "%u", (unsigned) conf.mode);
        param_value(mode_param, cur_mode, dummy_changed);
        unsigned nm = conf.mode;
        sscanf(mode_param.getValue(), "%u", &nm);
        nm = (nm > 2) ? 2 : nm;
        if (conf.mode != nm) {
            conf.mode = (uint8_t) nm;
            should_save = true;
        }
        Serial.printf("Parsed: stop=%.1f restart=%.1f timeout=%.1f mode=%u run_min=%.1f rest_min=%.1f should_save=%d\n",
                      conf.stop_temp, conf.restart_temp, conf.timeout, conf.mode,
                      conf.fault_run_min, conf.fault_rest_min, should_save);
        if (should_save) {
           Serial.println("Saving configuration to LFS (from callback)...");
           littlefs lfs;               // create a temp instance
           lfs.save("/config.data", conf);
        }
    });

    pinMode(BOOT_GPIO, INPUT_PULLUP);
    const bool forcePortal = (digitalRead(BOOT_GPIO) == LOW);

    if (forcePortal) {
        Serial.println("BOOT(GPIO0) held: forcing WiFiManager portal");
        wifiManager.setConfigPortalTimeout(180);

        // Optional but recommended if you want a "clean" reconfigure:
        // remove saved SSID/pass so it won't instantly reconnect next boot.
        //wifiManager.resetSettings();

        if (!wifiManager.startConfigPortal("HUMIDIFIER_CTRL")) {
            Serial.println("ERROR: config portal timeout");
            delay(1000);
            ESP.restart();
        }
    } else {
        if (!wifiManager.autoConnect("HUMIDIFIER_CTRL")) {
            Serial.println("ERROR: failed to connect and hit timeout");
            delay(3000);
            ESP.restart();
        }
    }

    // Improve Wi‑Fi responsiveness: disable modem sleep and set full TX power.
    // This reduces latency at the cost of a small increase in power usage.
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    if (should_save || forcePortal) {
        lfs.save("/config.data", conf);
    }

    // Initialize FSM
    DehumidifierFSM::Pins pins{};
    pins.gpio_comp = COMP_GPIO;
    pins.gpio_fan = FAN_GPIO;
    pins.gpio_demand = DEMAND_GPIO;
    pins.gpio_bucket = TANK_GPIO;
    pins.gpio_temp_adc = TEMP_ADC;
    pins.led_running = RUN_LED_GPIO;
    pins.led_defrost = DEFROST_LED_GPIO;
    pins.led_bucket = BUCKET_LED_GPIO;

    DehumidifierFSM::Config cfg{};
    cfg.stop_temp_c = conf.stop_temp;
    cfg.restart_temp_c = conf.restart_temp;
    cfg.debounce_ms = 50;
    cfg.fan_lead_ms = 7000;
    cfg.fan_lag_ms = 7000;
    cfg.min_comp_off_ms = 180000; // 180s anti-short-cycle
    // Legacy simple Beta params (kept for compatibility if SH is disabled)
    cfg.ntc_beta = 3600.0f;
    cfg.ntc_r0 = 10000.0f;
    cfg.ntc_t0_k = 298.15f; // 25C
    cfg.ntc_r_fixed = 12000.0f; // divider fixed resistor to 3.3V

    // Enable Steinhart–Hart using calibrated coefficients from your measurements
    cfg.ntc_use_steinhart = false;
    cfg.ntc_sh_A = 1.0403e-3f;
    cfg.ntc_sh_B = 2.3059e-4f;
    cfg.ntc_sh_C = 7.2345e-8f;

    // ADC calibration scale to align ADC ratio with measured divider behavior
    cfg.ntc_adc_scale = 1.045f;

    // sensor fault fallback timings
    cfg.fault_run_ms = (uint32_t)(conf.fault_run_min * 60.0f * 1000.0f);
    cfg.fault_rest_ms = (uint32_t)(conf.fault_rest_min * 60.0f * 1000.0f);

    adcAttachPin(TEMP_ADC);
    analogSetPinAttenuation(TEMP_ADC, ADC_11db); // ~0–3.6V range
    analogReadResolution(12);

    fsm.begin(pins, cfg);
    fsm.setMode(static_cast<DehumidifierFSM::Mode>(conf.mode));

    Serial.println("Dehumidifier started:" + WiFi.localIP().toString());

    // Load runtime stats and init server
    load_stats();
    last_stats_tick = millis();
    last_comp = fsm.isCompressorOn();
    last_fan = fsm.isFanOn();

    server.on("/", HTTP_GET, handle_root);
    server.on("/api/status", HTTP_GET, handle_status);
    server.on("/api/mode", HTTP_POST, handle_mode);
    server.on("/api/config", HTTP_POST, handle_config);
    server.on("/api/reset-stats", HTTP_POST, handle_reset_stats);
    // OTA endpoints (basic auth inside handlers)
    server.on("/update", HTTP_GET, handle_update_form);
    server.on("/update", HTTP_POST, handle_update_upload, handle_update_stream);
    server.begin();
}

void loop() {
    static uint32_t last_tick = 0;
    static uint32_t last_temp_log = 0;
    const uint32_t now = millis();

    if (now - last_tick >= 50) {
        last_tick = now;
        fsm.tick(now);
    }

    if (now - last_temp_log >= 3000) {
        last_temp_log = now;
        auto timers = fsm.getRemainingTimers(now);
        const float anti_s = timers.anti_short_remaining_ms / 1000.0f;
        const float lead_s = timers.fan_lead_remaining_ms / 1000.0f;
        const float lag_s = timers.fan_lag_remaining_ms / 1000.0f;

        const bool bucket_ok_raw = (digitalRead(TANK_GPIO) == LOW);
        Serial.printf(
            "Evap temp: %.2f C, mode=%u, state=%u, wait_start=%.1fs, lead=%.1fs, lag=%.1fs, bucket=%s\n",
            fsm.getLastTempC(),
            (unsigned) fsm.getMode(),
            (unsigned) fsm.getState(),
            anti_s,
            lead_s,
            lag_s,
            bucket_ok_raw ? "OK" : "FULL/OPEN"
        );
        Serial.printf("raw=%d\n", analogRead(TEMP_ADC));
    }

    // web server
    server.handleClient();

    // update runtime stats roughly each second
    static bool stats_dirty = false;
    if (now - last_stats_tick >= 1000) {
        uint32_t dt = now - last_stats_tick; // ms
        last_stats_tick = now;
        bool comp = fsm.isCompressorOn();
        bool fan = fsm.isFanOn();
        if (comp) {
            stats.comp_all_ms += dt;
            stats.comp_since_reset_ms += dt;
            session_comp_ms += dt;
            stats_dirty = true;
        }
        if (fan) {
            stats.fan_all_ms += dt;
            stats.fan_since_reset_ms += dt;
            session_fan_ms += dt;
            stats_dirty = true;
        }
        // save every 30s when running to keep totals persistent
        static uint32_t last_save = 0;
        if (stats_dirty && (now - last_save >= 60000)) { // save at most once a minute and only when changed
            last_save = now;
            save_stats();
            stats_dirty = false;
        }
        last_comp = comp; last_fan = fan;
    }

    // Let the scheduler breathe to keep Wi‑Fi/TCP healthy on busy loops
    yield();
}
