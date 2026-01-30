#include <utils/flog.h>
#include <module.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <core.h>
#include <gui/style.h>
#include <gui/smgui.h>
#include <iio.h>
#include <ad9361.h>
#include <utils/optionlist.h>
#include <algorithm>
#include <regex>
#include <cmath>

#define CONCAT(a, b) ((std::string(a) + b).c_str())
#define M_PI 3.141593

SDRPP_MOD_INFO{
    /* Name:            */ "plutosdr_source",
    /* Description:     */ "PlutoSDR source module for SDR++",
    /* Author:          */ "Ryzerth",
    /* Version:         */ 0, 3, 0,
    /* Max instances    */ 1
};

ConfigManager config;

const std::vector<const char*> deviceWhiteList = {
    "PlutoSDR",
    "ANTSDR",
    "LibreSDR"
};

class PlutoSDRSourceModule : public ModuleManager::Instance {
public:
    PlutoSDRSourceModule(std::string name) {
        this->name = name;

        // Define valid samplerates
        for (int sr = 1000000; sr <= 61440000; sr += 500000) {
            samplerates.define(sr, getBandwdithScaled(sr), sr);
        }
        samplerates.define(61440000, getBandwdithScaled(61440000.0), 61440000.0);

        // Define valid bandwidths
        bandwidths.define(0, "Auto", 0);
        for (int bw = 1000000.0; bw <= 52000000; bw += 500000) {
            bandwidths.define(bw, getBandwdithScaled(bw), bw);
        }

        // Define gain modes
        gainModes.define("manual", "Manual", "manual");
        gainModes.define("fast_attack", "Fast Attack", "fast_attack");
        gainModes.define("slow_attack", "Slow Attack", "slow_attack");
        gainModes.define("hybrid", "Hybrid", "hybrid");

        // Define TX ports
        txPorts.define("TX1", "TX1", "TX1");
        txPorts.define("TX2", "TX2", "TX2");

        // Load configuration
        config.acquire();
        devDesc = config.conf["device"];
        if (config.conf.contains("customIP")) {
            customIP = config.conf["customIP"];
            strncpy(ipBuf, customIP.c_str(), sizeof(ipBuf) - 1);
            ipBuf[sizeof(ipBuf) - 1] = '\0';
        }
        if (config.conf.contains("txEnabled")) {
            txEnabled = config.conf["txEnabled"];
        }
        if (config.conf.contains("txPort")) {
            std::string txPortStr = config.conf["txPort"];
            if (txPorts.keyExists(txPortStr)) {
                txPortId = txPorts.keyId(txPortStr);
            }
        }
        if (config.conf.contains("txFreq")) {
            txFreq = config.conf["txFreq"];
        }
        if (config.conf.contains("txAttenuation")) {
            txAttenuation = config.conf["txAttenuation"];
            txAttenuation = std::clamp<float>(txAttenuation, 0.0f, 89.75f);
        }
        if (config.conf.contains("txToneFreq")) {
            txToneFreq = config.conf["txToneFreq"];
        }
        config.release();

        // Enumerate devices
        refresh();

        // Select device
        select(devDesc);

        // Register source
        handler.ctx = this;
        handler.selectHandler = menuSelected;
        handler.deselectHandler = menuDeselected;
        handler.menuHandler = menuHandler;
        handler.startHandler = start;
        handler.stopHandler = stop;
        handler.tuneHandler = tune;
        handler.stream = &stream;
        sigpath::sourceManager.registerSource("PlutoSDR", &handler);
    }

    ~PlutoSDRSourceModule() {
        stop(this);
        sigpath::sourceManager.unregisterSource("PlutoSDR");
    }

    void postInit() {}

    void enable() {
        enabled = true;
    }

    void disable() {
        enabled = true;
    }

    bool isEnabled() {
        return enabled;
    }

private:
    std::string getBandwdithScaled(double bw) {
        char buf[1024];
        if (bw >= 1000000.0) {
            sprintf(buf, "%.1lfMHz", bw / 1000000.0);
        }
        else if (bw >= 1000.0) {
            sprintf(buf, "%.1lfKHz", bw / 1000.0);
        }
        else {
            sprintf(buf, "%.1lfHz", bw);
        }
        return std::string(buf);
    }

    void refresh() {
        // Clear device list
        devices.clear();

        // Add custom IP option first
        devices.define("ip_custom", "Custom IP Connection", "ip_custom");

        // Create scan context for USB devices
        iio_scan_context* sctx = iio_create_scan_context(NULL, 0);
        if (!sctx) {
            flog::error("Failed get scan context");
            return;
        }

        // Create parsing regexes
        std::regex backendRgx(".+(?=:)", std::regex::ECMAScript);
        std::regex modelRgx("\\(.+(?=\\),)", std::regex::ECMAScript);
        std::regex serialRgx("serial=[0-9A-Za-z]+", std::regex::ECMAScript);

        // Enumerate devices
        iio_context_info** ctxInfoList;
        ssize_t count = iio_scan_context_get_info_list(sctx, &ctxInfoList);
        if (count < 0) {
            flog::error("Failed to enumerate contexts");
            return;
        }
        for (ssize_t i = 0; i < count; i++) {
            iio_context_info* info = ctxInfoList[i];
            std::string desc = iio_context_info_get_description(info);
            std::string duri = iio_context_info_get_uri(info);

            // If the device is not a plutosdr, don't include it
            bool isPluto = false;
            for (const auto type : deviceWhiteList) {
                if (desc.find(type) != std::string::npos) {
                    isPluto = true;
                    break;
                }
            }
            if (!isPluto) {
                flog::warn("Ignored IIO device: [{}] {}", duri, desc);
                continue;
            }

            // Extract the backend
            std::string backend = "unknown";
            std::smatch backendMatch;
            if (std::regex_search(duri, backendMatch, backendRgx)) {
                backend = backendMatch[0];
            }

            // Extract the model
            std::string model = "Unknown";
            std::smatch modelMatch;
            if (std::regex_search(desc, modelMatch, modelRgx)) {
                model = modelMatch[0];
                int parenthPos = model.find('(');
                if (parenthPos != std::string::npos) {
                    model = model.substr(parenthPos+1);
                }
            }

            // Extract the serial
            std::string serial = "unknown";
            std::smatch serialMatch;
            if (std::regex_search(desc, serialMatch, serialRgx)) {
                serial = serialMatch[0].str().substr(7);
            }

            // Construct the device name
            std::string devName = '(' + backend + ") " + model + " [" + serial + ']';

            // Skip duplicate devices
            if (devices.keyExists(desc) || devices.nameExists(devName) || devices.valueExists(duri)) { continue; }

            // Save device
            devices.define(desc, devName, duri);
        }
        iio_context_info_list_free(ctxInfoList);
        
        // Destroy scan context
        iio_scan_context_destroy(sctx);

#ifdef __ANDROID__
        // On Android, a default IP entry must be made (TODO: This is not ideal since the IP cannot be changed)
        const char* androidURI = "ip:192.168.2.1";
        const char* androidName = "Default (192.168.2.1)";
        devices.define(androidName, androidName, androidURI);
#endif
    }

    void select(const std::string& desc) {
        // If custom IP is selected
        if (desc == "ip_custom") {
            devDesc = desc;
            uri = "ip:" + customIP;
            return;
        }

        // If no device is available, give up
        if (devices.empty() || !devices.keyExists(desc)) {
            // If custom IP exists, select it
            if (devices.keyExists("ip_custom")) {
                select("ip_custom");
            }
            return;
        }

        // Update URI
        devDesc = desc;
        uri = devices.value(devices.keyId(desc));

        // TODO: Enumerate capabilities

        // Load defaults
        samplerate = 4000000;
        bandwidth = 0;
        gmId = 0;
        gain = -1.0f;

        // Load device config
        config.acquire();
        if (config.conf["devices"][devDesc].contains("samplerate")) {
            samplerate = config.conf["devices"][devDesc]["samplerate"];
        }
        if (config.conf["devices"][devDesc].contains("bandwidth")) {
            bandwidth = config.conf["devices"][devDesc]["bandwidth"];
        }
        if (config.conf["devices"][devDesc].contains("gainMode")) {
            std::string gm = config.conf["devices"][devDesc]["gainMode"];
            if (gainModes.keyExists(gm)) {
                gmId = gainModes.keyId(gm);
            }
            else {
                gmId = 0;
            }
        }
        if (config.conf["devices"][devDesc].contains("gain")) {
            gain = config.conf["devices"][devDesc]["gain"];
            gain = std::clamp<int>(gain, -1.0f, 73.0f);
        }
        config.release();

        // Update samplerate ID
        if (samplerates.keyExists(samplerate)) {
            srId = samplerates.keyId(samplerate);
        }
        else {
            srId = 0;
            samplerate = samplerates.value(srId);
        }

        // Update bandwidth ID
        if (bandwidths.keyExists(bandwidth)) {
            bwId = bandwidths.keyId(bandwidth);
        }
        else {
            bwId = 0;
            bandwidth = bandwidths.value(bwId);
        }
    }

    static void menuSelected(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        core::setInputSampleRate(_this->samplerate);
        flog::info("PlutoSDRSourceModule '{0}': Menu Select!", _this->name);
    }

    static void menuDeselected(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        flog::info("PlutoSDRSourceModule '{0}': Menu Deselect!", _this->name);
    }

    static void start(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        if (_this->running) { return; }

        // If no device is selected, give up
        if (_this->devDesc.empty() || _this->uri.empty()) { return; }

        // Open context
        _this->ctx = iio_create_context_from_uri(_this->uri.c_str());
        if (_this->ctx == NULL) {
            flog::error("Could not open pluto ({})", _this->uri);
            return;
        }

        // Get phy and device handle
        _this->phy = iio_context_find_device(_this->ctx, "ad9361-phy");
        if (_this->phy == NULL) {
            flog::error("Could not connect to pluto phy");
            iio_context_destroy(_this->ctx);
            return;
        }
        _this->dev = iio_context_find_device(_this->ctx, "cf-ad9361-lpc");
        if (_this->dev == NULL) {
            flog::error("Could not connect to pluto dev");
            iio_context_destroy(_this->ctx);
            return;
        }

        // Get RX channels
        _this->rxChan = iio_device_find_channel(_this->phy, "voltage0", false);
        _this->rxLO = iio_device_find_channel(_this->phy, "altvoltage0", true);

        // Enable RX LO and disable TX LO initially
        iio_channel_attr_write_bool(_this->rxLO, "powerdown", false);
        iio_channel* txLO = iio_device_find_channel(_this->phy, "altvoltage1", true);
        iio_channel_attr_write_bool(txLO, "powerdown", !_this->txEnabled);  // Enable TX LO if TX is enabled

        // Configure RX channel
        iio_channel_attr_write(_this->rxChan, "rf_port_select", "A_BALANCED");
        iio_channel_attr_write_longlong(_this->rxLO, "frequency", round(_this->freq));
        iio_channel_attr_write_bool(_this->rxChan, "filter_fir_en", true);
        iio_channel_attr_write_longlong(_this->rxChan, "sampling_frequency", round(_this->samplerate));
        iio_channel_attr_write_double(_this->rxChan, "hardwaregain", _this->gain);
        iio_channel_attr_write(_this->rxChan, "gain_control_mode", _this->gainModes.value(_this->gmId).c_str());
        _this->setBandwidth(_this->bandwidth);
        
        // Configure the ADC filters
        ad9361_set_bb_rate(_this->phy, round(_this->samplerate));

        // Configure TX if enabled
        if (_this->txEnabled) {
            _this->configureTX();
        }

        // Start worker thread
        _this->running = true;
        _this->workerThread = std::thread(worker, _this);
        flog::info("PlutoSDRSourceModule '{0}': Start!", _this->name);
    }

    static void stop(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        if (!_this->running) { return; }

        // Stop worker thread
        _this->running = false;
        _this->stream.stopWriter();
        _this->workerThread.join();
        _this->stream.clearWriteStop();

        // Stop TX if enabled
        if (_this->txEnabled) {
            _this->disableTX();
        }

        // Close device
        if (_this->ctx != NULL) {
            iio_context_destroy(_this->ctx);
            _this->ctx = NULL;
        }

        flog::info("PlutoSDRSourceModule '{0}': Stop!", _this->name);
    }

    static void tune(double freq, void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        _this->freq = freq;
        if (_this->running) {
            iio_channel_attr_write_longlong(_this->rxLO, "frequency", round(freq));
        }
        flog::info("PlutoSDRSourceModule '{0}': Tune: {1}!", _this->name, freq);
    }

    static void menuHandler(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;

        if (_this->running) { SmGui::BeginDisabled(); }
        
        // Device selection
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Combo("##plutosdr_dev_sel", &_this->devId, _this->devices.txt)) {
            _this->select(_this->devices.key(_this->devId));
            core::setInputSampleRate(_this->samplerate);
            config.acquire();
            config.conf["device"] = _this->devices.key(_this->devId);
            config.release(true);
        }

        // Custom IP input
        if (_this->devDesc == "ip_custom") {
            SmGui::LeftLabel("IP Address");
            SmGui::FillWidth();
            if (SmGui::InputText(CONCAT("##plutosdr_ip_addr_", _this->name), _this->ipBuf, 64)) {
                _this->customIP = std::string(_this->ipBuf);
                _this->uri = "ip:" + _this->customIP;
                config.acquire();
                config.conf["customIP"] = _this->customIP;
                config.release(true);
            }
        }

        // Sample rate selection
        SmGui::LeftLabel("Sample Rate");
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Combo(CONCAT("##_pluto_sr_", _this->name), &_this->srId, _this->samplerates.txt)) {
            _this->samplerate = _this->samplerates.value(_this->srId);
            core::setInputSampleRate(_this->samplerate);
            if (!_this->devDesc.empty() && _this->devDesc != "ip_custom") {
                config.acquire();
                config.conf["devices"][_this->devDesc]["samplerate"] = _this->samplerate;
                config.release(true);
            }
        }

        // Refresh button
        SmGui::SameLine();
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Button(CONCAT("Refresh##_pluto_refr_", _this->name))) {
            _this->refresh();
            _this->select(_this->devDesc);
            core::setInputSampleRate(_this->samplerate);
        }
        
        if (_this->running) { SmGui::EndDisabled(); }

        // Bandwidth selection
        SmGui::LeftLabel("Bandwidth");
        SmGui::FillWidth();
        if (SmGui::Combo(CONCAT("##_pluto_bw_", _this->name), &_this->bwId, _this->bandwidths.txt)) {
            _this->bandwidth = _this->bandwidths.value(_this->bwId);
            if (_this->running) {
                _this->setBandwidth(_this->bandwidth);
            }
            if (!_this->devDesc.empty() && _this->devDesc != "ip_custom") {
                config.acquire();
                config.conf["devices"][_this->devDesc]["bandwidth"] = _this->bandwidth;
                config.release(true);
            }
        }

        // Gain mode selection
        SmGui::LeftLabel("Gain Mode");
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Combo(CONCAT("##_pluto_gainmode_select_", _this->name), &_this->gmId, _this->gainModes.txt)) {
            if (_this->running) {
                iio_channel_attr_write(_this->rxChan, "gain_control_mode", _this->gainModes.value(_this->gmId).c_str());
            }
            if (!_this->devDesc.empty() && _this->devDesc != "ip_custom") {
                config.acquire();
                config.conf["devices"][_this->devDesc]["gainMode"] = _this->gainModes.key(_this->gmId);
                config.release(true);
            }
        }

        // Gain slider
        SmGui::LeftLabel("Gain");
        if (_this->gmId) { SmGui::BeginDisabled(); }
        SmGui::FillWidth();
        if (SmGui::SliderFloatWithSteps(CONCAT("##_pluto_gain__", _this->name), &_this->gain, -1.0f, 73.0f, 1.0f, SmGui::FMT_STR_FLOAT_DB_NO_DECIMAL)) {
            if (_this->running) {
                iio_channel_attr_write_double(_this->rxChan, "hardwaregain", _this->gain);
            }
            if (!_this->devDesc.empty() && _this->devDesc != "ip_custom") {
                config.acquire();
                config.conf["devices"][_this->devDesc]["gain"] = _this->gain;
                config.release(true);
            }
        }
        if (_this->gmId) { SmGui::EndDisabled(); }

        // TX Configuration Section
        ImGui::Separator();
        SmGui::LeftLabel("TX Configuration");
        SmGui::FillWidth();
        
        // TX Enable checkbox
        if (SmGui::Checkbox(CONCAT("Enable TX##_pluto_tx_enable_", _this->name), &_this->txEnabled)) {
            if (_this->running) {
                if (_this->txEnabled) {
                    _this->configureTX();
                } else {
                    _this->disableTX();
                }
            }
            config.acquire();
            config.conf["txEnabled"] = _this->txEnabled;
            config.release(true);
        }

        // TX Port selection
        SmGui::LeftLabel("TX Port");
        SmGui::FillWidth();
        if (SmGui::Combo(CONCAT("##_pluto_tx_port_", _this->name), &_this->txPortId, _this->txPorts.txt)) {
            if (_this->running && _this->txEnabled) {
                _this->configureTX();
            }
            config.acquire();
            config.conf["txPort"] = _this->txPorts.key(_this->txPortId);
            config.release(true);
        }

        // TX Frequency input
        SmGui::LeftLabel("TX Frequency (Hz)");
        SmGui::FillWidth();
        char txFreqBuf[64];
        sprintf(txFreqBuf, "%.0f", _this->txFreq);
        if (SmGui::InputText(CONCAT("##_pluto_tx_freq_", _this->name), txFreqBuf, 64, ImGuiInputTextFlags_CharsDecimal)) {
            try {
                double newFreq = std::stod(txFreqBuf);
                if (newFreq >= 325000000 && newFreq <= 3800000000) {  // AD9361 frequency range
                    _this->txFreq = newFreq;
                    if (_this->running && _this->txEnabled) {
                        iio_channel* txLO = iio_device_find_channel(_this->phy, "altvoltage1", true);
                        iio_channel_attr_write_longlong(txLO, "frequency", round(_this->txFreq));
                    }
                    config.acquire();
                    config.conf["txFreq"] = _this->txFreq;
                    config.release(true);
                }
            } catch (...) {
                // Ignore invalid input
            }
        }

        // TX Tone Frequency input
        SmGui::LeftLabel("TX Tone Frequency (Hz)");
        SmGui::FillWidth();
        char txToneFreqBuf[64];
        sprintf(txToneFreqBuf, "%.0f", _this->txToneFreq);
        if (SmGui::InputText(CONCAT("##_pluto_tx_tone_freq_", _this->name), txToneFreqBuf, 64, ImGuiInputTextFlags_CharsDecimal)) {
            try {
                double newToneFreq = std::stod(txToneFreqBuf);
                // 限制音调频率不超过采样率的一半
                if (newToneFreq >= -(_this->samplerate/2) && newToneFreq <= (_this->samplerate/2)) {
                    _this->txToneFreq = newToneFreq;
                    if (_this->running && _this->txEnabled && _this->txBuf) {
                        _this->updateTXTone();
                    }
                    config.acquire();
                    config.conf["txToneFreq"] = _this->txToneFreq;
                    config.release(true);
                }
            } catch (...) {
                // Ignore invalid input
            }
        }

        // TX Attenuation slider
        SmGui::LeftLabel("TX Attenuation (dB)");
        SmGui::FillWidth();
        if (SmGui::SliderFloatWithSteps(CONCAT("##_pluto_tx_atten_", _this->name), &_this->txAttenuation, 0.0f, 89.75f, 0.25f, SmGui::FMT_STR_FLOAT_DB_NO_DECIMAL)) {
            if (_this->running && _this->txEnabled) {
                iio_channel* txChan = iio_device_find_channel(_this->phy, "voltage0", true);
                iio_channel_attr_write_double(txChan, "hardwaregain", -_this->txAttenuation);
            }
            config.acquire();
            config.conf["txAttenuation"] = _this->txAttenuation;
            config.release(true);
        }
    }

    void configureTX() {
        if (!ctx || !phy) return;

        // Get TX LO channel
        iio_channel* txLO = iio_device_find_channel(phy, "altvoltage1", true);
        if (!txLO) {
            flog::error("Could not find TX LO channel");
            return;
        }

        // Enable TX LO
        iio_channel_attr_write_bool(txLO, "powerdown", false);
        
        // Set TX frequency
        iio_channel_attr_write_longlong(txLO, "frequency", round(txFreq));
        
        // Get TX channel
        iio_channel* txChan = iio_device_find_channel(phy, "voltage0", true);
        if (!txChan) {
            flog::error("Could not find TX channel");
            return;
        }
        
        // Set TX port
        std::string txPortStr = txPorts.value(txPortId);
        iio_channel_attr_write(txChan, "rf_port_select", txPortStr.c_str());
        
        // Set TX attenuation (negative gain)
        iio_channel_attr_write_double(txChan, "hardwaregain", -txAttenuation);
        
        // Enable TX channel
        iio_channel_attr_write_bool(txChan, "powerdown", false);
        
        // Set TX sampling frequency (same as RX)
        iio_channel_attr_write_longlong(txChan, "sampling_frequency", round(samplerate));
        
        // Configure TX filters
        iio_channel_attr_write_bool(txChan, "filter_fir_en", true);
        
        // 获取TX设备
        iio_device* txDev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
        if (!txDev) {
            flog::error("Could not find TX DDS device");
            return;
        }
        
        // 获取TX通道
        iio_channel* tx0_i = iio_device_find_channel(txDev, "voltage0", true);
        iio_channel* tx0_q = iio_device_find_channel(txDev, "voltage1", true);
        if (!tx0_i || !tx0_q) {
            flog::error("Failed to acquire TX DDS channels");
            return;
        }
        
        // 启用TX通道
        iio_channel_enable(tx0_i);
        iio_channel_enable(tx0_q);
        
        // 创建循环缓冲区
        const int bufferSize = 4096;  // 缓冲区大小
        txBuf = iio_device_create_buffer(txDev, bufferSize, true);  // true = cyclic buffer
        if (!txBuf) {
            flog::error("Could not create TX cyclic buffer");
            return;
        }
        
        // 初始化音调
        updateTXTone();
        
        flog::info("TX configured: Port={}, Frequency={} Hz, Attenuation={} dB, Tone={} Hz", 
                   txPortStr, txFreq, txAttenuation, txToneFreq);
    }
    
    void updateTXTone() {
    if (!txBuf) return;
    
    // 获取缓冲区指针
    int16_t* buf = (int16_t*)iio_buffer_start(txBuf);
    if (!buf) return;
    
    // 获取缓冲区大小（正确的方法）
    char* end = (char*)iio_buffer_end(txBuf);
    char* start = (char*)iio_buffer_start(txBuf);
    ptrdiff_t bufferSize = end - start;
    size_t numSamples = bufferSize / (sizeof(int16_t) * 2);  // 每个样本是I和Q两个16位整数
    
    // 生成单音信号
    float amplitude = 0.7f;  // 幅度（避免过载）
    float phase = 0.0f;
    float phaseIncrement = 2.0f * M_PI * txToneFreq / samplerate;
    
    for (size_t i = 0; i < numSamples; i++) {
        float i_sample = amplitude * cos(phase);
        float q_sample = amplitude * sin(phase);
        
        // 转换为16位整数（Q1.15格式）
        buf[i * 2] = (int16_t)(i_sample * 32767.0f);
        buf[i * 2 + 1] = (int16_t)(q_sample * 32767.0f);
        
        phase += phaseIncrement;
        if (phase > 2.0f * M_PI) phase -= 2.0f * M_PI;
    }
    
    // 提交缓冲区（对于循环缓冲区，只需要推送一次）
    ssize_t ret = iio_buffer_push(txBuf);
    if (ret < 0) {
        flog::error("Failed to push TX buffer: {}", ret);
    }
}
    
    void disableTX() {
        // 禁用TX LO
        iio_channel* txLO = iio_device_find_channel(phy, "altvoltage1", true);
        if (txLO) {
            iio_channel_attr_write_bool(txLO, "powerdown", true);
        }
        
        // 禁用TX通道
        iio_channel* txChan = iio_device_find_channel(phy, "voltage0", true);
        if (txChan) {
            iio_channel_attr_write_bool(txChan, "powerdown", true);
        }
        
        // 释放TX缓冲区
        if (txBuf) {
            iio_buffer_destroy(txBuf);
            txBuf = NULL;
        }
        
        flog::info("TX disabled");
    }

    void setBandwidth(int bw) {
        if (bw > 0) {
            iio_channel_attr_write_longlong(rxChan, "rf_bandwidth", bw);
        }
        else {
            iio_channel_attr_write_longlong(rxChan, "rf_bandwidth", std::min<int>(samplerate, 52000000));
        }
    }

    static void worker(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        int blockSize = _this->samplerate / 200.0f;

        // Acquire channels
        iio_channel* rx0_i = iio_device_find_channel(_this->dev, "voltage0", 0);
        iio_channel* rx0_q = iio_device_find_channel(_this->dev, "voltage1", 0);
        if (!rx0_i || !rx0_q) {
            flog::error("Failed to acquire RX channels");
            return;
        }

        // Start streaming
        iio_channel_enable(rx0_i);
        iio_channel_enable(rx0_q);

        // Allocate buffer
        iio_buffer* rxbuf = iio_device_create_buffer(_this->dev, blockSize, false);
        if (!rxbuf) {
            flog::error("Could not create RX buffer");
            return;
        }

        // Receive loop
        while (true) {
            // Read samples
            iio_buffer_refill(rxbuf);

            // Get buffer pointer
            int16_t* buf = (int16_t*)iio_buffer_first(rxbuf, rx0_i);
            if (!buf) { break; }

            // Convert samples to CF32
            volk_16i_s32f_convert_32f((float*)_this->stream.writeBuf, buf, 32768.0f, blockSize * 2);

            // Send out the samples
            if (!_this->stream.swap(blockSize)) { break; };
        }

        // Stop streaming
        iio_channel_disable(rx0_i);
        iio_channel_disable(rx0_q);

        // Free buffer
        iio_buffer_destroy(rxbuf);
    }

    std::string name;
    bool enabled = true;
    dsp::stream<dsp::complex_t> stream;
    SourceManager::SourceHandler handler;
    std::thread workerThread;
    iio_context* ctx = NULL;
    iio_device* phy = NULL;
    iio_device* dev = NULL;
    iio_channel* rxLO = NULL;
    iio_channel* rxChan = NULL;
    bool running = false;

    std::string devDesc = "";
    std::string uri = "";
    std::string customIP = "192.168.2.1";
    char ipBuf[64] = "192.168.2.1";

    // RX parameters
    double freq;
    int samplerate = 4000000;
    int bandwidth = 0;
    float gain = -1;

    // TX parameters
    bool txEnabled = false;
    int txPortId = 0;
    double txFreq = 1000000000.0;  // Default 1 GHz
    double txToneFreq = 1000.0;    // Default 1 kHz tone
    float txAttenuation = 0.0f;    // Default 0 dB attenuation (max power)
    iio_buffer* txBuf = NULL;      // TX循环缓冲区

    int devId = 0;
    int srId = 0;
    int bwId = 0;
    int gmId = 0;

    OptionList<std::string, std::string> devices;
    OptionList<int, double> samplerates;
    OptionList<int, double> bandwidths;
    OptionList<std::string, std::string> gainModes;
    OptionList<std::string, std::string> txPorts;
};

MOD_EXPORT void _INIT_() {
    json defConf = {};
    defConf["device"] = "";
    defConf["customIP"] = "192.168.2.1";
    defConf["txEnabled"] = false;
    defConf["txPort"] = "TX1";
    defConf["txFreq"] = 1075000000.0;
    defConf["txToneFreq"] = 0.0;
    defConf["txAttenuation"] = 15.0f;  // 改为0dB以获得最大功率
    defConf["devices"] = {};
    config.setPath(core::args["root"].s() + "/plutosdr_source_config.json");
    config.load(defConf);
    config.enableAutoSave();

    // Reset the configuration if the old format is still used
    config.acquire();
    if (!config.conf.contains("device") || !config.conf.contains("devices")) {
        config.conf = defConf;
        config.release(true);
    }
    else {
        config.release();
    }
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new PlutoSDRSourceModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(ModuleManager::Instance* instance) {
    delete (PlutoSDRSourceModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
