#pragma once
/*
 * EncoderPCNT — API PCNT A/B/Z (OM).
 * Uso activo en el proyecto: solo dual-speed T0 (raw counts en FeedStage).
 * Medición / set0 / reset / validate ya no van en el ciclo; reintegrar desde aquí.
 */
#include <Arduino.h>
#include <stdint.h>

#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

    class EncoderPCNT {
public:
    // Encoder de 2000 P/R. En X4 se cuentan 4 flancos por pulso.
    static constexpr uint32_t PULSES_PER_REV = 2000;
    static constexpr uint32_t COUNTS_PER_REV = PULSES_PER_REV * 4;  // 8000
    static_assert(COUNTS_PER_REV == 8000, "Se esperan 8000 cuentas por vuelta");
    enum class Direction : int8_t {
        REVERSE = -1,
        STOPPED = 0,
        FORWARD = 1
    };
    struct Pins {
        int pinA;
        int pinB;
        int pinZ;       // -1 para deshabilitar el índice Z
    };
    struct Config {
        Pins right = {18, 19, 21};
        Pins left  = {22, 23, 25};
        uint32_t glitchFilterNs = 1000;
        // Límites del contador PCNT de 16 bits (highLimit > 0, lowLimit < 0).
        int highLimit = 32767;
        int lowLimit  = -32768;
        uint8_t inputMode = INPUT_PULLUP;
        bool invertDirection = false;
        uint32_t speedWindowUs = 10000;
        uint32_t stopTimeoutUs = 100000;
    };
    struct Reading {
        int64_t count = 0;          // Cuentas relativas al cero lógico
        int64_t rawCount = 0;       // Cuentas acumuladas desde begin()/reset
        float speedCps = 0.0f;      // Cuentas por segundo
        Direction direction = Direction::STOPPED;
        uint32_t indexCount = 0;    // Pulsos Z desde begin()/reset
        bool ready = false;         // Unidad configurada y contando
    };
    EncoderPCNT() = default;
    ~EncoderPCNT() {
        end();
    }
    EncoderPCNT(const EncoderPCNT&) = delete;
    EncoderPCNT& operator=(const EncoderPCNT&) = delete;
    bool begin() {
        Config config;
        return begin(config);
    }
    bool begin(const Config& config) {
        if (_started) {
            end();
        }
        setError(ESP_OK);
        setRecoveryError(ESP_OK);
        if (!configValid(config)) {
            setError(ESP_ERR_INVALID_ARG);
            return false;
        }
        _config = config;
        _mutex = xSemaphoreCreateMutex();
        if (_mutex == nullptr) {
            setError(ESP_ERR_NO_MEM);
            return false;
        }
        if (!beginSide(_right, _config.right)) {
            end();
            return false;
        }
        if (!beginSide(_left, _config.left)) {
            end();
            return false;
        }
        _started = true;
        return true;
    }
    void end() {
        endSide(_right);
        endSide(_left);

        _started = false;

        if (_mutex != nullptr) {
            vSemaphoreDelete(_mutex);
            _mutex = nullptr;
        }
    }
    bool isReady() const {
        return _started && right().ready && left().ready;
    }
    esp_err_t lastError() const {
        portENTER_CRITICAL(&_errMux);
        const esp_err_t err = _lastError;
        portEXIT_CRITICAL(&_errMux);
        return err;
    }
    esp_err_t lastRecoveryError() const {
        portENTER_CRITICAL(&_errMux);
        const esp_err_t err = _recoveryError;
        portEXIT_CRITICAL(&_errMux);
        return err;
    }
    void update() {
        if (!_started || _mutex == nullptr) return;
        if (xSemaphoreTake(_mutex, portMAX_DELAY) != pdTRUE) return;
        updateSide(_right);
        updateSide(_left);
        xSemaphoreGive(_mutex);
    }
    Reading right() const {
        return makeReading(_right);
    }
    Reading left() const {
        return makeReading(_left);
    }
    void setZeroRight() {
        setZeroSide(_right);
    }
    void setZeroLeft() {
        setZeroSide(_left);
    }
    void setZeroAll() {
        setZeroRight();
        setZeroLeft();
    }
    bool resetRight() {
        return resetSide(_right);
    }
    bool resetLeft() {
        return resetSide(_left);
    }
    bool resetAll() {
        const bool rightOk = resetRight();
        const bool leftOk  = resetLeft();
        return rightOk && leftOk;
    }
private:
    struct SideState {
        pcnt_unit_handle_t unit = nullptr;
        pcnt_channel_handle_t channelA = nullptr;
        pcnt_channel_handle_t channelB = nullptr;
        Pins pins = {-1, -1, -1};
        volatile uint32_t indexCount = 0;
        mutable portMUX_TYPE indexMux = portMUX_INITIALIZER_UNLOCKED;
        int32_t lastDriverCount = 0;
        int64_t totalCount = 0;
        int64_t zeroOffset = 0;
        int64_t lastSpeedCount = 0;
        uint32_t lastUpdateUs = 0;
        uint32_t lastMoveUs = 0;
        float speedCps = 0.0f;
        Direction direction = Direction::STOPPED;
        bool firstUpdate = true;
        bool unitStarted = false;   // false si un reset no pudo reiniciar el conteo
        bool ready = false;
        mutable portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
        bool unitEnabled = false;
        bool zAttached = false;
    };
    Config _config;
    SideState _right;
    SideState _left;
    bool _started = false;
    SemaphoreHandle_t _mutex = nullptr;   // Serializa update() y reset*()
    esp_err_t _lastError = ESP_OK;
    esp_err_t _recoveryError = ESP_OK;
    mutable portMUX_TYPE _errMux = portMUX_INITIALIZER_UNLOCKED;
    static void ARDUINO_ISR_ATTR indexISR(void* arg) {
        SideState* side = static_cast<SideState*>(arg);
        if (side == nullptr) return;
        portENTER_CRITICAL_ISR(&side->indexMux);
        side->indexCount++;
        portEXIT_CRITICAL_ISR(&side->indexMux);
    }
    bool configValid(const Config& config) const {
        if (config.highLimit <= 0 ||
            config.lowLimit >= 0 ||
            config.highLimit > 32767 ||
            config.lowLimit < -32768) {
            return false;
        }

        if (config.speedWindowUs == 0) return false;

        if (config.inputMode != INPUT &&
            config.inputMode != INPUT_PULLUP &&
            config.inputMode != INPUT_PULLDOWN) {
            return false;
        }

        return true;
    }

    void setError(esp_err_t err) {
        portENTER_CRITICAL(&_errMux);
        _lastError = err;
        portEXIT_CRITICAL(&_errMux);
    }

    void setRecoveryError(esp_err_t err) {
        portENTER_CRITICAL(&_errMux);
        _recoveryError = err;
        portEXIT_CRITICAL(&_errMux);
    }
    bool check(esp_err_t err) {
        if (err != ESP_OK) {
            setError(err);
            return false;
        }
        return true;
    }

    bool beginSide(SideState& side, const Pins& pins) {
        side.pins = pins;

        pinMode(pins.pinA, _config.inputMode);
        pinMode(pins.pinB, _config.inputMode);

        if (pins.pinZ >= 0) {
            pinMode(pins.pinZ, _config.inputMode);
        }

        pcnt_unit_config_t unitConfig = {};
        unitConfig.high_limit = _config.highLimit;
        unitConfig.low_limit = _config.lowLimit;

        // ESP-IDF 5.x: bit en flags (acumula overflow del contador 16-bit).
        unitConfig.flags.accum_count = 1;

        if (!check(pcnt_new_unit(&unitConfig, &side.unit))) return false;

        // Con invertDirection se intercambian INCREASE y DECREASE.
        const pcnt_channel_edge_action_t inc =
            _config.invertDirection ? PCNT_CHANNEL_EDGE_ACTION_DECREASE
                                    : PCNT_CHANNEL_EDGE_ACTION_INCREASE;
        const pcnt_channel_edge_action_t dec =
            _config.invertDirection ? PCNT_CHANNEL_EDGE_ACTION_INCREASE
                                    : PCNT_CHANNEL_EDGE_ACTION_DECREASE;

        // Canal A: flancos de A, nivel de B.
        pcnt_chan_config_t channelAConfig = {};
        channelAConfig.edge_gpio_num = pins.pinA;
        channelAConfig.level_gpio_num = pins.pinB;

        if (!check(pcnt_new_channel(
                side.unit, &channelAConfig, &side.channelA))) {
            return false;
        }

        if (!check(pcnt_channel_set_edge_action(
                side.channelA, inc, dec))) {
            return false;
        }

        if (!check(pcnt_channel_set_level_action(
                side.channelA,
                PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                PCNT_CHANNEL_LEVEL_ACTION_INVERSE))) {
            return false;
        }

        // Canal B: flancos de B, nivel de A (completa la decodificación X4).
        pcnt_chan_config_t channelBConfig = {};
        channelBConfig.edge_gpio_num = pins.pinB;
        channelBConfig.level_gpio_num = pins.pinA;

        if (!check(pcnt_new_channel(
                side.unit, &channelBConfig, &side.channelB))) {
            return false;
        }

        if (!check(pcnt_channel_set_edge_action(
                side.channelB, dec, inc))) {
            return false;
        }

        if (!check(pcnt_channel_set_level_action(
                side.channelB,
                PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                PCNT_CHANNEL_LEVEL_ACTION_INVERSE))) {
            return false;
        }

        // El driver exige configurar el filtro antes de habilitar la unidad.
        if (_config.glitchFilterNs > 0) {
            pcnt_glitch_filter_config_t filterConfig = {};
            filterConfig.max_glitch_ns = _config.glitchFilterNs;

            if (!check(pcnt_unit_set_glitch_filter(
                    side.unit, &filterConfig))) {
                return false;
            }
        }

        // Con accum_count, el driver solo acumula el desbordamiento si hay
        // watch points en ambos límites.
        if (!check(pcnt_unit_add_watch_point(
                side.unit, _config.highLimit))) {
            return false;
        }

        if (!check(pcnt_unit_add_watch_point(
                side.unit, _config.lowLimit))) {
            return false;
        }

        if (!check(pcnt_unit_enable(side.unit))) return false;
        side.unitEnabled = true;

        if (!check(pcnt_unit_clear_count(side.unit))) return false;
        if (!check(pcnt_unit_start(side.unit))) return false;

        const uint32_t now = micros();

        portENTER_CRITICAL(&side.dataMux);
        side.lastDriverCount = 0;
        side.totalCount = 0;
        side.zeroOffset = 0;
        side.lastSpeedCount = 0;
        side.lastUpdateUs = now;
        side.lastMoveUs = now;
        side.speedCps = 0.0f;
        side.direction = Direction::STOPPED;
        side.firstUpdate = true;
        side.unitStarted = true;
        side.ready = true;
        portEXIT_CRITICAL(&side.dataMux);

        portENTER_CRITICAL(&side.indexMux);
        side.indexCount = 0;
        portEXIT_CRITICAL(&side.indexMux);

        if (pins.pinZ >= 0) {
            attachInterruptArg(
                pins.pinZ,
                indexISR,
                &side,
                FALLING
            );
            side.zAttached = true;
        }

        return true;
    }

    void endSide(SideState& side) {
        if (side.zAttached && side.pins.pinZ >= 0) {
            detachInterrupt(side.pins.pinZ);
        }
        side.zAttached = false;

        if (side.unit != nullptr && side.unitEnabled) {
            pcnt_unit_stop(side.unit);
            pcnt_unit_disable(side.unit);
        }
        side.unitEnabled = false;

        if (side.channelB != nullptr) {
            pcnt_del_channel(side.channelB);
            side.channelB = nullptr;
        }

        if (side.channelA != nullptr) {
            pcnt_del_channel(side.channelA);
            side.channelA = nullptr;
        }

        if (side.unit != nullptr) {
            pcnt_del_unit(side.unit);
            side.unit = nullptr;
        }

        side.unitStarted = false;
        side.ready = false;
        side.pins = {-1, -1, -1};
    }

    // Llamar con _mutex tomado.
    void updateSide(SideState& side) {
        if (!side.ready) return;

        int driverCount = 0;
        if (!check(pcnt_unit_get_count(side.unit, &driverCount))) return;

        const uint32_t now = micros();

        portENTER_CRITICAL(&side.dataMux);

        if (side.firstUpdate) {
            side.lastDriverCount = driverCount;
            side.totalCount = driverCount;
            side.lastSpeedCount = side.totalCount;
            side.lastUpdateUs = now;
            side.lastMoveUs = now;
            side.firstUpdate = false;
            portEXIT_CRITICAL(&side.dataMux);
            return;
        }

        // pcnt_unit_get_count() suma el acumulado del driver al registro de
        // hardware. La resta modular extiende un posible rollover de 32 bits
        // a int64_t, siempre que no ocurran 2^31 o más cuentas entre llamadas.
        const uint32_t modularDelta =
            static_cast<uint32_t>(driverCount) -
            static_cast<uint32_t>(side.lastDriverCount);

        const int64_t delta =
            (modularDelta <= 0x7FFFFFFFUL)
                ? static_cast<int64_t>(modularDelta)
                : static_cast<int64_t>(modularDelta) - 0x100000000LL;

        side.totalCount += delta;
        side.lastDriverCount = driverCount;

        // La velocidad se recalcula al completar la ventana mínima.
        const uint32_t elapsedUs = now - side.lastUpdateUs;

        if (elapsedUs >= _config.speedWindowUs) {
            const int64_t speedDelta =
                side.totalCount - side.lastSpeedCount;

            side.speedCps =
                static_cast<float>(speedDelta) *
                1000000.0f / static_cast<float>(elapsedUs);

            if (speedDelta > 0) {
                side.direction = Direction::FORWARD;
                side.lastMoveUs = now;
            } else if (speedDelta < 0) {
                side.direction = Direction::REVERSE;
                side.lastMoveUs = now;
            } else if ((now - side.lastMoveUs) >= _config.stopTimeoutUs) {
                // Se conserva la última dirección hasta pasar el timeout.
                side.direction = Direction::STOPPED;
                side.speedCps = 0.0f;
            }

            side.lastSpeedCount = side.totalCount;
            side.lastUpdateUs = now;
        }

        portEXIT_CRITICAL(&side.dataMux);
    }

    Reading makeReading(const SideState& side) const {
        Reading result;

        portENTER_CRITICAL(&side.dataMux);
        result.ready = side.ready && side.unitStarted;
        result.rawCount = side.totalCount;
        result.count = side.totalCount - side.zeroOffset;
        result.speedCps = side.speedCps;
        result.direction = side.direction;
        portEXIT_CRITICAL(&side.dataMux);

        portENTER_CRITICAL(&side.indexMux);
        result.indexCount = side.indexCount;
        portEXIT_CRITICAL(&side.indexMux);

        return result;
    }

    void setZeroSide(SideState& side) {
        portENTER_CRITICAL(&side.dataMux);
        if (side.ready) {
            side.zeroOffset = side.totalCount;
        }
        portEXIT_CRITICAL(&side.dataMux);
    }

    /*
     * Secuencia: stop -> clear_count -> start, serializada con update().
     *
     * - Si stop falla: no se modifica nada; se registra el error.
     * - Si clear_count falla: se registra ese error como lastError() y se
     *   intenta reiniciar el conteo (pcnt_unit_start). Los acumuladores
     *   software NO se tocan, porque el contador hardware conserva su valor.
     *   El resultado del reinicio queda en lastRecoveryError(). Los pulsos
     *   ocurridos mientras la unidad estuvo detenida se pierden, por lo que
     *   el conteo puede quedar ligeramente atrasado respecto a la posición
     *   real; reset*() devuelve false para que el llamador lo sepa.
     * - Si start falla tras un clear correcto: se registra el error y
     *   unitStarted=false; otro reset*() puede intentar recuperarlo.
     */
    bool resetSide(SideState& side) {
        if (!_started || _mutex == nullptr || !side.ready) return false;

        if (xSemaphoreTake(_mutex, portMAX_DELAY) != pdTRUE) return false;

        setRecoveryError(ESP_OK);

        bool ok = false;

        // Si un reset anterior falló al arrancar, la unidad ya está detenida.
        // En ese estado no se debe llamar stop() nuevamente: se intenta
        // directamente clear_count() y después start() para permitir recuperar.
        bool wasStarted;
        portENTER_CRITICAL(&side.dataMux);
        wasStarted = side.unitStarted;
        portEXIT_CRITICAL(&side.dataMux);

        esp_err_t stopErr = ESP_OK;
        if (wasStarted) {
            stopErr = pcnt_unit_stop(side.unit);
        }

        if (stopErr != ESP_OK) {
            setError(stopErr);
        } else {
            // Después de stop exitoso (o si ya estaba detenido), se marca
            // detenido antes de proceder, para que el estado sea consistente
            // si clear/start falla.
            portENTER_CRITICAL(&side.dataMux);
            side.unitStarted = false;
            portEXIT_CRITICAL(&side.dataMux);

            const esp_err_t clearErr = pcnt_unit_clear_count(side.unit);

            if (clearErr != ESP_OK) {
                // Se conserva el error original y se intenta recuperar.
                setError(clearErr);

                const esp_err_t recoveryErr = pcnt_unit_start(side.unit);
                setRecoveryError(recoveryErr);

                portENTER_CRITICAL(&side.dataMux);
                side.unitStarted = (recoveryErr == ESP_OK);
                portEXIT_CRITICAL(&side.dataMux);
            } else {
                const esp_err_t startErr = pcnt_unit_start(side.unit);
                const uint32_t now = micros();

                portENTER_CRITICAL(&side.dataMux);
                side.totalCount = 0;
                side.zeroOffset = 0;
                side.lastDriverCount = 0;
                side.lastSpeedCount = 0;
                side.lastUpdateUs = now;
                side.lastMoveUs = now;
                side.speedCps = 0.0f;
                side.direction = Direction::STOPPED;
                side.firstUpdate = true;
                side.unitStarted = (startErr == ESP_OK);
                portEXIT_CRITICAL(&side.dataMux);

                portENTER_CRITICAL(&side.indexMux);
                side.indexCount = 0;
                portEXIT_CRITICAL(&side.indexMux);

                if (startErr != ESP_OK) {
                    setError(startErr);
                } else {
                    ok = true;
                }
            }
        }

        xSemaphoreGive(_mutex);
        return ok;
    }
};
