/*
 * MadFlight IBus Library
 *
 * A modern C++ library for handling Flysky/Turnigy RC iBUS protocol
 * Designed specifically for MadFlight boards using MF_Serial
 *
 * MIT License
 *
 * Copyright (c) 2025 Atin M <atinm.dev@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef IBUS_H
#define IBUS_H

#include <inttypes.h>
#include <stddef.h>
#include <functional>

// Forward declaration for MF_Serial
class MF_Serial;

/**
 * @brief MadFlight IBus Library - Event-Driven Implementation
 *
 * This library provides a modern, event-driven interface for the iBUS protocol
 * using callback-based processing and different architectural patterns than
 * traditional implementations.
 *
 * The iBUS protocol is a half-duplex protocol that can handle up to 14 servo
 * channels and 10 sensors, operating at 115200 baud.
 */
class IBus {
public:
    // Sensor type definitions
    static const uint8_t SENSOR_INTERNAL_VOLTAGE = 0x00;  // Internal voltage (0.01V)
    static const uint8_t SENSOR_TEMPERATURE = 0x01;        // Temperature (0.1°C, 0 = -40°C)
    static const uint8_t SENSOR_RPM = 0x02;                // RPM
    static const uint8_t SENSOR_EXTERNAL_VOLTAGE = 0x03;   // External voltage (0.01V)
    static const uint8_t SENSOR_PRESSURE = 0x41;           // Pressure (Pa)
    static const uint8_t SENSOR_SERVO = 0xFD;              // Servo value

    // Configuration constants
    static const int8_t NO_TIMER = -1;                     // Disable automatic timer
    static const uint8_t MAX_CHANNELS = 14;                // Maximum servo channels
    static const uint8_t MAX_SENSORS = 10;                 // Maximum sensors
    static const uint16_t CHANNEL_MIN = 1000;              // Minimum channel value
    static const uint16_t CHANNEL_MAX = 2000;              // Maximum channel value
    static const uint16_t CHANNEL_CENTER = 1500;           // Center/neutral value

    // Callback function types
    using ChannelUpdateCallback = std::function<void(uint8_t channel, uint16_t value)>;
    using SensorRequestCallback = std::function<void(uint8_t sensor_id, uint8_t request_type)>;
    using ErrorCallback = std::function<void(uint8_t error_code)>;

    /**
     * @brief Constructor
     */
    IBus();

    /**
     * @brief Destructor
     */
    ~IBus();

    /**
     * @brief Initialize the IBus communication with event-driven processing
     *
     * @param serial Reference to MF_Serial port to use
     * @param timer_id Timer ID for automatic processing (use NO_TIMER to disable)
     * @param rx_pin RX pin number (optional, auto-detected on MadFlight)
     * @param tx_pin TX pin number (optional, auto-detected on MadFlight)
     */
    void begin(MF_Serial& serial, int8_t timer_id = 0, int8_t rx_pin = -1, int8_t tx_pin = -1);

    /**
     * @brief Set callback for channel updates (event-driven approach)
     *
     * @param callback Function to call when channel values change
     */
    void on_channel_update(ChannelUpdateCallback callback);

    /**
     * @brief Set callback for sensor requests (event-driven approach)
     *
     * @param callback Function to call when sensor data is requested
     */
    void on_sensor_request(SensorRequestCallback callback);

    /**
     * @brief Set callback for error conditions
     *
     * @param callback Function to call when errors occur
     */
    void on_error(ErrorCallback callback);

    /**
     * @brief Get current channel value (polling approach as alternative)
     *
     * @param channel Channel number (0-13)
     * @return Channel value (1000-2000) or 0 if invalid channel
     */
    uint16_t get_channel_value(uint8_t channel) const;

    /**
     * @brief Register a sensor for telemetry (different approach)
     *
     * @param sensor_id Sensor ID (1-10)
     * @param sensor_type Sensor type (use SENSOR_* constants)
     * @param data_size Data size in bytes (2 or 4)
     * @return true if registration successful
     */
    bool register_sensor(uint8_t sensor_id, uint8_t sensor_type, uint8_t data_size = 2);

    /**
     * @brief Update sensor data (different approach)
     *
     * @param sensor_id Sensor ID (1-10)
     * @param data Sensor data value
     * @return true if update successful
     */
    bool update_sensor_data(uint8_t sensor_id, int32_t data);

    /**
     * @brief Process incoming data (event-driven processing)
     */
    void process_events();

    /**
     * @brief Check if new data is available (polling alternative)
     *
     * @return true if new data received since last check
     */
    bool has_new_data() const;

    /**
     * @brief Get communication statistics
     *
     * @return Statistics structure with various counters
     */
    struct Statistics {
        uint32_t messages_received;
        uint32_t messages_processed;
        uint32_t sensor_requests;
        uint32_t sensor_responses;
        uint32_t checksum_errors;
        uint32_t protocol_errors;
    };

    Statistics get_statistics() const;

    /**
     * @brief Reset all statistics
     */
    void reset_statistics();

    /**
     * @brief Get the number of active sensors
     *
     * @return Number of registered sensors
     */
    uint8_t get_active_sensor_count() const;

private:
    // Protocol constants - standard iBUS protocol (matching IBusBM)
    static const uint8_t IBUS_FRAME_SIZE = 32;
    static const uint8_t IBUS_HEADER_SIZE = 4;
    static const uint8_t IBUS_OVERHEAD = 3; // packet is <len><cmd><data....><chk_low><chk_high>, overhead=cmd+chk bytes
    static const uint8_t IBUS_TIMEGAP = 3; // Packets are received very ~7ms so use ~half that for the gap
    static const uint8_t IBUS_CMD_SERVO = 0x40;        // Command to set servo or motor speed is always 0x40
    static const uint8_t IBUS_CMD_SENSOR_DISCOVER = 0x80; // Command discover sensor (lowest 4 bits are sensor)
    static const uint8_t IBUS_CMD_SENSOR_TYPE = 0x90;     // Command discover sensor (lowest 4 bits are sensor)
    static const uint8_t IBUS_CMD_SENSOR_DATA = 0xA0;    // Command send sensor data (lowest 4 bits are sensor)

    // Different parsing approach - using a parser class
    class MessageParser {
    public:
        enum class ParseResult {
            INCOMPLETE,
            COMPLETE,
            ERROR
        };

        enum class State {
            GET_LENGTH,
            GET_DATA,
            GET_CHKSUM_LOW,
            GET_CHKSUM_HIGH,
            DISCARD
        };

        ParseResult parse_byte(uint8_t byte);
        void reset();
        bool is_valid() const;
        uint8_t get_command() const;
        uint8_t get_data_length() const;
        const uint8_t* get_data() const;

    private:
        uint8_t m_frame_buffer[IBUS_FRAME_SIZE];
        uint8_t m_position;
        uint8_t m_expected_length;
        uint16_t m_calculated_checksum;
        uint16_t m_received_checksum;
        bool m_checksum_valid;
        State m_state;
    };

    // Different sensor management approach
    struct SensorEntry {
        uint8_t type;
        uint8_t data_size;
        int32_t current_value;
        bool is_registered;
        uint32_t last_update_time;

        SensorEntry() : type(0), data_size(2), current_value(0),
                       is_registered(false), last_update_time(0) {}
    };

    // Member variables with different organization
    MF_Serial* m_serial_interface;
    MessageParser m_parser;
    uint16_t m_channel_values[MAX_CHANNELS];
    SensorEntry m_sensor_registry[MAX_SENSORS];
    uint8_t m_registered_sensors;
    bool m_data_available_flag;
    uint32_t m_last_activity_time;
    Statistics m_stats;

    // Callback functions
    ChannelUpdateCallback m_channel_callback;
    SensorRequestCallback m_sensor_callback;
    ErrorCallback m_error_callback;

    // Different processing methods
    void process_complete_message();
    void handle_servo_message(const uint8_t* data, uint8_t length);
    void handle_sensor_message(const uint8_t* data, uint8_t length);
    void send_sensor_reply(uint8_t sensor_id, uint8_t command_type);
    void trigger_error(uint8_t error_code);
    bool validate_channel_index(uint8_t channel) const;
    bool validate_sensor_id(uint8_t sensor_id) const;

    // Static instance management (different approach)
    static IBus* m_primary_instance;
    static void timer_interrupt_handler();
};

#endif // IBUS_H
