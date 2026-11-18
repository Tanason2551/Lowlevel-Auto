#pragma once

#include <stddef.h>
#include <stdint.h>
#include "serial_frame.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

/**
 * @brief Sliding-window binary frame parser for ROS1 FRAME_HIGH_2_LOW packets.
 *
 * Searches incoming byte streams for the 71-byte frame delimited by
 * header {'#', 'f'} and footer {'\r', '\n'}, storing the most recent valid frame.
 */
class H2LParser
{
public:
    static constexpr size_t BUFFER_CAPACITY = 256;
    static constexpr size_t FRAME_SIZE = sizeof(FRAME_HIGH_2_LOW);

    /**
     * @brief Feed a chunk of received bytes into the parser.
     * @return true if at least one valid frame was decoded from this chunk.
     */
    bool feedBytes(const uint8_t *data, size_t len, uint32_t nowMs);

    bool hasFrame() const { return hasFrame_; }
    const FRAME_HIGH_2_LOW &frame() const { return latestFrame_; }
    uint32_t frameCount() const { return frameCount_; }
    uint32_t droppedBytes() const { return droppedBytes_; }
    uint32_t lastRxMs() const { return lastRxMs_; }

    /// Check whether a valid frame has arrived within `timeoutMs`.
    bool isLinkAlive(uint32_t nowMs, uint32_t timeoutMs) const
    {
        return hasFrame_ && ((nowMs - lastRxMs_) <= timeoutMs);
    }

    /// Unscale robot pose from milli-units into (meters, meters, radians).
    void robotPose(float &xM, float &yM, float &yawRad) const
    {
        xM = static_cast<float>(latestFrame_.robot_pos_x) * 0.001f;
        yM = static_cast<float>(latestFrame_.robot_pos_y) * 0.001f;
        yawRad = static_cast<float>(latestFrame_.robot_pos_yaw) * 0.001f;
    }

private:
    bool parseBuffer(uint32_t nowMs);

    uint8_t buf_[BUFFER_CAPACITY]{};
    size_t len_ = 0;

    FRAME_HIGH_2_LOW latestFrame_{};
    bool hasFrame_ = false;
    uint32_t frameCount_ = 0;
    uint32_t droppedBytes_ = 0;
    uint32_t lastRxMs_ = 0;
};

#ifdef ARDUINO
/**
 * @brief Teensy HardwareSerial receiver and formatter for ROS1 FRAME_HIGH_2_LOW.
 */
class Ros1SerialRx
{
public:
    static constexpr size_t EXTRA_RX_BUFFER_BYTES = 512;
    static constexpr size_t MAX_BYTES_PER_UPDATE = 256;

    /**
     * @brief Initialize the hardware serial port and expand its RX ring buffer.
     *
     * Teensy 4.1 HardwareSerial1 defaults to a 64-byte RX buffer, which is smaller
     * than the 71-byte FRAME_HIGH_2_LOW packet; extra memory prevents bursts from overflowing.
     */
    void begin(HardwareSerialIMXRT &port, uint32_t baud = 1000000);

    /**
     * @brief Drain available bytes from the UART without blocking.
     * @return true if at least one new valid frame arrived during this call.
     */
    bool update();

    bool hasFrame() const { return parser_.hasFrame(); }
    const FRAME_HIGH_2_LOW &frame() const { return parser_.frame(); }
    uint32_t frameCount() const { return parser_.frameCount(); }
    uint32_t droppedBytes() const { return parser_.droppedBytes(); }
    uint32_t lastRxMs() const { return parser_.lastRxMs(); }

    bool isLinkAlive(uint32_t nowMs, uint32_t timeoutMs) const
    {
        return parser_.isLinkAlive(nowMs, timeoutMs);
    }

    void robotPose(float &xM, float &yM, float &yawRad) const
    {
        parser_.robotPose(xM, yM, yawRad);
    }

    /**
     * @brief Print all decoded and unscaled fields of the latest frame to out.
     */
    void printSummary(Print &out) const;

private:
    HardwareSerialIMXRT *port_ = nullptr;
    uint8_t rxStorage_[EXTRA_RX_BUFFER_BYTES]{};
    H2LParser parser_;
};
#endif
