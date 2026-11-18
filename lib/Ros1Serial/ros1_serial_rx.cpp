#include "ros1_serial_rx.h"
#include <string.h>

namespace
{
constexpr uint8_t H2L_HEADER_0 = '#';
constexpr uint8_t H2L_HEADER_1 = 'f';
constexpr uint8_t H2L_FOOTER_0 = '\r';
constexpr uint8_t H2L_FOOTER_1 = '\n';

inline float unscaleMilli(int16_t raw)
{
    return static_cast<float>(raw) * 0.001f;
}

inline uint8_t clampActionCount(uint8_t count)
{
    return (count <= FRAME_ACTION_MAX_COUNT) ? count : FRAME_ACTION_MAX_COUNT;
}
} // namespace

bool H2LParser::feedBytes(const uint8_t *data, size_t len, uint32_t nowMs)
{
    if (data == nullptr || len == 0)
    {
        return false;
    }

    bool gotNewFrame = false;
    size_t offset = 0;

    while (offset < len)
    {
        size_t freeSpace = BUFFER_CAPACITY - len_;
        if (freeSpace == 0)
        {
            // parseBuffer() guarantees len_ < FRAME_SIZE (< BUFFER_CAPACITY),
            // so reaching 0 freeSpace cannot happen in normal operation; guard defensively.
            droppedBytes_ += len_;
            len_ = 0;
            freeSpace = BUFFER_CAPACITY;
        }

        size_t chunk = len - offset;
        if (chunk > freeSpace)
        {
            chunk = freeSpace;
        }

        memcpy(&buf_[len_], &data[offset], chunk);
        len_ += chunk;
        offset += chunk;

        if (parseBuffer(nowMs))
        {
            gotNewFrame = true;
        }
    }

    return gotNewFrame;
}

bool H2LParser::parseBuffer(uint32_t nowMs)
{
    bool gotNewFrame = false;
    size_t i = 0;

    while (len_ - i >= FRAME_SIZE)
    {
        const bool headerMatch = (buf_[i] == H2L_HEADER_0 && buf_[i + 1] == H2L_HEADER_1);
        const bool footerMatch = (buf_[i + FRAME_SIZE - 2] == H2L_FOOTER_0 &&
                                  buf_[i + FRAME_SIZE - 1] == H2L_FOOTER_1);

        if (headerMatch && footerMatch)
        {
            memcpy(&latestFrame_, &buf_[i], FRAME_SIZE);
            frameCount_++;
            lastRxMs_ = nowMs;
            hasFrame_ = true;
            gotNewFrame = true;
            i += FRAME_SIZE;
        }
        else
        {
            i++;
            droppedBytes_++;
        }
    }

    if (i > 0)
    {
        const size_t remaining = len_ - i;
        if (remaining > 0)
        {
            memmove(buf_, &buf_[i], remaining);
        }
        len_ = remaining;
    }

    return gotNewFrame;
}

#ifdef ARDUINO

namespace
{
void printU8Array(Print &out, const uint8_t *arr, size_t count)
{
    out.print('[');
    for (size_t i = 0; i < count; i++)
    {
        if (i > 0)
        {
            out.print(", ");
        }
        out.print(arr[i]);
    }
    out.print(']');
}
} // namespace

void Ros1SerialRx::begin(HardwareSerialIMXRT &port, uint32_t baud)
{
    port_ = &port;
    port_->addMemoryForRead(rxStorage_, sizeof(rxStorage_));
    port_->begin(baud);
}

bool Ros1SerialRx::update()
{
    if (port_ == nullptr)
    {
        return false;
    }

    bool gotNewFrame = false;
    uint8_t chunk[128];
    size_t totalRead = 0;
    const uint32_t nowMs = millis();

    while (port_->available() > 0 && totalRead < MAX_BYTES_PER_UPDATE)
    {
        size_t count = 0;
        const size_t remainingBudget = MAX_BYTES_PER_UPDATE - totalRead;
        const size_t maxChunk = (remainingBudget < sizeof(chunk)) ? remainingBudget : sizeof(chunk);
        while (port_->available() > 0 && count < maxChunk)
        {
            int c = port_->read();
            if (c < 0)
            {
                break;
            }
            chunk[count++] = static_cast<uint8_t>(c);
        }

        totalRead += count;
        if (count > 0 && parser_.feedBytes(chunk, count, nowMs))
        {
            gotNewFrame = true;
        }
    }

    return gotNewFrame;
}

void Ros1SerialRx::printSummary(Print &out) const
{
    const FRAME_HIGH_2_LOW &f = parser_.frame();
    const uint32_t ageMs = millis() - parser_.lastRxMs();

    out.println(F("========== ROS1 FRAME_HIGH_2_LOW =========="));
    out.printf("Stats      : frames=%lu, age=%lu ms, dropped=%lu B\r\n",
               static_cast<unsigned long>(parser_.frameCount()),
               static_cast<unsigned long>(ageMs),
               static_cast<unsigned long>(parser_.droppedBytes()));

    out.printf("Robot Pose : x=%.3f m, y=%.3f m, yaw=%.3f rad\r\n",
               unscaleMilli(f.robot_pos_x),
               unscaleMilli(f.robot_pos_y),
               unscaleMilli(f.robot_pos_yaw));

    out.printf("Forest Err : x=%.3f, y=%.3f, yaw=%.3f\r\n",
               unscaleMilli(f.forest_error_x),
               unscaleMilli(f.forest_error_y),
               unscaleMilli(f.forest_error_yaw));

    out.printf("Spearhead  : pos=(%.3f, %.3f), order=%u, lift=%.3f, flipper=%.3f, offset=(%.3f, %.3f)\r\n",
               unscaleMilli(f.sph_pos_x),
               unscaleMilli(f.sph_pos_y),
               f.sph_order,
               unscaleMilli(f.sph_lift_level),
               unscaleMilli(f.sph_flipper_angle),
               unscaleMilli(f.sph_offset_x),
               unscaleMilli(f.sph_offset_y));

    out.printf("KFS / Cab  : kfs_err=(%.3f, %.3f, %.3f), cab_err=(x=%.3f, yaw=%.3f)\r\n",
               unscaleMilli(f.kfs_error_x),
               unscaleMilli(f.kfs_error_y),
               unscaleMilli(f.kfs_error_yaw),
               unscaleMilli(f.cab_error_x),
               unscaleMilli(f.cab_error_yaw));

    out.printf("States     : state_1=0x%02X (hb=%u, sph=%u, ms200=%u), state_2=0x%02X (scan=[%u,%u,%u])\r\n",
               f.state_1,
               (f.state_1 >> 0) & 0x01,
               (f.state_1 >> 1) & 0x01,
               (f.state_1 >> 2) & 0x01,
               f.state_2,
               (f.state_2 >> 0) & 0x01,
               (f.state_2 >> 1) & 0x01,
               (f.state_2 >> 2) & 0x01);

    out.printf("Quantities : sph_qty=%u, kfs_qty=%u | sph_prio=", f.sph_quantity, f.kfs_quantity);
    printU8Array(out, f.sph_priority, sizeof(f.sph_priority));
    out.print(F(" | ttt_prio="));
    printU8Array(out, f.ttt_priority, sizeof(f.ttt_priority));
    out.println();

    const uint8_t actionCount = clampActionCount(f.action_count);
    out.printf("Action Seq : count=%u (raw=%u)", actionCount, f.action_count);
    for (uint8_t i = 0; i < actionCount; i++)
    {
        const uint8_t action = (f.action_mask[i / 8] >> (i % 8)) & 0x01;
        out.printf(" | [%u:%s:%d]", i, action ? "pick" : "move", f.slot_data[i]);
    }
    out.println();
}

#endif
