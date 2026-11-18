#pragma once
#include <stdint.h>

constexpr uint8_t FRAME_ACTION_MAX_COUNT = 16;

constexpr uint8_t FRAME_LOW_2_HIGH_HEADER[2] = {'#', 'f'};

struct __attribute__((packed)) FRAME_LOW_2_HIGH
{
    uint8_t header[2];

    uint8_t forest_up_state;
    uint8_t forest_down_state;
    uint8_t seq_forest_state;
    uint8_t seq_sph_state;
    uint8_t seq_xo_state;

    // velocity
    int16_t vx;
    int16_t vy;
    int16_t vr;

    // position feedback
    int16_t lift1_a;
    int16_t lift1_b;
    int16_t lift2_a;
    int16_t lift2_b;
    int16_t linear_rail;
    int16_t sph_gripper;
    int16_t kfs_gripper;
    int16_t base_gripper;

    uint8_t state_1;
    uint8_t state_2;

    uint8_t footer[2];
};
static_assert(sizeof(FRAME_LOW_2_HIGH) == 33, "FRAME_LOW_2_HIGH size error check version");

struct __attribute__((packed)) FRAME_HIGH_2_LOW
{
#ifdef ARDUINO
    uint8_t header[2];
#else
    uint8_t header[2] = {'#', 'f'};
#endif

    int16_t robot_pos_x;
    int16_t robot_pos_y;
    int16_t robot_pos_yaw;

    int16_t forest_error_x;
    int16_t forest_error_y;
    int16_t forest_error_yaw;

    int16_t sph_pos_x;
    int16_t sph_pos_y;
    uint8_t sph_order;

    int16_t kfs_error_x;
    int16_t kfs_error_y;
    int16_t kfs_error_yaw;

    int16_t cab_error_x;
    int16_t cab_error_yaw;

    uint8_t state_1;
    uint8_t state_2;

    uint8_t sph_priority[6];
    uint8_t sph_quantity;
    uint8_t kfs_quantity;
    uint8_t ttt_priority[3];

    int16_t sph_lift_level;
    int16_t sph_flipper_angle;

    int16_t sph_offset_x;
    int16_t sph_offset_y;

    // forest action sequence
    uint8_t action_count;                     // จำนวน pair (max 16)
    uint8_t action_mask[2];                   // bit i = action ของ pair i (0=move, 1=pick)
    int8_t slot_data[FRAME_ACTION_MAX_COUNT]; // slot ของ pair i (1–30, -1 =หยุด)

#ifdef ARDUINO
    uint8_t footer[2];
#else
    uint8_t footer[2] = {'\r', '\n'};
#endif
};
static_assert(sizeof(FRAME_HIGH_2_LOW) == 71, "FRAME_HIGH_2_LOW size error check version");

#ifdef ARDUINO

union H2L_Frame
{
    FRAME_HIGH_2_LOW data;
    uint8_t bytes[sizeof(FRAME_HIGH_2_LOW)];
};

union L2H_Frame
{
    FRAME_LOW_2_HIGH data;
    uint8_t bytes[sizeof(FRAME_LOW_2_HIGH)];
};

#define H2L_FRAME_SIZE sizeof(FRAME_HIGH_2_LOW)
#define L2H_FRAME_SIZE sizeof(FRAME_LOW_2_HIGH)

#endif
