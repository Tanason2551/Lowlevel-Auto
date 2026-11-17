#ifndef PACKETS_H
#define PACKETS_H

#include <stdint.h>


struct StereoCameraPacket {
    char startByte;     // '#' (0x23)
    char id;            // 'C'
    float x;            // พิกัดแกน X (เมตร)
    float y;            // พิกัดแกน Y (เมตร)
    float yaw;          // มุม Yaw / Heading (rad หรือ deg)
    uint8_t checksum;  
    char stopByte;      // '\n' (0x0A)
} __attribute__((packed));


struct LidarPacket {
    char startByte;     // '#' (0x23)
    char id;            // 'L'
    float x;            // พิกัดแกน X (เมตร)
    float y;            // พิกัดแกน Y (เมตร)
    float yaw;          // มุม Yaw / Heading (rad หรือ deg)
    uint8_t checksum;   
    char stopByte;      // '\n' (0x0A)
} __attribute__((packed));


struct WebcamPacket {
    char startByte;     // '#' (0x23)
    char id;            // 'W'
    float x;            // พิกัดแกน X (เมตร)
    float y;            // พิกัดแกน Y (เมตร)
    float yaw;          // มุม Yaw / Heading (rad หรือ deg)
    uint8_t checksum;   
    char stopByte;      // '\n' (0x0A)
} __attribute__((packed));


struct McuStagePacket {
    char startByte;                 // '#' (0x23)
    char id;                        // 'S'
    int16_t main_stage;             // Stage ภาพรวมของระบบ MCU
    int16_t stereo_camera_stage;    // Stage สำหรับ Stereo Camera localization
    int16_t lidar_stage;            // Stage สำหรับ LiDAR localization
    int16_t webcam_stage;           // Stage สำหรับ Webcam localization
    uint8_t checksum;               
    char stopByte;                  // '\n' (0x0A)
} __attribute__((packed));


struct GoalPacket {
    char startByte;     // '#' (0x23)
    char id;            // 'G'
    float x;            // เป้าหมายแกน X (เมตร)
    float y;            // เป้าหมายแกน Y (เมตร)
    float yaw;          // เป้าหมายมุม Yaw (degrees)
    uint8_t enable;     // 1 = อนุญาตให้เคลื่อนที่, 0 = หยุด/Hold
    uint8_t checksum;
    char stopByte;      // '\n' (0x0A)
} __attribute__((packed));

#endif
