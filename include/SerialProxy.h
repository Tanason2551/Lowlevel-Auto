#ifndef SERIALPROXY_H
#define SERIALPROXY_H

#include <Arduino.h>
#include <Stream.h>

#define MAX_REGISTERED_PACKETS 10

enum DecodeState { WAIT_START, READ_ID, READ_DATA };

struct PacketRecord {
  char id;
  uint8_t* address;
  int size;
  bool isReady;
};

class SerialProxy {
public:
  SerialProxy(Stream& port);
  void begin();
  
  // ลงทะเบียน Struct
  bool attachPacket(char id, uint8_t* address, int size);
  
  // ฟังก์ชันรับข้อมูล
  void update();
  
  // เช็คว่า ID ที่ต้องการ มีข้อมูลใหม่เข้ามาและพร้อมใช้หรือยัง
  bool isPacketReady(char id);

  template <typename T>
  void sendFast(T& packet) {
    uint8_t* ptr = (uint8_t*)&packet;
    ptr[0] = '#'; 
    ptr[sizeof(T) - 1] = '\n'; 
    uint8_t sum = 0;
    for (size_t i = 1; i < sizeof(T) - 2; i++) {
      sum += ptr[i];
    }
    ptr[sizeof(T) - 2] = sum; 

    _serial.write(ptr, sizeof(T));
  }

private:
  Stream& _serial;
  
  DecodeState _state;
  uint8_t* _currentPtr;
  int _targetSize;
  int _byteCount;
  int _currentIndex;
  
  PacketRecord _registry[MAX_REGISTERED_PACKETS];
  int _registryCount;

  uint8_t calculateChecksum(uint8_t* buffer, int length);
};

#endif
