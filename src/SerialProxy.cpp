#include "SerialProxy.h"

SerialProxy::SerialProxy(Stream& port) : _serial(port) {
  _state = WAIT_START;
  _currentPtr = nullptr;
  _targetSize = 0;
  _byteCount = 0;
  _currentIndex = -1;
  _registryCount = 0;
}

void SerialProxy::begin() {
  _state = WAIT_START;
}

bool SerialProxy::attachPacket(char id, uint8_t* address, int size) {
  if (_registryCount >= MAX_REGISTERED_PACKETS) {
    return false;
  }
  
  _registry[_registryCount].id = id;
  _registry[_registryCount].address = address;
  _registry[_registryCount].size = size;
  _registry[_registryCount].isReady = false;
  
  _registryCount++;
  return true;
}

bool SerialProxy::isPacketReady(char id) {
  for (int i = 0; i < _registryCount; i++) {
    if (_registry[i].id == id) {
      if (_registry[i].isReady) {
        _registry[i].isReady = false;
        return true;
      }
      return false;
    }
  }
  return false;
}

uint8_t SerialProxy::calculateChecksum(uint8_t* buffer, int length) {
  uint8_t sum = 0;
  for (int i = 1; i < length - 2; i++) {
    sum += buffer[i];
  }
  return sum;
}

void SerialProxy::update() {
  while (_serial.available() > 0) {
    uint8_t inByte = _serial.read();

    switch (_state) {
      case WAIT_START:
        if (inByte == '#') {
          _state = READ_ID; 
        }
        break;

      case READ_ID:
        {
          bool idFound = false;
          for (int i = 0; i < _registryCount; i++) {
            if (inByte == _registry[i].id) {
              _currentPtr = _registry[i].address;
              _targetSize = _registry[i].size;
              _currentIndex = i;
              idFound = true;
              break;
            }
          }

          if (idFound) {
            _currentPtr[0] = '#';
            _currentPtr[1] = inByte;
            _byteCount = 2; 
            _state = READ_DATA;
          } else {
            _state = WAIT_START;
          }
        }
        break;

      case READ_DATA:
        _currentPtr[_byteCount] = inByte;
        _byteCount++;
        
        if (_byteCount >= _targetSize) {
          if (_currentPtr[_targetSize - 1] == '\n') {
            uint8_t calcSum = calculateChecksum(_currentPtr, _targetSize);
            uint8_t rxSum = _currentPtr[_targetSize - 2];
            
            if (calcSum == rxSum) {
              _registry[_currentIndex].isReady = true;
            } else {
              Serial.println("[ERROR] Checksum error detected in SerialProxy!");
            }
          }
          _state = WAIT_START;
        }
        break;
    }
  }
}
