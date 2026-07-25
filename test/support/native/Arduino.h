#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <deque>
#include <string>

#define PROGMEM
#define F(value) reinterpret_cast<const __FlashStringHelper *>(value)
#define memcpy_P memcpy
#define strncmp_P strncmp

class __FlashStringHelper;

class Stream {
  public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual size_t write(uint8_t value) = 0;

  protected:
    ~Stream() {}
};

inline uint32_t &arduinoFakeMillisStorage() {
    static uint32_t value = 0;
    return value;
}

inline uint32_t millis() { return arduinoFakeMillisStorage(); }
inline void arduinoSetMillis(uint32_t value) {
    arduinoFakeMillisStorage() = value;
}

class HardwareSerial : public Stream {
  public:
    HardwareSerial() : baud_(0), writable_(255), receive_(), transmit_() {}

    void begin(unsigned long baud) { baud_ = baud; }
    int availableForWrite() const { return writable_; }
    size_t write(const uint8_t *data, size_t length) {
        transmit_.append(
            reinterpret_cast<const char *>(data), length
        );
        return length;
    }
    size_t write(uint8_t value) {
        transmit_.push_back(static_cast<char>(value));
        return 1;
    }
    size_t print(const __FlashStringHelper *value) {
        const char *text = reinterpret_cast<const char *>(value);
        transmit_.append(text);
        return strlen(text);
    }
    int available() {
        return static_cast<int>(receive_.size());
    }
    int read() {
        if (receive_.empty()) return -1;
        const uint8_t value = receive_.front();
        receive_.pop_front();
        return value;
    }

    void queueReceive(const char *text) {
        while (*text != '\0')
            receive_.push_back(static_cast<uint8_t>(*text++));
    }
    void queueReceive(const uint8_t *data, size_t length) {
        for (size_t index = 0; index < length; ++index)
            receive_.push_back(data[index]);
    }
    const std::string &transmit() const { return transmit_; }
    void clearTransmit() { transmit_.clear(); }
    unsigned long baud() const { return baud_; }

  private:
    unsigned long baud_;
    int writable_;
    std::deque<uint8_t> receive_;
    std::string transmit_;
};
