#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

class MyCobot
{
private:
  int fd = -1;

  // Protocol Header and Footer
  const uint8_t HEADER = 0xFE;
  const uint8_t FOOTER = 0xFA;

  // Official Command IDs
  const uint8_t CMD_POWER_ON = 0x10;
  const uint8_t CMD_POWER_OFF = 0x11;
  const uint8_t CMD_RELEASE_ALL_SERVOS = 0x13;
  const uint8_t CMD_GET_ANGLES = 0x20;
  const uint8_t CMD_SEND_ANGLES = 0x22;
  const uint8_t CMD_SEND_ANGLE = 0x21;
  const uint8_t CMD_SET_COLOR = 0x6A;

  bool write_bytes(const std::vector<uint8_t> & data)
  {
    if (fd < 0) return false;
    ssize_t written = write(fd, data.data(), data.size());
    return written == static_cast<ssize_t>(data.size());
  }

  int read_byte(uint8_t & byte, int timeout_ms = 100)
  {
    auto start = std::chrono::steady_clock::now();
    while (true) {
      int res = read(fd, &byte, 1);
      if (res > 0) return 1;
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - start)
                       .count();
      if (elapsed >= timeout_ms) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;  // Timeout
  }

public:
  MyCobot() = default;
  ~MyCobot() { disconnect(); }

  bool connect(const std::string & port, speed_t baud = B1000000)
  {
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
      std::cerr << "Failed to open port: " << port << std::endl;
      return false;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
      close(fd);
      fd = -1;
      return false;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    // 8N1 configuration (8 data bits, no parity, 1 stop bit)
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    // Raw mode (disable character processing)
    tty.c_iflag &=
      ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;  // 100ms timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      close(fd);
      fd = -1;
      return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
  }

  void disconnect()
  {
    if (fd >= 0) {
      close(fd);
      fd = -1;
    }
  }

  bool is_connected() const { return fd >= 0; }

  // Power on and engage servo torque
  void power_on()
  {
    std::vector<uint8_t> frame = {HEADER, HEADER, 3, CMD_POWER_ON, 1, FOOTER};
    write_bytes(frame);
  }

  // Set the Atom Matrix LED color (RGB, 0-255 each)
  void set_color(uint8_t r, uint8_t g, uint8_t b)
  {
    std::vector<uint8_t> frame = {HEADER, HEADER, 5, CMD_SET_COLOR, r, g, b, FOOTER};
    write_bytes(frame);
  }

  // Release all servos (disengage holding torque)
  void release_all_servos()
  {
    std::vector<uint8_t> frame = {HEADER, HEADER, 2, CMD_RELEASE_ALL_SERVOS, FOOTER};
    write_bytes(frame);
  }

  // Send target angles for all 6 joints (in degrees)
  void send_angles(const std::vector<float> & angles, int speed)
  {
    if (angles.size() < 6) return;

    // Protocol packet: [0xFE, 0xFE, LEN=15, 0x22, J1_H, J1_L ... J6_H, J6_L, SPEED, 0xFA]
    std::vector<uint8_t> frame = {HEADER, HEADER, 15, CMD_SEND_ANGLES};

    for (int i = 0; i < 6; ++i) {
      int16_t raw_val = static_cast<int16_t>(angles[i] * 100.0f);
      frame.push_back(static_cast<uint8_t>((raw_val >> 8) & 0xFF));
      frame.push_back(static_cast<uint8_t>(raw_val & 0xFF));
    }

    frame.push_back(static_cast<uint8_t>(speed & 0xFF));
    frame.push_back(FOOTER);

    write_bytes(frame);
  }

  // Send target angle to a single joint (joint_id: 1 to 6)
  void send_angle(int joint_id, float angle, int speed)
  {
    if (joint_id < 1 || joint_id > 6) return;

    // Protocol packet: [0xFE, 0xFE, LEN=6, 0x21, JOINT, VAL_H, VAL_L, SPEED, 0xFA]
    int16_t raw_val = static_cast<int16_t>(angle * 100.0f);
    std::vector<uint8_t> frame = {
      HEADER,
      HEADER,
      6,
      CMD_SEND_ANGLE,
      static_cast<uint8_t>(joint_id),
      static_cast<uint8_t>((raw_val >> 8) & 0xFF),
      static_cast<uint8_t>(raw_val & 0xFF),
      static_cast<uint8_t>(speed & 0xFF),
      FOOTER};

    write_bytes(frame);
  }

  // Read current angles of all 6 joints
  std::vector<float> get_angles()
  {
    std::vector<float> result;
    if (fd < 0) return result;

    tcflush(fd, TCIFLUSH);

    // Request joint angles
    std::vector<uint8_t> req = {HEADER, HEADER, 2, CMD_GET_ANGLES, FOOTER};
    write_bytes(req);

    // Parse response packet: [0xFE, 0xFE, LEN, 0x20, DATA(12 bytes), 0xFA]
    uint8_t b = 0;
    while (read_byte(b, 200)) {
      if (b == HEADER) {
        if (read_byte(b, 50) && b == HEADER) {
          uint8_t len = 0;
          if (!read_byte(len, 50)) break;
          uint8_t cmd = 0;
          if (!read_byte(cmd, 50)) break;

          if (cmd == CMD_GET_ANGLES && len >= 14) {
            std::vector<uint8_t> payload(12);
            bool ok = true;
            for (int i = 0; i < 12; ++i) {
              if (!read_byte(payload[i], 50)) {
                ok = false;
                break;
              }
            }
            if (ok) {
              for (int i = 0; i < 6; ++i) {
                int16_t val = (static_cast<int16_t>(payload[i * 2]) << 8) | payload[i * 2 + 1];
                result.push_back(val / 100.0f);
              }
              return result;
            }
          }
        }
      }
    }
    return result;
  }
};
