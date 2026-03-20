#pragma once

#include <cstdint>
#include <cstddef>
#include "Hal/Pgm.h"

namespace Espfc {

namespace Connect {

/// Kích thước buffer nhận MSP (byte) — đủ cho phần lớn gói tin Betaflight Configurator
constexpr size_t MSP_BUF_SIZE = 192;
/// Kích thước buffer gửi MSP (byte) — lớn hơn để chứa các response dài (blackbox, PID dump)
constexpr size_t MSP_BUF_OUT_SIZE = 240;

/**
 * @brief Trạng thái máy trạng thái parser MSP
 *
 * Parser xử lý byte-by-byte theo state machine này.
 * Hỗ trợ cả MSP v1 (header '$M') và MSP v2 (header '$X').
 */
enum MspState {
  MSP_STATE_IDLE,             ///< Đang chờ ký tự '$' bắt đầu gói tin
  MSP_STATE_HEADER_START,     ///< Đã nhận '$', chờ 'M' hoặc 'X'

  MSP_STATE_HEADER_M,         ///< MSP v1: đã nhận '$M', chờ '<' hoặc '>'
  MSP_STATE_HEADER_V1,        ///< MSP v1: đang nhận header (size + cmd)
  MSP_STATE_PAYLOAD_V1,       ///< MSP v1: đang nhận payload
  MSP_STATE_CHECKSUM_V1,      ///< MSP v1: đang xác nhận checksum

  MSP_STATE_HEADER_X,         ///< MSP v2: đã nhận '$X', chờ '<' hoặc '>'
  MSP_STATE_HEADER_V2,        ///< MSP v2: đang nhận header (flags + cmd + size)
  MSP_STATE_PAYLOAD_V2,       ///< MSP v2: đang nhận payload
  MSP_STATE_CHECKSUM_V2,      ///< MSP v2: đang xác nhận CRC8 DVB-S2

  MSP_STATE_RECEIVED,         ///< Gói tin hoàn chỉnh và hợp lệ, sẵn sàng xử lý
};

/**
 * @brief Loại gói tin MSP — lệnh từ Configurator hay reply từ FC
 */
enum MspType {
  MSP_TYPE_CMD,   ///< Lệnh từ Betaflight Configurator gửi đến FC (direction '<')
  MSP_TYPE_REPLY  ///< Reply từ FC gửi về Configurator (direction '>')
};

/**
 * @brief Phiên bản giao thức MSP
 */
enum MspVersion {
  MSP_V1, ///< MSP version 1 — header '$M', checksum XOR đơn giản
  MSP_V2  ///< MSP version 2 — header '$X', CRC8 DVB-S2, hỗ trợ cmd 16-bit
};

/**
 * @brief Header gói tin MSP v1 (packed, 2 byte)
 */
struct MspHeaderV1 {
    uint8_t size; ///< Kích thước payload (byte)
    uint8_t cmd;  ///< Mã lệnh MSP (8-bit)
} __attribute__((packed));

/**
 * @brief Header gói tin MSP v2 (packed, 5 byte)
 */
struct MspHeaderV2 {
    uint8_t  flags;   ///< Flags (thường = 0)
    uint16_t cmd;     ///< Mã lệnh MSP (16-bit, little-endian)
    uint16_t size;    ///< Kích thước payload (byte, little-endian)
} __attribute__((packed));

/**
 * @brief Đại diện cho một gói tin MSP đang được nhận (inbound message)
 *
 * Parser state machine ghi vào struct này khi nhận byte từ UART.
 * Khi `state == MSP_STATE_RECEIVED`, gói tin hoàn chỉnh và sẵn sàng xử lý.
 */
class MspMessage
{
public:
  MspMessage();

  /**
   * @brief Kiểm tra gói tin đã nhận hoàn chỉnh và hợp lệ chưa
   *
   * @return true nếu `state == MSP_STATE_RECEIVED`
   */
  bool isReady() const;

  /**
   * @brief Kiểm tra gói tin là lệnh từ Configurator (không phải reply)
   *
   * @return true nếu `dir == MSP_TYPE_CMD`
   */
  bool isCmd() const;

  /**
   * @brief Kiểm tra parser đang ở trạng thái rỗi (chưa nhận gói tin)
   *
   * @return true nếu `state == MSP_STATE_IDLE`
   */
  bool isIdle() const;

  /**
   * @brief Số byte còn lại trong buffer chưa được đọc
   *
   * @return Số byte chưa đọc = received - read
   */
  int remain() const;

  /**
   * @brief Tăng con trỏ đọc lên `size` byte
   *
   * @param size Số byte cần bỏ qua
   */
  void advance(size_t size);

  /** @brief Đọc 1 byte unsigned từ buffer payload */
  uint8_t readU8();
  /** @brief Đọc 2 byte unsigned (little-endian) từ buffer payload */
  uint16_t readU16();
  /** @brief Đọc 4 byte unsigned (little-endian) từ buffer payload */
  uint32_t readU32();

  MspState state;          ///< Trạng thái hiện tại của parser state machine
  MspType dir;             ///< Hướng gói tin (CMD từ GCS hoặc REPLY từ FC)
  MspVersion version;      ///< Phiên bản MSP (V1 hoặc V2)
  uint8_t flags;           ///< MSP v2 flags
  uint16_t cmd;            ///< Mã lệnh MSP
  uint16_t expected;       ///< Số byte payload dự kiến
  uint16_t received;       ///< Số byte payload đã nhận
  uint16_t read;           ///< Con trỏ đọc hiện tại trong buffer
  uint8_t checksum;        ///< Checksum đang tính (MSP v1: XOR, v2: CRC8)
  uint8_t checksum2;       ///< Checksum tham chiếu từ gói tin
  uint8_t buffer[MSP_BUF_SIZE]; ///< Buffer lưu payload
};

/**
 * @brief Đại diện cho một gói tin MSP cần gửi về (outbound response)
 *
 * Handler của từng MSP command ghi dữ liệu vào struct này thông qua các
 * phương thức `writeU8()`, `writeU16()`, `writeU32()`, v.v.
 * Sau đó `serialize()` chuyển thành byte stream để gửi qua UART.
 */
class MspResponse
{
public:
  MspResponse();

  MspVersion version;           ///< Phiên bản MSP dùng để serialize
  uint16_t cmd;                 ///< Mã lệnh MSP tương ứng với request
  int8_t  result;               ///< Kết quả xử lý: 1 = OK, -1 = lỗi, 0 = unsupported
  uint8_t direction;            ///< Hướng response (thường = '>')
  uint16_t len;                 ///< Số byte đã ghi vào `data`
  uint8_t data[MSP_BUF_OUT_SIZE]; ///< Buffer dữ liệu response

  /**
   * @brief Số byte còn có thể ghi vào buffer
   *
   * @return `MSP_BUF_OUT_SIZE - len`
   */
  int remain() const;

  /**
   * @brief Tăng con trỏ ghi lên `size` byte (dùng để bỏ qua khoảng trống)
   *
   * @param size Số byte cần bỏ qua
   */
  void advance(size_t size);

  /**
   * @brief Ghi `size` byte từ mảng `v` vào buffer response
   *
   * @param v    Con trỏ đến dữ liệu nguồn
   * @param size Số byte cần ghi
   */
  void writeData(const char * v, int size);

  /** @brief Ghi chuỗi null-terminated từ RAM vào buffer */
  void writeString(const char * v);
  /** @brief Ghi chuỗi từ Flash (PROGMEM) vào buffer */
  void writeString(const __FlashStringHelper *ifsh);
  /** @brief Ghi 1 byte unsigned vào buffer */
  void writeU8(uint8_t v);
  /** @brief Ghi 2 byte unsigned (little-endian) vào buffer */
  void writeU16(uint16_t v);
  /** @brief Ghi 4 byte unsigned (little-endian) vào buffer */
  void writeU32(uint32_t v);

  /**
   * @brief Serialize response thành byte stream hoàn chỉnh
   *
   * Tự động chọn format V1 hoặc V2 theo `version`.
   *
   * @param buff    Buffer đích để ghi byte stream
   * @param len_max Kích thước tối đa của buffer đích
   * @return Số byte đã ghi
   */
  size_t serialize(uint8_t * buff, size_t len_max) const;

  /**
   * @brief Serialize theo định dạng MSP v1 (`$M>`)
   *
   * @param buff    Buffer đích
   * @param len_max Kích thước tối đa
   * @return Số byte đã ghi
   */
  size_t serializeV1(uint8_t * buff, size_t len_max) const;

  /**
   * @brief Serialize theo định dạng MSP v2 (`$X>`)
   *
   * @param buff    Buffer đích
   * @param len_max Kích thước tối đa
   * @return Số byte đã ghi
   */
  size_t serializeV2(uint8_t * buff, size_t len_max) const;
};

}

}
