#pragma once

namespace Espfc {

/**
 * @brief Kết quả của các thao tác đọc/ghi EEPROM
 */
enum StorageResult
{
  STORAGE_NONE,              ///< Chưa thực hiện thao tác nào
  STORAGE_LOAD_SUCCESS,      ///< Đọc cấu hình thành công
  STORAGE_SAVE_SUCCESS,      ///< Ghi cấu hình thành công
  STORAGE_SAVE_ERROR,        ///< Lỗi khi ghi (flash error, v.v.)
  STORAGE_ERR_BAD_MAGIC,     ///< Magic byte không khớp — EEPROM chưa được ghi lần nào
  STORAGE_ERR_BAD_VERSION,   ///< Version không khớp — firmware mới, config cũ bị bỏ qua
  STORAGE_ERR_BAD_SIZE,      ///< Kích thước config không khớp — struct đã thay đổi
};

}

#ifndef UNIT_TEST

#include "ModelConfig.h"

namespace Espfc {

namespace Utils {

/**
 * @brief Quản lý lưu trữ cấu hình flight controller vào EEPROM/Flash
 *
 * Đọc và ghi `ModelConfig` vào bộ nhớ không mất điện (EEPROM hoặc NVS flash).
 * Mỗi lần ghi bao gồm:
 *   - Magic byte (`0xA5`) để xác nhận EEPROM đã được ghi
 *   - Version byte (`0x01`) để phát hiện thay đổi cấu trúc config
 *   - Toàn bộ `ModelConfig` struct (tối đa 2048 byte)
 *
 * Khi load:
 *   - `STORAGE_ERR_BAD_MAGIC` → EEPROM trống, dùng config mặc định
 *   - `STORAGE_ERR_BAD_VERSION` / `STORAGE_ERR_BAD_SIZE` → config cũ không tương thích,
 *     dùng config mặc định và ghi lại
 *
 * @note Không khả dụng trong `UNIT_TEST` build (guarded bởi `#ifndef UNIT_TEST`)
 * @note Trên ESP32: dùng Arduino EEPROM emulation trên NVS flash
 * @note Trên RP2040: dùng EEPROM emulation trong flash page cuối
 */
class Storage
{
  public:
    /**
     * @brief Khởi tạo EEPROM subsystem
     *
     * @return 1 nếu thành công
     */
    int begin();

    /**
     * @brief Đọc cấu hình từ EEPROM vào `config`
     *
     * Kiểm tra magic, version và size trước khi đọc.
     * Nếu kiểm tra thất bại, `config` không bị thay đổi.
     *
     * @param config Đối tượng cấu hình đích để ghi kết quả đọc
     * @return Kết quả: STORAGE_LOAD_SUCCESS hoặc một trong các STORAGE_ERR_*
     */
    StorageResult load(ModelConfig& config) const;

    /**
     * @brief Ghi `config` vào EEPROM
     *
     * Ghi magic byte, version byte, rồi toàn bộ `ModelConfig`.
     *
     * @param config Cấu hình cần lưu
     * @return STORAGE_SAVE_SUCCESS hoặc STORAGE_SAVE_ERROR
     */
    StorageResult save(const ModelConfig& config);

  private:
    static constexpr uint8_t EEPROM_MAGIC   = 0xA5; ///< Byte nhận dạng — EEPROM đã được ghi
    static constexpr uint8_t EEPROM_VERSION = 0x01; ///< Phiên bản cấu trúc config
    static constexpr size_t  EEPROM_SIZE    = 2048;  ///< Kích thước vùng EEPROM (byte)
};

}

}

#endif
