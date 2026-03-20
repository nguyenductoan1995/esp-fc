#pragma once

#include <cstdint>
#include <cstddef>
#include "Utils/Math.hpp"

namespace Espfc {

/**
 * @brief Các loại filter số được hỗ trợ trong esp-fc
 *
 * Dùng để cấu hình `FilterConfig` và chọn implementation trong `Filter::begin()`.
 */
enum FilterType {
  FILTER_PT1,       ///< First-order IIR low-pass (đơn giản, ít CPU nhất)
  FILTER_BIQUAD,    ///< Biquad filter (LPF/Notch/BPF) — dùng trong gyro LPF và notch
  FILTER_PT2,       ///< Second-order IIR low-pass (2× PT1 cascade)
  FILTER_PT3,       ///< Third-order IIR low-pass (3× PT1 cascade)
  FILTER_NOTCH,     ///< Notch filter biquad transposed form II (TF2)
  FILTER_NOTCH_DF1, ///< Notch filter biquad direct form I (DF1) — ổn định hơn khi reconfigure
  FILTER_BPF,       ///< Band-pass filter biquad
  FILTER_FO,        ///< First-order IIR dạng direct form (DF1 / DF2)
  FILTER_FIR2,      ///< 2-tap FIR filter đơn giản — dùng làm pre-filter trước notch
  FILTER_MEDIAN3,   ///< Median filter 3-sample — loại spike/glitch từ sensor
  FILTER_NONE,      ///< Bypass — không lọc
};

/**
 * @brief Loại biquad filter được dùng khi type == FILTER_BIQUAD
 */
enum BiquadFilterType {
  BIQUAD_FILTER_LPF,   ///< Low-pass filter
  BIQUAD_FILTER_NOTCH, ///< Notch (band-reject) filter
  BIQUAD_FILTER_BPF    ///< Band-pass filter
};

/**
 * @brief Cấu hình cho một bộ lọc số
 *
 * Được dùng để khởi tạo `Filter` qua `Filter::begin(config, rate)`.
 * Có thể serialize vào EEPROM như một phần của `ModelConfig`.
 */
class FilterConfig
{
  public:
    FilterConfig();

    /**
     * @brief Khởi tạo FilterConfig với type, tần số cắt và optional bandwidth
     *
     * @param t Loại filter (FilterType)
     * @param f Tần số cắt chính (Hz) — dùng làm center freq cho notch
     * @param c Tần số cắt thứ hai (Hz) — dùng làm bandwidth cho notch (tính Q)
     */
    FilterConfig(FilterType t, int16_t f, int16_t c = 0);

    /**
     * @brief Kiểm tra và điều chỉnh cấu hình về giá trị hợp lệ
     *
     * Đảm bảo `freq` không vượt quá Nyquist (rate/2) và không âm.
     * Được gọi trong `Filter::begin()` trước khi tính hệ số.
     *
     * @param rate Sample rate (Hz) của hệ thống
     * @return FilterConfig đã được sanitize
     */
    FilterConfig sanitize(int rate) const;

    int8_t type;    ///< Loại filter (cast từ FilterType)
    int16_t freq;   ///< Tần số cắt chính (Hz)
    int16_t cutoff; ///< Tần số cắt phụ (Hz) — dùng cho notch bandwidth hoặc DynLPF cutoff thấp
};

/// Số lượng notch filter tối đa cho dynamic notch (tính trên mỗi trục)
constexpr size_t DYN_NOTCH_COUNT_MAX = 6;

/**
 * @brief Cấu hình cho dynamic notch filter
 *
 * Dynamic notch tự động theo dõi và notch ra các tần số rung động từ
 * FreqAnalyzer/FFTAnalyzer. Dùng `count` notch song song trên mỗi trục.
 */
class DynamicFilterConfig {
  public:
    DynamicFilterConfig() {}

    /**
     * @brief Khởi tạo với đầy đủ tham số
     *
     * @param c  Số notch song song (1–DYN_NOTCH_COUNT_MAX)
     * @param qf Q factor × 10 (ví dụ: 300 = Q=30, narrow notch)
     * @param lf Tần số tối thiểu hợp lệ (Hz)
     * @param hf Tần số tối đa hợp lệ (Hz)
     */
    DynamicFilterConfig(int8_t c, int16_t qf, int16_t lf, int16_t hf): count(c), q(qf), min_freq(lf), max_freq(hf) {}

    uint8_t count = 4;          ///< Số notch song song mỗi trục (mặc định: 4)
    int16_t q = 300;            ///< Q factor × 10 (mặc định: 300 = Q=30)
    int16_t min_freq = 80;      ///< Tần số tối thiểu để track (Hz)
    int16_t max_freq = 400;     ///< Tần số tối đa để track (Hz)
    static constexpr int MIN_FREQ = 1000; ///< Tần số sample tối thiểu để bật dyn notch
};

namespace Utils {

/**
 * @brief Tính gain cho PT1 filter (first-order IIR)
 *
 * Công thức: k = dt / (dt + RC) với RC = 1 / (2π × freq)
 *
 * @param rate Sample rate (Hz)
 * @param freq Tần số cắt (Hz)
 * @return Hệ số gain k ∈ (0, 1)
 */
inline float pt1Gain(float rate, float freq)
{
  float rc = 1.f / (2.f * pi() * freq);
  float dt = 1.f / rate;
  return dt / (dt + rc);
}

/**
 * @brief Trạng thái nội bộ cho PT1 filter (first-order IIR low-pass)
 *
 * Công thức: v = v + k × (n - v)
 * Nhẹ nhất về CPU, phù hợp cho LPF thứ cấp hoặc gyro LPF khi không cần Q control.
 */
class FilterStatePt1 {
  public:
    void reset();
    void reconfigure(const FilterStatePt1& from);
    void init(float rate, float freq);  ///< Tính k từ rate và freq
    float update(float n);              ///< Cập nhật filter với sample mới

    float k; ///< Gain coefficient
    float v; ///< State (giá trị đầu ra hiện tại)
};

/**
 * @brief Trạng thái nội bộ cho FIR2 filter (2-tap FIR moving average)
 *
 * output = (n + v[0]) / 2 — đơn giản, không có phase shift lớn.
 * Dùng làm pre-filter trước notch filter.
 */
class FilterStateFir2 {
  public:
    void reset();
    void init();
    void reconfigure(const FilterStateFir2& from);
    float update(float n); ///< Cập nhật filter với sample mới

    float v[2]; ///< Hai mẫu lịch sử
};

/**
 * @brief Trạng thái nội bộ cho Biquad filter (IIR bậc 2)
 *
 * Hỗ trợ LPF, Notch, BPF theo chuẩn bilinear transform.
 * Hai variant: Transposed Form II (update) và Direct Form I (updateDF1).
 * DF1 ổn định hơn khi reconfigure động (ít discontinuity).
 */
class FilterStateBiquad {
  public:
    void reset();
    /**
     * @brief Tính hệ số biquad từ type, rate, freq và Q
     *
     * @param filterType Loại biquad (LPF/Notch/BPF)
     * @param rate       Sample rate (Hz)
     * @param freq       Tần số cắt hoặc center (Hz)
     * @param q          Q factor (bandwidth control)
     */
    void init(BiquadFilterType filterType, float rate, float freq, float q);
    void reconfigure(const FilterStateBiquad& from);
    float update(float n);     ///< Transposed Form II (TF2) — mặc định
    float updateDF1(float n);  ///< Direct Form I (DF1) — dùng cho FILTER_NOTCH_DF1

    float b0, b1, b2; ///< Feed-forward coefficients
    float a1, a2;     ///< Feed-back coefficients (a0 = 1, đã chuẩn hóa)
    float x1, x2;    ///< Input delay state (dùng cho DF1)
    float y1, y2;    ///< Output delay state
};

/**
 * @brief Trạng thái nội bộ cho first-order IIR filter (DF1/DF2)
 *
 * Tương tự biquad nhưng bậc 1. Linh hoạt hơn PT1 vì hỗ trợ cả DF1.
 */
class FilterStateFirstOrder {
  public:
    void reset();
    void init(float rate, float freq);
    void reconfigure(const FilterStateFirstOrder& from);
    float update(float n);    ///< Transposed Form II
    float updateDF1(float n); ///< Direct Form I

    float b0, b1; ///< Feed-forward coefficients
    float a1;     ///< Feed-back coefficient
    float x1;     ///< Input delay state (DF1)
    float y1;     ///< Output delay state
};

/**
 * @brief Trạng thái nội bộ cho Median3 filter (median của 3 mẫu)
 *
 * Loại bỏ spike/glitch hiệu quả hơn LPF khi nhiễu xung.
 * Dùng cho baro và accel để loại bỏ outlier trước LPF.
 */
class FilterStateMedian {
  public:
    void reset();
    void init();
    void reconfigure(const FilterStateMedian& from);
    float update(float n); ///< Cập nhật với sample mới, trả về median

    float v[3]; ///< 3 mẫu lịch sử (sliding window)
};

/**
 * @brief Trạng thái nội bộ cho PT2 filter (2× PT1 cascade)
 *
 * Dốc cắt -40dB/decade (so với -20dB/decade của PT1).
 * Dùng cho baro pre-filter và accel vertical trong Altitude estimator.
 */
class FilterStatePt2 {
  public:
    void reset();
    void init(float rate, float freq);
    void reconfigure(const FilterStatePt2& from);
    float update(float n); ///< Cập nhật filter cascade 2 tầng PT1

    float k;    ///< Gain coefficient (dùng cho cả 2 tầng)
    float v[2]; ///< State của 2 tầng PT1
};

/**
 * @brief Trạng thái nội bộ cho PT3 filter (3× PT1 cascade)
 *
 * Dốc cắt -60dB/decade. Dùng khi cần lọc mạnh với ít CPU hơn biquad.
 */
class FilterStatePt3 {
  public:
    void reset();
    void init(float rate, float freq);
    void reconfigure(const FilterStatePt3& from);
    float update(float n); ///< Cập nhật filter cascade 3 tầng PT1

    float k;    ///< Gain coefficient (dùng cho cả 3 tầng)
    float v[3]; ///< State của 3 tầng PT1
};

/**
 * @brief Lớp filter đa hình — wrapper thống nhất cho tất cả FilterType
 *
 * Đây là lớp được sử dụng trực tiếp trong toàn bộ codebase esp-fc.
 * Nội bộ dùng `union` để chứa state của loại filter được chọn, tiết kiệm bộ nhớ.
 *
 * Sử dụng điển hình:
 * ```cpp
 * Utils::Filter f;
 * f.begin(FilterConfig(FILTER_BIQUAD, 200), 4000); // LPF 200Hz tại 4kHz
 * float out = f.update(input);
 * ```
 *
 * `reconfigure()` cho phép thay đổi tần số cắt động mà không reset state hoàn toàn
 * (dùng cho dynamic LPF và RPM filter). Truyền state cũ sang state mới qua `reconfigure(from)`.
 *
 * @note Trường `_input_weight` / `_output_weight` cho phép blend filter này với
 *       giá trị input/output khác (dùng cho RPM filter fade-in).
 */
class Filter
{
  public:
    Filter();

    /** @brief Khởi tạo filter bypass (FILTER_NONE) */
    void begin();

    /**
     * @brief Khởi tạo filter với cấu hình và sample rate
     *
     * @param config Cấu hình filter (type, freq, cutoff)
     * @param rate   Sample rate của hệ thống (Hz)
     */
    void begin(const FilterConfig& config, int rate);

    /**
     * @brief Cập nhật filter với một sample mới
     *
     * @param v Sample đầu vào
     * @return Giá trị đầu ra đã lọc
     */
    float update(float v);

    /** @brief Reset state nội bộ về 0 */
    void reset();

    /**
     * @brief Cập nhật tần số cắt không reset state (dynamic reconfiguration)
     *
     * Dùng cho dynamic LPF (thay đổi cutoff theo throttle) và RPM filter.
     *
     * @param freq   Tần số cắt mới (Hz)
     * @param cutoff Tần số cắt phụ (Hz, dùng cho notch bandwidth)
     */
    void reconfigure(int16_t freq, int16_t cutoff = 0);

    /**
     * @brief Reconfigure với Q factor và weight tùy chỉnh
     *
     * @param freq    Tần số cắt mới (Hz)
     * @param cutoff  Tần số cắt phụ (Hz)
     * @param q       Q factor mới
     * @param weight  Trọng số blend input [0.0, 1.0]
     */
    void reconfigure(int16_t freq, int16_t cutoff, float q, float weight = 1.0f);

    /** @brief Reconfigure từ FilterConfig và rate mới */
    void reconfigure(const FilterConfig& config, int rate);

    /** @brief Reconfigure với FilterConfig, rate, Q và weight */
    void reconfigure(const FilterConfig& config, int rate, float q, float weight);

    /**
     * @brief Copy cấu hình và tần số từ filter khác (không copy state)
     *
     * @param filter Filter nguồn để lấy cấu hình
     */
    void reconfigure(const Filter& filter);

    /**
     * @brief Đặt trọng số blend đầu vào
     *
     * @param weight [0.0, 1.0] — 1.0 = filter thuần túy, 0.0 = bypass
     */
    void setWeight(float weight);

    /**
     * @brief Tính Q factor gần đúng từ center freq và cutoff (notch bandwidth)
     *
     * Công thức nhanh, ít chính xác hơn `getNotchQ()` nhưng không dùng division.
     *
     * @param freq   Center frequency (Hz)
     * @param cutoff Bandwidth (Hz)
     * @return Q factor ước tính
     */
    float getNotchQApprox(float freq, float cutoff);

    /**
     * @brief Tính Q factor chính xác từ center freq và cutoff (notch bandwidth)
     *
     * @param freq   Center frequency (Hz)
     * @param cutoff Bandwidth (Hz)
     * @return Q factor chính xác
     */
    float getNotchQ(float freq, float cutoff);

#if !defined(UNIT_TEST)
  private:
#endif

    int _rate;          ///< Sample rate (Hz) được dùng khi khởi tạo
    FilterConfig _conf; ///< Cấu hình hiện tại
    union {
      FilterStatePt1 pt1;      ///< State cho FILTER_PT1
      FilterStateBiquad bq;    ///< State cho FILTER_BIQUAD, FILTER_NOTCH, FILTER_NOTCH_DF1, FILTER_BPF
      FilterStateFir2 fir2;    ///< State cho FILTER_FIR2
      FilterStateMedian median; ///< State cho FILTER_MEDIAN3
      FilterStatePt2 pt2;      ///< State cho FILTER_PT2
      FilterStatePt3 pt3;      ///< State cho FILTER_PT3
      FilterStateFirstOrder fo; ///< State cho FILTER_FO
    } _state;
    float _input_weight;  ///< Trọng số blend đầu vào (mặc định: 1.0)
    float _output_weight; ///< Trọng số blend đầu ra (mặc định: 0.0)
};

}

}