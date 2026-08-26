// C++23 module partition: bounded breadcrumb storage and the route recorder.
module;

#include <array>
#include <cstddef>
#include <span>

export module ugv.rth:route;

import :types;
import :geo;

export namespace ugv::rth {

/// Fixed-capacity contiguous container (MISRA-friendly: no dynamic allocation,
/// bounded worst-case memory and time). A thin subset of the std::vector API.
template <class T, std::size_t Capacity>
class StaticVector {
public:
    using value_type = T;

    [[nodiscard]] constexpr std::size_t size() const noexcept { return size_; }
    [[nodiscard]] static constexpr std::size_t capacity() noexcept { return Capacity; }
    [[nodiscard]] constexpr bool empty() const noexcept { return size_ == 0U; }
    [[nodiscard]] constexpr bool full() const noexcept { return size_ == Capacity; }

    /// Append `v`; returns false (no-op) if the buffer is full.
    constexpr bool push_back(const T& v) noexcept {
        if (size_ == Capacity) {
            return false;
        }
        data_[size_] = v;
        ++size_;
        return true;
    }

    constexpr void clear() noexcept { size_ = 0U; }

    constexpr void pop_back() noexcept {
        if (size_ != 0U) {
            --size_;
        }
    }

    [[nodiscard]] constexpr T& operator[](std::size_t i) noexcept { return data_[i]; }
    [[nodiscard]] constexpr const T& operator[](std::size_t i) const noexcept { return data_[i]; }
    [[nodiscard]] constexpr const T& front() const noexcept { return data_[0]; }
    [[nodiscard]] constexpr const T& back() const noexcept { return data_[size_ - 1U]; }

    [[nodiscard]] constexpr std::span<const T> view() const noexcept { return {data_.data(), size_}; }
    [[nodiscard]] constexpr const T* begin() const noexcept { return data_.data(); }
    [[nodiscard]] constexpr const T* end() const noexcept { return data_.data() + size_; }

    /// Halve the resolution in place, keeping index 0 (home) and every 2nd
    /// sample. Used to make room without ever discarding the launch point.
    constexpr void decimateByHalf() noexcept {
        std::size_t w = 0U;
        for (std::size_t r = 0U; r < size_; r += 2U) {
            data_[w] = data_[r];
            ++w;
        }
        size_ = w;
    }

private:
    std::array<T, Capacity> data_{};
    std::size_t size_{0U};
};

/// Records the driven trail as a decimated breadcrumb list. New fixes are only
/// stored once the vehicle has moved at least `min_record_distance` from the
/// last stored point, bounding both memory and the eventual mission length.
class RouteRecorder {
public:
    RouteRecorder() = default;
    explicit RouteRecorder(RouteConfig cfg) noexcept
        : cfg_(cfg), min_dist_(cfg.min_record_distance) {}

    /// Offer a fresh position. Returns true if it was stored as a breadcrumb.
    bool record(const GeoPoint& p) noexcept {
        if (buf_.empty()) {
            return buf_.push_back(p);
        }
        if (geo::haversine(buf_.back(), p) < min_dist_) {
            return false;  // too close -> decimated away
        }
        if (buf_.full()) {
            buf_.decimateByHalf();  // preserve home; coarsen resolution
            min_dist_ *= 2.0;       // and slow future growth
        }
        return buf_.push_back(p);
    }

    [[nodiscard]] std::span<const GeoPoint> points() const noexcept { return buf_.view(); }
    [[nodiscard]] std::size_t size() const noexcept { return buf_.size(); }
    [[nodiscard]] bool empty() const noexcept { return buf_.empty(); }
    [[nodiscard]] const GeoPoint& home() const noexcept { return buf_.front(); }
    [[nodiscard]] const GeoPoint& last() const noexcept { return buf_.back(); }

    void clear() noexcept {
        buf_.clear();
        min_dist_ = cfg_.min_record_distance;
    }

    /// Total driven length along the recorded trail [m].
    [[nodiscard]] Scalar travelledDistance() const noexcept {
        Scalar d = 0.0;
        for (std::size_t i = 1U; i < buf_.size(); ++i) {
            d += geo::haversine(buf_[i - 1U], buf_[i]);
        }
        return d;
    }

private:
    RouteConfig cfg_{};
    StaticVector<GeoPoint, kMaxRoutePoints> buf_{};
    Scalar min_dist_{1.0};
};

}  // namespace ugv::rth
