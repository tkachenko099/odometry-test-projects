// C++23 module partition: fundamental navigation types and sensor samples.
module;

#include <Eigen/Dense>
#include <chrono>
#include <cstdint>

export module ugv.nav.core:types;

export namespace ugv::nav {

using Scalar = double;
using Vec3 = Eigen::Vector3d;
using Vec2 = Eigen::Vector2d;
using Quat = Eigen::Quaterniond;
using Mat3 = Eigen::Matrix3d;

template <int R, int C>
using Mat = Eigen::Matrix<Scalar, R, C>;

/// Monotonic timestamp in seconds (GNSS seconds-of-week compatible).
using Stamp = Scalar;

/// Geodetic position: latitude/longitude in radians, altitude in metres.
struct Geodetic {
    Scalar lat{0.0};  ///< Latitude  [rad]
    Scalar lon{0.0};  ///< Longitude [rad]
    Scalar alt{0.0};  ///< Ellipsoidal altitude [m]
};

/// Inertial measurement: body-frame specific force and angular rate.
struct ImuSample {
    Stamp t{0.0};
    Vec3 accel{Vec3::Zero()};  ///< Specific force [m/s^2] (forward-right-down)
    Vec3 gyro{Vec3::Zero()};   ///< Angular rate  [rad/s]
};

/// GNSS fix expressed as geodetic position with per-axis NED std-dev.
struct GnssSample {
    Stamp t{0.0};
    Geodetic llh{};
    Vec3 std_ned{Vec3::Constant(1.0)};  ///< Position std-dev (N,E,D) [m]
};

/// Differential-drive wheel encoder rates.
struct WheelSample {
    Stamp t{0.0};
    Scalar omega_left{0.0};   ///< Left wheel angular rate  [rad/s]
    Scalar omega_right{0.0};  ///< Right wheel angular rate [rad/s]
};

/// Full navigation state: position (NED, m), velocity (NED, m/s),
/// attitude (body->nav), and IMU biases.
struct NavState {
    Stamp t{0.0};
    Vec3 p{Vec3::Zero()};              ///< Position in local NED frame [m]
    Vec3 v{Vec3::Zero()};              ///< Velocity in local NED frame [m/s]
    Quat q{Quat::Identity()};          ///< Attitude body->nav (Hamilton)
    Vec3 accel_bias{Vec3::Zero()};     ///< Accelerometer bias [m/s^2]
    Vec3 gyro_bias{Vec3::Zero()};      ///< Gyroscope bias [rad/s]
};

/// 15-dof error-state ordering used by the ESKF covariance matrix.
enum class ErrIdx : int {
    kPos = 0,    ///< dp   (3)
    kVel = 3,    ///< dv   (3)
    kAtt = 6,    ///< dphi (3)
    kBAcc = 9,   ///< dba  (3)
    kBGyro = 12  ///< dbg  (3)
};

inline constexpr int kStateDim = 15;

}  // namespace ugv::nav
