// Eigen <-> C++20 modules link shim.
//
// Clang 19 attaches template instantiations that are first triggered inside a
// named module's purview to that module, giving them internal linkage. Eigen's
// free operators (e.g. `operator*(double, Vector3d)`) then end up as undefined
// references for any importer. This *non-module* translation unit includes
// Eigen normally and force-instantiates the offending operators with ordinary
// vague linkage, providing the definitions the archive is missing.
#include <Eigen/Dense>

namespace ugv::nav::eigen_support {

// Referenced from nowhere at runtime; exists purely so the linker emits the
// enclosed Eigen template instantiations with external/vague linkage.
[[maybe_unused]] void anchor() {
    using V3 = Eigen::Vector3d;
    using M15 = Eigen::Matrix<double, 15, 15>;

    V3 v = V3::Ones();
    volatile double sink = 0.0;

    const V3 a = 0.5 * v;            // operator*(double, MatrixBase<Vector3d>)
    const V3 b = 2.0 * v;
    const V3 c = -1.0 * v;
    sink += a.sum() + b.sum() + c.sum();

    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(0.1, V3::UnitX());  // Quaternion::operator=(AngleAxis)
    q = Eigen::AngleAxisd(0.2, V3::UnitY());
    q = Eigen::AngleAxisd(0.3, V3::UnitZ());
    sink += q.norm();

    M15 m = M15::Identity();
    const M15 n = 0.5 * m;           // operator*(double, MatrixBase<15x15>)
    sink += n.trace();

    (void)sink;
}

}  // namespace ugv::nav::eigen_support
