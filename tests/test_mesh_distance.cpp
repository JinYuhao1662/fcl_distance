// fcl_distance (mesh-mesh only) — golden-value tests.
//
// No external test framework, to match the library's dependency profile.
// Expected values are closed-form: the meshes are axis-aligned boxes, so the
// exact distance between them is known analytically.

#include "fcl/narrowphase/distance.h"
#include "fcl/geometry/bvh/BVH_model.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_NEAR(expr, expected, tol)                                       \
  do {                                                                        \
    ++g_checks;                                                               \
    const double v_ = static_cast<double>(expr);                              \
    const double e_ = static_cast<double>(expected);                          \
    if (!(std::fabs(v_ - e_) <= (tol))) {                                     \
      ++g_failures;                                                           \
      std::printf("FAIL %s:%d  %s = %.17g, expected %.17g (tol %g)\n",        \
                  __FILE__, __LINE__, #expr, v_, e_, static_cast<double>(tol));\
    }                                                                         \
  } while (0)

#define CHECK_TRUE(cond)                                                      \
  do {                                                                        \
    ++g_checks;                                                               \
    if (!(cond)) {                                                            \
      ++g_failures;                                                           \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);             \
    }                                                                         \
  } while (0)

using fcl::Vector3d;
using fcl::Transform3d;

namespace {

const double kPi = 3.14159265358979323846;

Transform3d at(double x, double y, double z)
{
  Transform3d tf = Transform3d::Identity();
  tf.translation() = Vector3d(x, y, z);
  return tf;
}

/// Axis-aligned box mesh spanning [-hx,hx] x [-hy,hy] x [-hz,hz], 12 triangles.
template <typename BV>
void buildBoxMesh(fcl::BVHModel<BV>& model, double hx, double hy, double hz)
{
  using S = typename BV::S;
  std::vector<fcl::Vector3<S>> v(8);
  v[0] = fcl::Vector3<S>(-hx, -hy, -hz);
  v[1] = fcl::Vector3<S>(+hx, -hy, -hz);
  v[2] = fcl::Vector3<S>(+hx, +hy, -hz);
  v[3] = fcl::Vector3<S>(-hx, +hy, -hz);
  v[4] = fcl::Vector3<S>(-hx, -hy, +hz);
  v[5] = fcl::Vector3<S>(+hx, -hy, +hz);
  v[6] = fcl::Vector3<S>(+hx, +hy, +hz);
  v[7] = fcl::Vector3<S>(-hx, +hy, +hz);
  const int f[12][3] = {{0, 2, 1}, {0, 3, 2},   // bottom
                        {4, 5, 6}, {4, 6, 7},   // top
                        {0, 1, 5}, {0, 5, 4},   // front
                        {2, 3, 7}, {2, 7, 6},   // back
                        {1, 2, 6}, {1, 6, 5},   // right
                        {3, 0, 4}, {3, 4, 7}};  // left
  std::vector<fcl::Triangle> tris;
  tris.reserve(12);
  for (const auto& t : f)
    tris.push_back(fcl::Triangle(t[0], t[1], t[2]));
  model.beginModel();
  model.addSubModel(v, tris);
  model.endModel();
  model.computeLocalAABB();
}

/// Single triangle in the z = 0 plane.
template <typename BV>
void buildTriangle(fcl::BVHModel<BV>& model, const Vector3d& a,
                   const Vector3d& b, const Vector3d& c)
{
  using S = typename BV::S;
  std::vector<fcl::Vector3<S>> v{a, b, c};
  std::vector<fcl::Triangle> tris{fcl::Triangle(0, 1, 2)};
  model.beginModel();
  model.addSubModel(v, tris);
  model.endModel();
  model.computeLocalAABB();
}

double run(const fcl::CollisionGeometry<double>* g1, const Transform3d& tf1,
           const fcl::CollisionGeometry<double>* g2, const Transform3d& tf2,
           bool nearest_points = false,
           fcl::DistanceResult<double>* out = nullptr)
{
  fcl::DistanceRequest<double> req;
  req.enable_nearest_points = nearest_points;
  fcl::DistanceResult<double> local;
  fcl::DistanceResult<double>& res = out ? *out : local;
  return fcl::distance(g1, tf1, g2, tf2, req, res);
}

template <typename BV>
void test_box_meshes(const char* name, double tol)
{
  std::printf("-- %s\n", name);

  fcl::BVHModel<BV> m1, m2;
  buildBoxMesh(m1, 0.5, 0.5, 0.5);   // unit cube centred at the origin
  buildBoxMesh(m2, 0.5, 0.5, 0.5);

  // Face to face along x: gap = 3 - 0.5 - 0.5.
  {
    fcl::DistanceResult<double> res;
    double d = run(&m1, at(0, 0, 0), &m2, at(3, 0, 0), true, &res);
    CHECK_NEAR(d, 2.0, tol);
    CHECK_NEAR(res.min_distance, 2.0, tol);
    CHECK_NEAR((res.nearest_points[0] - res.nearest_points[1]).norm(), d, 1e-6);
    // The witnesses must sit on the two facing faces.
    CHECK_NEAR(res.nearest_points[0][0], 0.5, 1e-6);
    CHECK_NEAR(res.nearest_points[1][0], 2.5, 1e-6);
  }

  // Diagonal offset: closest features are the two corners.
  {
    double d = run(&m1, at(0, 0, 0), &m2, at(2, 2, 2));
    CHECK_NEAR(d, std::sqrt(3.0) * 1.0, tol);   // (2-1) per axis
  }

  // Touching exactly: triangle distance reports 0, never negative.
  {
    double d = run(&m1, at(0, 0, 0), &m2, at(1, 0, 0));
    CHECK_NEAR(d, 0.0, 1e-12);
  }

  // Overlapping: still 0 (upstream FCL behaviour for mesh-mesh).
  {
    fcl::DistanceResult<double> res;
    run(&m1, at(0, 0, 0), &m2, at(0.5, 0, 0), false, &res);
    CHECK_NEAR(res.min_distance, 0.0, 1e-12);
    CHECK_TRUE(res.min_distance >= 0.0);
  }

  // Symmetry.
  {
    double d1 = run(&m1, at(0, 0, 0), &m2, at(1.7, 0.9, -0.4));
    double d2 = run(&m2, at(1.7, 0.9, -0.4), &m1, at(0, 0, 0));
    CHECK_NEAR(d1, d2, 1e-9);
  }

  // Rotation invariance: rotating both meshes about a shared axis by the same
  // amount leaves the distance unchanged.
  {
    double d_ref = run(&m1, at(0, 0, 0), &m2, at(3, 0, 0));
    Transform3d r1 = Transform3d::Identity();
    r1.linear() =
        fcl::AngleAxis<double>(0.7, Vector3d(0, 0, 1)).toRotationMatrix();
    Transform3d r2 = r1;
    r2.translation() = r1.linear() * Vector3d(3, 0, 0);
    double d_rot = run(&m1, r1, &m2, r2);
    CHECK_NEAR(d_rot, d_ref, 1e-9);
  }

  // 45-degree rotation about z: the corner ridge reaches sqrt(2)/2 along +x,
  // so a second box centred at (4,0,0) leaves 4 - sqrt(2)/2 - 0.5.
  {
    Transform3d tfr = Transform3d::Identity();
    tfr.linear() =
        fcl::AngleAxis<double>(kPi / 4.0, Vector3d(0, 0, 1)).toRotationMatrix();
    double d = run(&m1, tfr, &m2, at(4, 0, 0));
    CHECK_NEAR(d, 4.0 - std::sqrt(2.0) / 2.0 - 0.5, tol);
  }
}

template <typename BV>
void test_triangles(const char* name)
{
  std::printf("-- %s (single triangles)\n", name);

  fcl::BVHModel<BV> t1, t2;
  buildTriangle(t1, Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0));
  buildTriangle(t2, Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0));

  // Pure z separation of two identical coplanar triangles.
  {
    fcl::DistanceResult<double> res;
    double d = run(&t1, at(0, 0, 0), &t2, at(0, 0, 2.5), true, &res);
    CHECK_NEAR(d, 2.5, 1e-9);
    CHECK_NEAR((res.nearest_points[0] - res.nearest_points[1]).norm(), 2.5,
               1e-6);
  }

  // Coplanar, separated along x by a known gap.
  {
    double d = run(&t1, at(0, 0, 0), &t2, at(5, 0, 0));
    CHECK_NEAR(d, 4.0, 1e-9);   // vertex (1,0,0) to vertex (5,0,0)
  }

  // Identical position: overlapping, so 0.
  {
    double d = run(&t1, at(0, 0, 0), &t2, at(0, 0, 0));
    CHECK_NEAR(d, 0.0, 1e-12);
  }
}

/// All four BV types registered for mesh-mesh must agree with each other.
void test_bv_agreement()
{
  std::printf("-- cross-BV agreement\n");

  fcl::BVHModel<fcl::AABB<double>> a1, a2;
  fcl::BVHModel<fcl::RSS<double>> r1, r2;
  buildBoxMesh(a1, 0.6, 0.4, 0.9); buildBoxMesh(a2, 0.3, 0.7, 0.5);
  buildBoxMesh(r1, 0.6, 0.4, 0.9); buildBoxMesh(r2, 0.3, 0.7, 0.5);

  Transform3d tf1 = Transform3d::Identity();
  tf1.linear() =
      fcl::AngleAxis<double>(0.4, Vector3d(1, 1, 0).normalized())
          .toRotationMatrix();
  Transform3d tf2 = at(2.5, 1.1, -0.6);
  tf2.linear() =
      fcl::AngleAxis<double>(-0.9, Vector3d(0, 1, 1).normalized())
          .toRotationMatrix();

  const double da = run(&a1, tf1, &a2, tf2);
  const double dr = run(&r1, tf1, &r2, tf2);

  std::printf("   AABB=%.12g RSS=%.12g\n", da, dr);
  CHECK_NEAR(dr, da, 1e-9);
}

/// Streaming new vertex positions through beginReplaceModel /
/// replaceSubModel / endReplaceModel.  This is not an optional convenience:
/// the generic-BV distance path calls exactly these three internally to bake
/// the pose into the vertices before traversing, so it has to keep working.
template <typename BV>
void test_replace(const char* name)
{
  std::printf("-- %s (replace / rebuild)\n", name);

  fcl::BVHModel<BV> m2;
  buildBoxMesh(m2, 0.5, 0.5, 0.5);

  // Shrink a unit cube to half size, so the gap to m2 at x = 3 grows from
  // 3 - 0.5 - 0.5 to 3 - 0.25 - 0.5.
  using S = typename BV::S;
  std::vector<fcl::Vector3<S>> v(8);
  const double h = 0.25;
  v[0] = fcl::Vector3<S>(-h, -h, -h);
  v[1] = fcl::Vector3<S>(+h, -h, -h);
  v[2] = fcl::Vector3<S>(+h, +h, -h);
  v[3] = fcl::Vector3<S>(-h, +h, -h);
  v[4] = fcl::Vector3<S>(-h, -h, +h);
  v[5] = fcl::Vector3<S>(+h, -h, +h);
  v[6] = fcl::Vector3<S>(+h, +h, +h);
  v[7] = fcl::Vector3<S>(-h, +h, +h);

  // Same geometry through the replace path.
  fcl::BVHModel<BV> m3;
  buildBoxMesh(m3, 0.5, 0.5, 0.5);
  m3.beginReplaceModel();
  m3.replaceSubModel(v);
  m3.endReplaceModel();
  m3.computeLocalAABB();

  const double d2 = run(&m3, at(0, 0, 0), &m2, at(3, 0, 0));
  CHECK_NEAR(d2, 3.0 - 0.25 - 0.5, 1e-6);
}

/// The CollisionObject-level entry point must agree with the geometry one.
void test_collision_object()
{
  std::printf("-- CollisionObject entry point\n");

  auto m1 = std::make_shared<fcl::BVHModel<fcl::RSS<double>>>();
  auto m2 = std::make_shared<fcl::BVHModel<fcl::RSS<double>>>();
  buildBoxMesh(*m1, 0.5, 0.5, 0.5);
  buildBoxMesh(*m2, 0.5, 0.5, 0.5);

  fcl::CollisionObject<double> o1(m1, at(0, 0, 0));
  fcl::CollisionObject<double> o2(m2, at(3, 0, 0));

  fcl::DistanceRequest<double> req;
  fcl::DistanceResult<double> res;
  double d = fcl::distance(&o1, &o2, req, res);
  CHECK_NEAR(d, 2.0, 1e-6);
}

}  // namespace

int main()
{
  test_box_meshes<fcl::RSS<double>>("RSS", 1e-6);
  test_box_meshes<fcl::AABB<double>>("AABB", 1e-6);
  test_triangles<fcl::RSS<double>>("RSS");
  test_bv_agreement();
  test_replace<fcl::RSS<double>>("RSS");
  test_replace<fcl::AABB<double>>("AABB");
  test_collision_object();

  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
