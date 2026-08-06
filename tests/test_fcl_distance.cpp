// fcl_distance — golden-value and consistency tests.
//
// No external test framework: a tiny CHECK harness keeps the suite
// dependency-free, matching the library's goals.  Values are validated
// against closed-form geometry; tolerances follow the solver defaults
// (distance_tolerance = 1e-6 for iterative paths, ~1e-12 for analytic
// specializations in double).

#include "fcl/fcl.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

static int g_failures = 0;
static int g_checks = 0;

static const double kPi = 3.14159265358979323846;

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

Transform3d at(double x, double y, double z)
{
  Transform3d tf = Transform3d::Identity();
  tf.translation() = Vector3d(x, y, z);
  return tf;
}

double run_distance(const fcl::CollisionGeometry<double>* g1,
                    const Transform3d& tf1,
                    const fcl::CollisionGeometry<double>* g2,
                    const Transform3d& tf2,
                    fcl::GJKSolverType solver,
                    bool signed_distance = false,
                    bool nearest_points = false,
                    fcl::DistanceResult<double>* out = nullptr)
{
  fcl::DistanceRequest<double> req;
  req.gjk_solver_type = solver;
  req.enable_signed_distance = signed_distance;
  req.enable_nearest_points = nearest_points;
  fcl::DistanceResult<double> local;
  fcl::DistanceResult<double>& res = out ? *out : local;
  return fcl::distance(g1, tf1, g2, tf2, req, res);
}

// Axis-aligned box mesh [-hx,hx]x[-hy,hy]x[-hz,hz] with 12 triangles.
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
  const int f[12][3] = {{0, 2, 1}, {0, 3, 2},   // bottom (z = -hz)
                        {4, 5, 6}, {4, 6, 7},   // top
                        {0, 1, 5}, {0, 5, 4},   // front (y = -hy)
                        {2, 3, 7}, {2, 7, 6},   // back
                        {1, 2, 6}, {1, 6, 5},   // right (x = +hx)
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

void test_primitive_pairs(fcl::GJKSolverType solver, double tol_analytic,
                          double tol_gjk)
{
  using namespace fcl;

  // Sphere-Sphere (analytic specialization in both solvers).
  {
    Sphere<double> s1(1.0), s2(1.0);
    DistanceResult<double> res;
    double d = run_distance(&s1, at(0, 0, 0), &s2, at(3, 0, 0), solver,
                            false, true, &res);
    CHECK_NEAR(d, 1.0, tol_analytic);
    CHECK_NEAR(res.min_distance, 1.0, tol_analytic);
    CHECK_NEAR(res.nearest_points[0][0], 1.0, tol_analytic);
    CHECK_NEAR(res.nearest_points[1][0], 2.0, tol_analytic);
  }

  // Box-Box, axis-aligned (generic GJK path).
  {
    Box<double> b1(2, 2, 2), b2(2, 2, 2);
    double d = run_distance(&b1, at(0, 0, 0), &b2, at(5, 0, 0), solver);
    CHECK_NEAR(d, 3.0, tol_gjk);
  }

  // Sphere-Box (analytic specialization).
  {
    Sphere<double> s(1.0);
    Box<double> b(2, 2, 2);
    DistanceResult<double> res;
    double d = run_distance(&s, at(0, 0, 0), &b, at(4, 0, 0), solver,
                            false, true, &res);
    CHECK_NEAR(d, 2.0, tol_analytic);
    CHECK_NEAR((res.nearest_points[0] - res.nearest_points[1]).norm(), d,
               1e-6);
  }

  // Sphere-Capsule (analytic): capsule axis z in [-1, 1], radius 0.5.
  {
    Sphere<double> s(0.5);
    Capsule<double> c(0.5, 2.0);
    double d = run_distance(&s, at(3, 0, 0), &c, at(0, 0, 0), solver);
    CHECK_NEAR(d, 2.0, tol_analytic);
  }

  // Sphere-Cylinder (analytic).
  {
    Sphere<double> s(1.0);
    Cylinder<double> c(1.0, 2.0);
    double d = run_distance(&s, at(4, 0, 0), &c, at(0, 0, 0), solver);
    CHECK_NEAR(d, 2.0, tol_analytic);
  }

  // Capsule-Capsule (signed analytic function).
  {
    Capsule<double> c1(0.5, 2.0), c2(0.5, 2.0);
    double d = run_distance(&c1, at(0, 0, 0), &c2, at(2, 0, 0), solver);
    CHECK_NEAR(d, 1.0, tol_analytic);
  }

  // Rotated box vs sphere: box 2x2x2 rotated 45 deg about z; its corner
  // ridge reaches sqrt(2) along +x; sphere r=1 centered at (4,0,0).
  {
    Box<double> b(2, 2, 2);
    Sphere<double> s(1.0);
    Transform3d tfb = Transform3d::Identity();
    tfb.linear() =
        fcl::AngleAxis<double>(kPi / 4.0, Vector3d(0, 0, 1)).toRotationMatrix();
    double d = run_distance(&b, tfb, &s, at(4, 0, 0), solver);
    CHECK_NEAR(d, 4.0 - std::sqrt(2.0) - 1.0, tol_gjk);
  }

  // Ellipsoid vs sphere along principal axis (generic GJK).
  {
    Ellipsoid<double> e(1.0, 2.0, 3.0);
    Sphere<double> s(1.0);
    double d = run_distance(&e, at(0, 0, 0), &s, at(5, 0, 0), solver);
    CHECK_NEAR(d, 3.0, tol_gjk);
  }

  // Symmetry: distance(A, B) == distance(B, A).
  {
    Box<double> b(1, 2, 3);
    Capsule<double> c(0.5, 2.0);
    double d1 = run_distance(&b, at(0, 0, 0), &c, at(3, 1, 0), solver);
    double d2 = run_distance(&c, at(3, 1, 0), &b, at(0, 0, 0), solver);
    CHECK_NEAR(d1, d2, 1e-6);
  }
}

void test_signed_distance()
{
  using namespace fcl;

  // Overlapping spheres, penetration depth 0.5.
  // GST_LIBCCD: EPA-based signed distance (GEOM x GEOM early-return path).
  {
    Sphere<double> s1(1.0), s2(1.0);
    DistanceResult<double> res;
    run_distance(&s1, at(0, 0, 0), &s2, at(1.5, 0, 0), GST_LIBCCD, true,
                 true, &res);
    CHECK_NEAR(res.min_distance, -0.5, 1e-4);
  }

  // GST_INDEP: falls back to collide(); sphere-sphere intersect
  // specialization produces a positive depth, so min_distance = -depth.
  {
    Sphere<double> s1(1.0), s2(1.0);
    DistanceResult<double> res;
    run_distance(&s1, at(0, 0, 0), &s2, at(1.5, 0, 0), GST_INDEP, true,
                 true, &res);
    CHECK_NEAR(res.min_distance, -0.5, 1e-6);
  }

  // Separated + signed flag: behaves like plain distance.
  {
    Sphere<double> s1(1.0), s2(1.0);
    DistanceResult<double> res;
    double d = run_distance(&s1, at(0, 0, 0), &s2, at(3, 0, 0), GST_LIBCCD,
                            true, false, &res);
    CHECK_NEAR(d, 1.0, 1e-6);
  }
}

template <typename BV>
void test_mesh_shape(fcl::GJKSolverType solver, double tol)
{
  using namespace fcl;

  BVHModel<BV> mesh;
  buildBoxMesh(mesh, 1.0, 1.0, 1.0);  // box [-1,1]^3 as triangle mesh

  Sphere<double> s(1.0);
  double d = run_distance(&mesh, at(0, 0, 0), &s, at(4, 0, 0), solver);
  CHECK_NEAR(d, 2.0, tol);

  // Swapped order exercises the GEOM x BVH -> swap path.
  double d2 = run_distance(&s, at(4, 0, 0), &mesh, at(0, 0, 0), solver);
  CHECK_NEAR(d2, 2.0, tol);

  // Penetrating mesh-shape without the signed flag: upstream FCL reports a
  // non-positive value (leaf writes -1 on penetration).
  DistanceResult<double> res;
  run_distance(&mesh, at(0, 0, 0), &s, at(1.0, 0, 0), solver, false, false,
               &res);
  CHECK_TRUE(res.min_distance <= 0.0);
}

template <typename BV>
void test_mesh_mesh(double tol)
{
  using namespace fcl;

  BVHModel<BV> m1, m2;
  buildBoxMesh(m1, 0.5, 0.5, 0.5);
  buildBoxMesh(m2, 0.5, 0.5, 0.5);

  fcl::DistanceRequest<double> req;
  req.enable_nearest_points = true;
  fcl::DistanceResult<double> res;
  double d = fcl::distance(&m1, at(0, 0, 0), &m2, at(3, 0, 0), req, res);
  CHECK_NEAR(d, 2.0, tol);
  CHECK_NEAR((res.nearest_points[0] - res.nearest_points[1]).norm(), d, 1e-6);

  // Touching/overlapping meshes report 0 (triangle distance is never
  // negative).
  res.clear();
  fcl::distance(&m1, at(0, 0, 0), &m2, at(0.5, 0, 0), req, res);
  CHECK_NEAR(res.min_distance, 0.0, 1e-12);
}

void test_collision_objects()
{
  using namespace fcl;

  auto s1 = std::make_shared<Sphere<double>>(1.0);
  auto s2 = std::make_shared<Sphere<double>>(1.0);
  CollisionObject<double> o1(s1, at(0, 0, 0));
  CollisionObject<double> o2(s2, at(3, 0, 0));

  DistanceRequest<double> req;
  DistanceResult<double> res;
  double d = distance(&o1, &o2, req, res);
  CHECK_NEAR(d, 1.0, 1e-6);
}

void test_solver_agreement()
{
  using namespace fcl;

  // Both solvers should agree on separated smooth cases within tolerance.
  Box<double> b(1.2, 0.7, 2.1);
  Cylinder<double> c(0.6, 1.4);
  Transform3d tfb = Transform3d::Identity();
  tfb.linear() =
      fcl::AngleAxis<double>(0.3, Vector3d(1, 1, 1).normalized())
          .toRotationMatrix();
  tfb.translation() = Vector3d(0.1, -0.2, 0.05);

  double d1 = run_distance(&b, tfb, &c, at(3, 1, -0.5), GST_LIBCCD);
  double d2 = run_distance(&b, tfb, &c, at(3, 1, -0.5), GST_INDEP);
  CHECK_NEAR(d1, d2, 1e-4);
}

}  // namespace

int main()
{
  test_primitive_pairs(fcl::GST_LIBCCD, 1e-9, 1e-4);
  test_primitive_pairs(fcl::GST_INDEP, 1e-9, 1e-4);
  test_signed_distance();
  test_mesh_shape<fcl::OBBRSS<double>>(fcl::GST_LIBCCD, 1e-6);
  test_mesh_shape<fcl::OBBRSS<double>>(fcl::GST_INDEP, 1e-6);
  test_mesh_shape<fcl::RSS<double>>(fcl::GST_LIBCCD, 1e-6);
  test_mesh_shape<fcl::kIOS<double>>(fcl::GST_LIBCCD, 1e-6);
  test_mesh_mesh<fcl::OBBRSS<double>>(1e-6);
  test_mesh_mesh<fcl::RSS<double>>(1e-6);
  test_mesh_mesh<fcl::kIOS<double>>(1e-6);
  test_mesh_mesh<fcl::AABB<double>>(1e-6);
  test_collision_objects();
  test_solver_agreement();

  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
