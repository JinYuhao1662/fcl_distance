// fcl_distance — cross-validation sampler.
//
// Build this file TWICE: once against real FCL 0.7.0 (define
// CROSSCHECK_UPSTREAM, link fcl + ccd, needs Eigen) and once against this
// port (no defines, just -Iinclude).  Each build prints one line per query:
//   case_id solver d min_distance p1x p1y p1z p2x p2y p2z
// Diff the two outputs (tools/crosscheck_compare.py) to quantify agreement.
//
// The RNG is a fixed-seed LCG so both builds enumerate identical cases.

#ifdef CROSSCHECK_UPSTREAM
#include "fcl/fcl.h"
#include "fcl/narrowphase/distance.h"
#else
#include "fcl/fcl.h"
#endif

#include <cstdio>
#include <memory>
#include <vector>

using S = double;

// Deterministic LCG (identical sequence on every platform/compiler).
static unsigned long long g_state = 88172645463325252ull;
static double urand()  // in [-1, 1)
{
  g_state = g_state * 6364136223846793005ull + 1442695040888963407ull;
  return static_cast<double>(static_cast<long long>(g_state >> 11))
         / 4611686018427387904.0;
}

static fcl::Transform3<S> randomTransform(double span)
{
  fcl::Transform3<S> tf = fcl::Transform3<S>::Identity();
  fcl::Vector3<S> axis(urand(), urand(), urand());
  double n = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1]
                       + axis[2] * axis[2]);
  if (n < 1e-9)
  {
    axis = fcl::Vector3<S>(1, 0, 0);
    n = 1;
  }
  axis = axis / n;
  tf.linear() = fcl::AngleAxis<S>(urand() * 3.0, axis).toRotationMatrix();
  tf.translation() =
      fcl::Vector3<S>(urand() * span, urand() * span, urand() * span);
  return tf;
}

static void report(int case_id, int solver,
                   const fcl::DistanceResult<S>& res, double d)
{
  std::printf("%d %d %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n",
              case_id, solver, d, res.min_distance,
              res.nearest_points[0][0], res.nearest_points[0][1],
              res.nearest_points[0][2], res.nearest_points[1][0],
              res.nearest_points[1][1], res.nearest_points[1][2]);
}

static void query(int case_id, const fcl::CollisionGeometry<S>* g1,
                  const fcl::Transform3<S>& tf1,
                  const fcl::CollisionGeometry<S>* g2,
                  const fcl::Transform3<S>& tf2)
{
  const fcl::GJKSolverType solvers[2] = {fcl::GST_LIBCCD, fcl::GST_INDEP};
  for (int si = 0; si < 2; ++si)
  {
    fcl::DistanceRequest<S> req;
    req.gjk_solver_type = solvers[si];
    req.enable_nearest_points = true;
    fcl::DistanceResult<S> res;
    double d = fcl::distance(g1, tf1, g2, tf2, req, res);
    report(case_id, si, res, d);
  }
}

template <typename BV>
static std::shared_ptr<fcl::BVHModel<BV>> boxMesh(double hx, double hy,
                                                  double hz)
{
  auto model = std::make_shared<fcl::BVHModel<BV>>();
  std::vector<fcl::Vector3<S>> v(8);
  v[0] = fcl::Vector3<S>(-hx, -hy, -hz);
  v[1] = fcl::Vector3<S>(+hx, -hy, -hz);
  v[2] = fcl::Vector3<S>(+hx, +hy, -hz);
  v[3] = fcl::Vector3<S>(-hx, +hy, -hz);
  v[4] = fcl::Vector3<S>(-hx, -hy, +hz);
  v[5] = fcl::Vector3<S>(+hx, -hy, +hz);
  v[6] = fcl::Vector3<S>(+hx, +hy, +hz);
  v[7] = fcl::Vector3<S>(-hx, +hy, +hz);
  const int f[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                        {0, 1, 5}, {0, 5, 4}, {2, 3, 7}, {2, 7, 6},
                        {1, 2, 6}, {1, 6, 5}, {3, 0, 4}, {3, 4, 7}};
  std::vector<fcl::Triangle> tris;
  for (const auto& t : f)
    tris.push_back(fcl::Triangle(t[0], t[1], t[2]));
  model->beginModel();
  model->addSubModel(v, tris);
  model->endModel();
  model->computeLocalAABB();
  return model;
}

int main()
{
  int case_id = 0;

  // --- primitive pairs, random poses (separated to overlapping) ---
  fcl::Box<S> box(1.2, 0.8, 1.7);
  fcl::Sphere<S> sphere(0.9);
  fcl::Capsule<S> capsule(0.4, 1.5);
  fcl::Cylinder<S> cylinder(0.6, 1.1);
  fcl::Cone<S> cone(0.7, 1.3);
  fcl::Ellipsoid<S> ellipsoid(0.5, 0.9, 1.4);

  const fcl::CollisionGeometry<S>* shapes[] = {&box,      &sphere,
                                               &capsule,  &cylinder,
                                               &cone,     &ellipsoid};
  const int nshapes = 6;

  for (int i = 0; i < nshapes; ++i)
  {
    for (int j = i; j < nshapes; ++j)
    {
      for (int k = 0; k < 40; ++k)
      {
        // span shrinks so later cases overlap
        double span = (k < 20) ? 4.0 : 1.0;
        fcl::Transform3<S> tf1 = randomTransform(span);
        fcl::Transform3<S> tf2 = randomTransform(span);
        query(case_id++, shapes[i], tf1, shapes[j], tf2);
      }
    }
  }

  // --- mesh vs shape, mesh vs mesh ---
  auto mesh_obbrss = boxMesh<fcl::OBBRSS<S>>(0.6, 0.6, 0.6);
  auto mesh_rss = boxMesh<fcl::RSS<S>>(0.6, 0.6, 0.6);
  auto mesh2_obbrss = boxMesh<fcl::OBBRSS<S>>(0.4, 0.7, 0.5);

  for (int k = 0; k < 60; ++k)
  {
    double span = (k < 30) ? 4.0 : 1.2;
    fcl::Transform3<S> tf1 = randomTransform(span);
    fcl::Transform3<S> tf2 = randomTransform(span);
    query(case_id++, mesh_obbrss.get(), tf1, &sphere, tf2);
    query(case_id++, mesh_rss.get(), tf1, &box, tf2);
    query(case_id++, mesh_obbrss.get(), tf1, mesh2_obbrss.get(), tf2);
  }

  std::fprintf(stderr, "cases: %d\n", case_id);
  return 0;
}
