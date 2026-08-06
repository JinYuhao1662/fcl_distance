// fcl_distance — cross-validation sampler (mesh vs mesh).
//
// Build this file TWICE and diff the output:
//   upstream : g++ -DCROSSCHECK_UPSTREAM ... -lfcl -lccd      (system FCL 0.7.0)
//   port     : g++ -Iinclude ...                              (this extraction)
//
// Each build prints one line per query:
//   case_id bv d min_distance p1x p1y p1z p2x p2y p2z
//
// The RNG is a fixed-seed LCG, so both builds enumerate identical cases.

#ifdef CROSSCHECK_UPSTREAM
#include "fcl/narrowphase/distance.h"
#include "fcl/geometry/bvh/BVH_model.h"
#else
#include "fcl/narrowphase/distance.h"
#include "fcl/geometry/bvh/BVH_model.h"
#endif

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

using S = double;

// Deterministic LCG (identical sequence on every platform and compiler).
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

/// A coarse two-ring "torus" mesh, to exercise deeper BVH trees than a box.
template <typename BV>
static std::shared_ptr<fcl::BVHModel<BV>> ringMesh(int seg, double R, double r)
{
  auto model = std::make_shared<fcl::BVHModel<BV>>();
  std::vector<fcl::Vector3<S>> v;
  const double kPi = 3.14159265358979323846;
  for (int i = 0; i < seg; ++i)
  {
    const double a = 2.0 * kPi * i / seg;
    for (int j = 0; j < 4; ++j)
    {
      const double b = 2.0 * kPi * j / 4;
      v.push_back(fcl::Vector3<S>((R + r * std::cos(b)) * std::cos(a),
                                  (R + r * std::cos(b)) * std::sin(a),
                                  r * std::sin(b)));
    }
  }
  std::vector<fcl::Triangle> tris;
  for (int i = 0; i < seg; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      const int i2 = (i + 1) % seg, j2 = (j + 1) % 4;
      const int a = i * 4 + j, b = i2 * 4 + j, c = i2 * 4 + j2, d = i * 4 + j2;
      tris.push_back(fcl::Triangle(a, b, c));
      tris.push_back(fcl::Triangle(a, c, d));
    }
  }
  model->beginModel();
  model->addSubModel(v, tris);
  model->endModel();
  model->computeLocalAABB();
  return model;
}

static void report(int case_id, int bv, const fcl::DistanceResult<S>& res,
                   double d)
{
  std::printf("%d %d %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n",
              case_id, bv, d, res.min_distance,
              res.nearest_points[0][0], res.nearest_points[0][1],
              res.nearest_points[0][2], res.nearest_points[1][0],
              res.nearest_points[1][1], res.nearest_points[1][2]);
}

template <typename BV>
static void query(int case_id, int bv_id,
                  const fcl::BVHModel<BV>* m1, const fcl::Transform3<S>& tf1,
                  const fcl::BVHModel<BV>* m2, const fcl::Transform3<S>& tf2)
{
  fcl::DistanceRequest<S> req;
  req.enable_nearest_points = true;
  fcl::DistanceResult<S> res;
  double d = fcl::distance(m1, tf1, m2, tf2, req, res);
  report(case_id, bv_id, res, d);
}

int main()
{
  auto box_a_aabb = boxMesh<fcl::AABB<S>>(0.6, 0.6, 0.6);
  auto box_b_aabb = boxMesh<fcl::AABB<S>>(0.4, 0.7, 0.5);
  auto box_a_rss = boxMesh<fcl::RSS<S>>(0.6, 0.6, 0.6);
  auto box_b_rss = boxMesh<fcl::RSS<S>>(0.4, 0.7, 0.5);
  auto box_a_kios = boxMesh<fcl::kIOS<S>>(0.6, 0.6, 0.6);
  auto box_b_kios = boxMesh<fcl::kIOS<S>>(0.4, 0.7, 0.5);
  auto box_a_obbrss = boxMesh<fcl::OBBRSS<S>>(0.6, 0.6, 0.6);
  auto box_b_obbrss = boxMesh<fcl::OBBRSS<S>>(0.4, 0.7, 0.5);

  auto ring_a = ringMesh<fcl::OBBRSS<S>>(16, 1.0, 0.3);
  auto ring_b = ringMesh<fcl::OBBRSS<S>>(16, 0.8, 0.25);
  auto ring_a_rss = ringMesh<fcl::RSS<S>>(16, 1.0, 0.3);
  auto ring_b_rss = ringMesh<fcl::RSS<S>>(16, 0.8, 0.25);

  int case_id = 0;

  // Boxes: all four BV types over the same poses (separated then overlapping).
  for (int k = 0; k < 120; ++k)
  {
    const double span = (k < 60) ? 4.0 : 1.2;
    const fcl::Transform3<S> tf1 = randomTransform(span);
    const fcl::Transform3<S> tf2 = randomTransform(span);
    query(case_id, 0, box_a_aabb.get(), tf1, box_b_aabb.get(), tf2);
    query(case_id, 1, box_a_rss.get(), tf1, box_b_rss.get(), tf2);
    query(case_id, 2, box_a_kios.get(), tf1, box_b_kios.get(), tf2);
    query(case_id, 3, box_a_obbrss.get(), tf1, box_b_obbrss.get(), tf2);
    ++case_id;
  }

  // Rings: deeper BVH trees, 128 triangles each.
  for (int k = 0; k < 80; ++k)
  {
    const double span = (k < 40) ? 4.0 : 1.5;
    const fcl::Transform3<S> tf1 = randomTransform(span);
    const fcl::Transform3<S> tf2 = randomTransform(span);
    query(case_id, 3, ring_a.get(), tf1, ring_b.get(), tf2);
    query(case_id, 1, ring_a_rss.get(), tf1, ring_b_rss.get(), tf2);
    ++case_id;
  }

  std::fprintf(stderr, "cases: %d\n", case_id);
  return 0;
}
