// fcl_distance — full-instantiation compile test.
//
// Calling fcl::distance / fcl::collide constructs the per-solver lookup
// tables, whose constructors take the address of (and therefore instantiate)
// every registered dispatch function — which transitively instantiates the
// whole traversal/solver/primitive closure.  If this translation unit
// compiles and links for both scalar types and both solvers, the port is
// structurally complete.

#include "fcl/fcl.h"

#include <memory>

template <typename S>
void instantiate_all()
{
  using namespace fcl;

  auto box = std::make_shared<Box<S>>(S(1), S(1), S(1));
  auto sphere = std::make_shared<Sphere<S>>(S(1));

  Transform3<S> tf1 = Transform3<S>::Identity();
  Transform3<S> tf2 = Transform3<S>::Identity();
  tf2.translation() = Vector3<S>(S(3), S(0), S(0));

  // Both solver types via request.gjk_solver_type — this constructs both
  // DistanceFunctionMatrix<GJKSolver_libccd> and ...<GJKSolver_indep>.
  DistanceRequest<S> dreq_ccd;
  dreq_ccd.gjk_solver_type = GST_LIBCCD;
  DistanceRequest<S> dreq_indep;
  dreq_indep.gjk_solver_type = GST_INDEP;
  DistanceResult<S> dres;

  distance(box.get(), tf1, sphere.get(), tf2, dreq_ccd, dres);
  dres.clear();
  distance(box.get(), tf1, sphere.get(), tf2, dreq_indep, dres);

  // Collision matrices (the signed-distance fallback needs them anyway).
  CollisionRequest<S> creq_ccd;
  creq_ccd.gjk_solver_type = GST_LIBCCD;
  CollisionRequest<S> creq_indep;
  creq_indep.gjk_solver_type = GST_INDEP;
  CollisionResult<S> cres;

  collide(box.get(), tf1, sphere.get(), tf2, creq_ccd, cres);
  cres.clear();
  collide(box.get(), tf1, sphere.get(), tf2, creq_indep, cres);

  // CollisionObject-level entry points.
  CollisionObject<S> obj1(box, tf1);
  CollisionObject<S> obj2(sphere, tf2);
  dres.clear();
  distance(&obj1, &obj2, dreq_ccd, dres);
  cres.clear();
  collide(&obj1, &obj2, creq_ccd, cres);

  // Every BVH model type used by the distance matrix, plus building.
  {
    BVHModel<AABB<S>> m1;
    BVHModel<OBB<S>> m2;
    BVHModel<RSS<S>> m3;
    BVHModel<OBBRSS<S>> m4;
    BVHModel<kIOS<S>> m5;
    BVHModel<KDOP<S, 16>> m6;
    BVHModel<KDOP<S, 18>> m7;
    BVHModel<KDOP<S, 24>> m8;
    (void)m1; (void)m2; (void)m3; (void)m4;
    (void)m5; (void)m6; (void)m7; (void)m8;
  }

  // All nine shapes exist and register.
  Ellipsoid<S> e(S(1), S(2), S(3));
  Capsule<S> ca(S(1), S(2));
  Cone<S> co(S(1), S(2));
  Cylinder<S> cy(S(1), S(2));
  Halfspace<S> hs(Vector3<S>(S(0), S(0), S(1)), S(0));
  Plane<S> pl(Vector3<S>(S(0), S(0), S(1)), S(0));
  TriangleP<S> tp(Vector3<S>(S(0), S(0), S(0)),
                  Vector3<S>(S(1), S(0), S(0)),
                  Vector3<S>(S(0), S(1), S(0)));
  (void)e; (void)ca; (void)co; (void)cy; (void)hs; (void)pl; (void)tp;
}

int main()
{
  instantiate_all<double>();
  instantiate_all<float>();
  return 0;
}
