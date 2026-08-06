/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** @author Jia Pan */

// fcl_distance: header-only extraction of FCL 0.7.0 include/fcl/narrowphase/distance-inl.h

#ifndef FCL_DISTANCE_INL_H
#define FCL_DISTANCE_INL_H

#include "fcl/narrowphase/distance.h"

namespace fcl
{

//==============================================================================
//==============================================================================
//==============================================================================
template <typename GJKSolver>
detail::DistanceFunctionMatrix<GJKSolver>& getDistanceFunctionLookTable()
{
  static detail::DistanceFunctionMatrix<GJKSolver> table;
  return table;
}

//==============================================================================
template <typename NarrowPhaseSolver>
typename NarrowPhaseSolver::S distance(
    const CollisionObject<typename NarrowPhaseSolver::S>* o1,
    const CollisionObject<typename NarrowPhaseSolver::S>* o2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename NarrowPhaseSolver::S>& request,
    DistanceResult<typename NarrowPhaseSolver::S>& result)
{
  return distance<NarrowPhaseSolver>(
        o1->collisionGeometry().get(),
        o1->getTransform(),
        o2->collisionGeometry().get(),
        o2->getTransform(),
        nsolver,
        request,
        result);
}

//==============================================================================
template <typename NarrowPhaseSolver>
typename NarrowPhaseSolver::S distance(
    const CollisionGeometry<typename NarrowPhaseSolver::S>* o1,
    const Transform3<typename NarrowPhaseSolver::S>& tf1,
    const CollisionGeometry<typename NarrowPhaseSolver::S>* o2,
    const Transform3<typename NarrowPhaseSolver::S>& tf2,
    const NarrowPhaseSolver* nsolver_,
    const DistanceRequest<typename NarrowPhaseSolver::S>& request,
    DistanceResult<typename NarrowPhaseSolver::S>& result)
{
  using S = typename NarrowPhaseSolver::S;

  const NarrowPhaseSolver* nsolver = nsolver_;
  if(!nsolver_)
    nsolver = new NarrowPhaseSolver();

  const auto& looktable = getDistanceFunctionLookTable<NarrowPhaseSolver>();

  OBJECT_TYPE object_type1 = o1->getObjectType();
  NODE_TYPE node_type1 = o1->getNodeType();
  OBJECT_TYPE object_type2 = o2->getObjectType();
  NODE_TYPE node_type2 = o2->getNodeType();

  S res = std::numeric_limits<S>::max();


  if(object_type1 == OT_GEOM && object_type2 == OT_BVH)
  {
    if(!looktable.distance_matrix[node_type2][node_type1])
    {
      std::cerr << "Warning: distance function between node type " << node_type1 << " and node type " << node_type2 << " is not supported\n";
    }
    else
    {
      res = looktable.distance_matrix[node_type2][node_type1](o2, tf2, o1, tf1, nsolver, request, result);
    }
  }
  else
  {
    if(!looktable.distance_matrix[node_type1][node_type2])
    {
      std::cerr << "Warning: distance function between node type " << node_type1 << " and node type " << node_type2 << " is not supported\n";
    }
    else
    {
      res = looktable.distance_matrix[node_type1][node_type2](o1, tf1, o2, tf2, nsolver, request, result);
    }
  }

  // Upstream FCL follows this with a signed-distance workaround: when the
  // result came out negative it re-runs the query through collide() and
  // reports minus the deepest penetration.  That path is dead for
  // mesh-vs-mesh, because TriangleDistance<S>::triDistance never returns a
  // negative value -- it returns exactly 0 for overlapping triangles.  It is
  // dropped here, which is what lets the whole collision dispatch chain
  // (collision_func_matrix, the collision traversal nodes, Intersect<S>, the
  // primitive intersection routines) stay out of this extraction.
  //
  // DistanceRequest::enable_signed_distance therefore has no effect: an
  // overlapping mesh pair reports 0, exactly as upstream FCL does when the
  // flag is left at its default.

  if(!nsolver_)
    delete nsolver;

  return res;
}

//==============================================================================
template <typename S>
S distance(
    const CollisionObject<S>* o1,
    const CollisionObject<S>* o2,
    const DistanceRequest<S>& request,
    DistanceResult<S>& result)
{
  // Upstream branches on request.gjk_solver_type here to build a
  // GJKSolver_libccd or a GJKSolver_indep.  Mesh-vs-mesh never consults the
  // solver (see detail::MeshDistanceSolver), so both branches would behave
  // identically; a single placeholder replaces them.
  detail::MeshDistanceSolver<S> solver;
  return distance(o1, o2, &solver, request, result);
}

//==============================================================================
template <typename S>
S distance(
    const CollisionGeometry<S>* o1, const Transform3<S>& tf1,
    const CollisionGeometry<S>* o2, const Transform3<S>& tf2,
    const DistanceRequest<S>& request, DistanceResult<S>& result)
{
  // See the note on the CollisionObject overload above.
  detail::MeshDistanceSolver<S> solver;
  return distance(o1, tf1, o2, tf2, &solver, request, result);
}

} // namespace fcl

#endif
