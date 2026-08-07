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

// fcl_distance (mesh-mesh only): reduced from FCL 0.7.0
// include/fcl/narrowphase/detail/distance_func_matrix-inl.h.
//
// Upstream registers 192 dispatch entries covering every shape pair, every
// BVH-vs-shape pair and the octree families.  This extraction serves
// mesh-vs-mesh distance only, so it keeps the four BVH-vs-BVH entries that
// upstream registers for that case and nothing else.  The code that remains
// is copied verbatim; only whole functions that no reachable entry referred
// to were dropped.

#ifndef FCL_DISTANCE_FUNC_MATRIX_INL_H
#define FCL_DISTANCE_FUNC_MATRIX_INL_H

#include "fcl/narrowphase/detail/distance_func_matrix.h"

#include "fcl/config.h"

#include "fcl/common/types.h"
#include "fcl/common/unused.h"

#include "fcl/narrowphase/collision_object.h"

#include "fcl/narrowphase/detail/traversal/collision_node.h"

#include "fcl/narrowphase/detail/traversal/distance/bvh_distance_traversal_node.h"
#include "fcl/narrowphase/detail/traversal/distance/distance_traversal_node_base.h"
#include "fcl/narrowphase/detail/traversal/distance/mesh_distance_traversal_node.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <typename S, typename BV>
struct BVHDistanceImpl
{
  static S run(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const DistanceRequest<S>& request,
      DistanceResult<S>& result)
  {
    if(request.isSatisfied(result)) return result.min_distance;
    MeshDistanceTraversalNode<BV> node;
    const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>* >(o1);
    const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>* >(o2);
    BVHModel<BV>* obj1_tmp = new BVHModel<BV>(*obj1);
    Transform3<S> tf1_tmp = tf1;
    BVHModel<BV>* obj2_tmp = new BVHModel<BV>(*obj2);
    Transform3<S> tf2_tmp = tf2;

    initialize(node, *obj1_tmp, tf1_tmp, *obj2_tmp, tf2_tmp, request, result);
    distance(&node);
    delete obj1_tmp;
    delete obj2_tmp;

    return result.min_distance;
  }
};

//==============================================================================
template <typename BV>
typename BV::S BVHDistance(
    const CollisionGeometry<typename BV::S>* o1,
    const Transform3<typename BV::S>& tf1,
    const CollisionGeometry<typename BV::S>* o2,
    const Transform3<typename BV::S>& tf2,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result)
{
  return BVHDistanceImpl<typename BV::S, BV>::run(
        o1, tf1, o2, tf2, request, result);
}

template <typename OrientedMeshDistanceTraversalNode, typename BV>
typename BV::S orientedMeshDistance(
    const CollisionGeometry<typename BV::S>* o1,
    const Transform3<typename BV::S>& tf1,
    const CollisionGeometry<typename BV::S>* o2,
    const Transform3<typename BV::S>& tf2,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result)
{
  if(request.isSatisfied(result)) return result.min_distance;
  OrientedMeshDistanceTraversalNode node;
  const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>* >(o1);
  const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>* >(o2);

  initialize(node, *obj1, tf1, *obj2, tf2, request, result);
  distance(&node);

  return result.min_distance;
}

//==============================================================================
template <typename S>
struct BVHDistanceImpl<S, RSS<S>>
{
  static S run(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const DistanceRequest<S>& request,
      DistanceResult<S>& result)
  {
    return detail::orientedMeshDistance<
        MeshDistanceTraversalNodeRSS<S>, RSS<S>>(
            o1, tf1, o2, tf2, request, result);
  }
};

//==============================================================================
template <typename S>
struct BVHDistanceImpl<S, kIOS<S>>
{
  static S run(
      const CollisionGeometry<S>* o1,
      const Transform3<S>& tf1,
      const CollisionGeometry<S>* o2,
      const Transform3<S>& tf2,
      const DistanceRequest<S>& request,
      DistanceResult<S>& result)
  {
    return detail::orientedMeshDistance<
        MeshDistanceTraversalNodekIOS<S>, kIOS<S>>(
            o1, tf1, o2, tf2, request, result);
  }
};


//==============================================================================
template <typename BV, typename NarrowPhaseSolver>
typename BV::S BVHDistance(
    const CollisionGeometry<typename BV::S>* o1,
    const Transform3<typename BV::S>& tf1,
    const CollisionGeometry<typename BV::S>* o2,
    const Transform3<typename BV::S>& tf2,
    const NarrowPhaseSolver* nsolver,
    const DistanceRequest<typename BV::S>& request,
    DistanceResult<typename BV::S>& result)
{
  FCL_UNUSED(nsolver);

  return BVHDistance<BV>(o1, tf1, o2, tf2, request, result);
}


//==============================================================================
template <typename NarrowPhaseSolver>
DistanceFunctionMatrix<NarrowPhaseSolver>::DistanceFunctionMatrix()
{
  for(int i = 0; i < NODE_COUNT; ++i)
  {
    for(int j = 0; j < NODE_COUNT; ++j)
      distance_matrix[i][j] = nullptr;
  }

  using S = typename NarrowPhaseSolver::S;

  distance_matrix[BV_AABB][BV_AABB] = &BVHDistance<AABB<S>, NarrowPhaseSolver>;
  distance_matrix[BV_RSS][BV_RSS] = &BVHDistance<RSS<S>, NarrowPhaseSolver>;
  distance_matrix[BV_kIOS][BV_kIOS] = &BVHDistance<kIOS<S>, NarrowPhaseSolver>;
}

} // namespace detail
} // namespace fcl

#endif
