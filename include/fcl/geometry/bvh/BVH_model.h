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

// fcl_distance: header-only extraction of FCL 0.7.0 include/fcl/geometry/bvh/BVH_model.h

#ifndef FCL_BVH_MODEL_H
#define FCL_BVH_MODEL_H

#include <vector>
#include <memory>

#include "fcl/math/bv/OBB.h"
#include "fcl/geometry/collision_geometry.h"
#include "fcl/geometry/bvh/BVH_internal.h"
#include "fcl/geometry/bvh/BV_node.h"
#include "fcl/geometry/bvh/detail/BV_splitter.h"
#include "fcl/geometry/bvh/detail/BV_fitter.h"

namespace fcl
{

/// @brief A class describing the bounding hierarchy of a mesh model or a point cloud model (which is viewed as a degraded version of mesh)
template <typename BV>
class FCL_EXPORT BVHModel : public CollisionGeometry<typename BV::S>
{
public:

  using S = typename BV::S;

  /// @brief Model type described by the instance
  BVHModelType getModelType() const;

  /// @brief Constructing an empty BVH
  BVHModel();

  /// @brief copy from another BVH
  BVHModel(const BVHModel& other);

  /// @brief deconstruction, delete mesh data related.
  ~BVHModel();

  /// @brief We provide getBV() and getNumBVs() because BVH may be compressed
  /// (in future), so we must provide some flexibility here
  
  /// @brief Access the bv giving the its index
  const BVNode<BV>& getBV(int id) const;

  /// @brief Access the bv giving the its index
  BVNode<BV>& getBV(int id);

  /// @brief Get the number of bv in the BVH
  int getNumBVs() const;

  /// @brief Get the object type: it is a BVH
  OBJECT_TYPE getObjectType() const override;

  /// @brief Get the BV type: default is unknown
  NODE_TYPE getNodeType() const override;

  /// @brief Compute the AABB for the BVH, used for broad-phase collision
  void computeLocalAABB() override;

  /// @brief Begin a new BVH model
  int beginModel(int num_tris = 0, int num_vertices = 0);

  /// @brief Add one point in the new BVH model
  int addVertex(const Vector3<S>& p);

  /// @brief Add one triangle in the new BVH model
  int addTriangle(const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3);

  /// @brief Add a set of triangles in the new BVH model
  int addSubModel(const std::vector<Vector3<S>>& ps, const std::vector<Triangle>& ts);

  /// @brief Add a set of points in the new BVH model
  int addSubModel(const std::vector<Vector3<S>>& ps);

  /// @brief End BVH model construction, will build the bounding volume hierarchy
  int endModel();


  /// @brief Replace the geometry information of current frame (i.e. should have the same mesh topology with the previous frame)
  int beginReplaceModel();

  /// @brief Replace one point in the old BVH model
  int replaceVertex(const Vector3<S>& p);

  /// @brief Replace one triangle in the old BVH model
  int replaceTriangle(const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3);

  /// @brief Replace a set of points in the old BVH model
  int replaceSubModel(const std::vector<Vector3<S>>& ps);

  /// @brief End BVH model replacement; rebuilds the bounding volume
  /// hierarchy.  fcl_distance: upstream can refit instead of rebuilding
  /// here, but refitting is not part of this build, so the refit and
  /// bottomup parameters are gone and the tree is always rebuilt.
  int endReplaceModel();




  /// @brief This is a special acceleration: BVH_model default stores the BV's transform in world coordinate. However, we can also store each BV's transform related to its parent 
  /// BV node. When traversing the BVH, this can save one matrix transformation.

  Vector3<S> computeCOM() const override;

  S computeVolume() const override;

  Matrix3<S> computeMomentofInertia() const override;

public:
  /// @brief Geometry point data
  Vector3<S>* vertices;

  /// @brief Geometry triangle index data, will be nullptr for point clouds
  Triangle* tri_indices;

  /// @brief Number of triangles
  int num_tris;

  /// @brief Number of points
  int num_vertices;

  /// @brief The state of BVH building process
  BVHBuildState build_state;

  /// @brief Split rule to split one BV node into two children
  std::shared_ptr<detail::BVSplitterBase<BV>> bv_splitter;

  /// @brief Fitting rule to fit a BV node to a set of geometry primitives
  std::shared_ptr<detail::BVFitterBase<BV>> bv_fitter;

private:

  int num_tris_allocated;
  int num_vertices_allocated;
  int num_bvs_allocated;
  int num_vertex_updated; /// for ccd vertex update
  unsigned int* primitive_indices;

  /// @brief Bounding volume hierarchy
  BVNode<BV>* bvs;

  /// @brief Number of BV nodes in bounding volume hierarchy
  int num_bvs;

  /// @brief Build the bounding volume hierarchy
  int buildTree();

  /// @brief Recursive kernel for hierarchy construction
  int recursiveBuildTree(int bv_id, int first_primitive, int num_primitives);

};

} // namespace fcl

#include "fcl/geometry/bvh/BVH_model-inl.h"

#endif
