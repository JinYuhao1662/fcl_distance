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

// fcl_distance: header-only extraction of FCL 0.7.0 include/fcl/geometry/bvh/detail/BV_fitter-inl.h

#ifndef FCL_BV_FITTER_INL_H
#define FCL_BV_FITTER_INL_H

#include "fcl/geometry/bvh/detail/BV_fitter.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <typename BV>
BVFitter<BV>::~BVFitter()
{
  // Do nothing
}

//==============================================================================
template <typename S, typename BV>
struct SetImpl;

//==============================================================================
template <typename BV>
void BVFitter<BV>::set(
    Vector3<typename BVFitter<BV>::S>* vertices_,
    Triangle* tri_indices_,
    BVHModelType type_)
{
  SetImpl<typename BV::S, BV>::run(*this, vertices_, tri_indices_, type_);
}

//==============================================================================
template <typename BV>
void BVFitter<BV>::set(
    Vector3<typename BVFitter<BV>::S>* vertices_,
    Vector3<typename BVFitter<BV>::S>* prev_vertices_,
    Triangle* tri_indices_,
    BVHModelType type_)
{
  SetImpl<typename BV::S, BV>::run(
        *this, vertices_, prev_vertices_, tri_indices_, type_);
}

//==============================================================================
template <typename S, typename BV>
struct FitImpl;

//==============================================================================
template <typename BV>
BV BVFitter<BV>::fit(unsigned int* primitive_indices, int num_primitives)
{
  return FitImpl<typename BV::S, BV>::run(
        *this, primitive_indices, num_primitives);
}

//==============================================================================
template <typename BV>
void BVFitter<BV>::clear()
{
  vertices = nullptr;
  prev_vertices = nullptr;
  tri_indices = nullptr;
  type = BVH_MODEL_UNKNOWN;
}

//==============================================================================
template <typename S, typename BV>
struct SetImpl
{
  static void run(
      BVFitter<BV>& fitter,
      Vector3<S>* vertices_,
      Triangle* tri_indices_,
      BVHModelType type_)
  {
    fitter.vertices = vertices_;
    fitter.prev_vertices = nullptr;
    fitter.tri_indices = tri_indices_;
    fitter.type = type_;
  }

  static void run(
      BVFitter<BV>& fitter,
      Vector3<S>* vertices_,
      Vector3<S>* prev_vertices_,
      Triangle* tri_indices_,
      BVHModelType type_)
  {
    fitter.vertices = vertices_;
    fitter.prev_vertices = prev_vertices_;
    fitter.tri_indices = tri_indices_;
    fitter.type = type_;
  }
};

//==============================================================================
template <typename S>
struct SetImpl<S, RSS<S>>
{
  static void run(
      BVFitter<RSS<S>>& fitter,
      Vector3<S>* vertices_,
      Triangle* tri_indices_,
      BVHModelType type_)
  {
    fitter.vertices = vertices_;
    fitter.prev_vertices = nullptr;
    fitter.tri_indices = tri_indices_;
    fitter.type = type_;
  }

  static void run(
      BVFitter<RSS<S>>& fitter,
      Vector3<S>* vertices_,
      Vector3<S>* prev_vertices_,
      Triangle* tri_indices_,
      BVHModelType type_)
  {
    fitter.vertices = vertices_;
    fitter.prev_vertices = prev_vertices_;
    fitter.tri_indices = tri_indices_;
    fitter.type = type_;
  }
};

//==============================================================================
template <typename S, typename BV>
struct FitImpl
{
  static BV run(
      const BVFitter<BV>& fitter,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    BV bv;

    if(fitter.type == BVH_MODEL_TRIANGLES)             /// The primitive is triangle
    {
      for(int i = 0; i < num_primitives; ++i)
      {
        Triangle t = fitter.tri_indices[primitive_indices[i]];
        bv += fitter.vertices[t[0]];
        bv += fitter.vertices[t[1]];
        bv += fitter.vertices[t[2]];

        if(fitter.prev_vertices)                      /// can fitting both current and previous frame
        {
          bv += fitter.prev_vertices[t[0]];
          bv += fitter.prev_vertices[t[1]];
          bv += fitter.prev_vertices[t[2]];
        }
      }
    }
    else if(fitter.type == BVH_MODEL_POINTCLOUD)       /// The primitive is point
    {
      for(int i = 0; i < num_primitives; ++i)
      {
        bv += fitter.vertices[primitive_indices[i]];

        if(fitter.prev_vertices)                       /// can fitting both current and previous frame
        {
          bv += fitter.prev_vertices[primitive_indices[i]];
        }
      }
    }

    return bv;
  }
};

//==============================================================================
template <typename S>
struct FitImpl<S, RSS<S>>
{
  static RSS<S> run(
      const BVFitter<RSS<S>>& fitter,
      unsigned int* primitive_indices,
      int num_primitives)
  {
    RSS<S> bv;

    Matrix3<S> M; // row first matrix
    Matrix3<S> E; // row first eigen-vectors
    Vector3<S> s; // three eigen values
    getCovariance(
          fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
          primitive_indices, num_primitives, M);
    eigen_old(M, s, E);
    axisFromEigen(E, s, bv.axis);

    // set rss origin, rectangle size and radius
    getRadiusAndOriginAndRectangleSize(
          fitter.vertices, fitter.prev_vertices, fitter.tri_indices,
          primitive_indices, num_primitives, bv.axis, bv.To, bv.l, bv.r);

    return bv;
  }
};


} // namespace detail
} // namespace fcl

#endif
