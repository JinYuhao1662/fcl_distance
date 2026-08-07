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

// fcl_distance: header-only extraction of FCL 0.7.0 include/fcl/math/bv/OBB-inl.h

#ifndef FCL_BV_OBB_INL_H
#define FCL_BV_OBB_INL_H

#include "fcl/math/bv/OBB.h"

#include "fcl/common/unused.h"

namespace fcl
{

//==============================================================================
//==============================================================================
//==============================================================================
//==============================================================================
//==============================================================================
//==============================================================================
//==============================================================================
template <typename S>
OBB<S>::OBB()
{
  // Do nothing
}

//==============================================================================
template <typename S>
OBB<S>::OBB(const Matrix3<S>& axis_,
                 const Vector3<S>& center_,
                 const Vector3<S>& extent_)
  : axis(axis_), To(center_), extent(extent_)
{
  // Do nothing
}

//==============================================================================
template <typename S>
bool OBB<S>::overlap(const OBB<S>& other) const
{
  /// compute the relative transform that takes us from this->frame to
  /// other.frame

  Vector3<S> t = other.To - To;
  Vector3<S> T(
        axis.col(0).dot(t), axis.col(1).dot(t), axis.col(2).dot(t));
  Matrix3<S> R = axis.transpose() * other.axis;

  return !obbDisjoint(R, T, extent, other.extent);
}

//==============================================================================
template <typename S>
bool OBB<S>::overlap(const OBB& other, OBB& overlap_part) const
{
  FCL_UNUSED(overlap_part);

  return overlap(other);
}

//==============================================================================
template <typename S>
bool OBB<S>::contain(const Vector3<S>& p) const
{
  Vector3<S> local_p = p - To;
  S proj = local_p.dot(axis.col(0));
  if((proj > extent[0]) || (proj < -extent[0]))
    return false;

  proj = local_p.dot(axis.col(1));
  if((proj > extent[1]) || (proj < -extent[1]))
    return false;

  proj = local_p.dot(axis.col(2));
  if((proj > extent[2]) || (proj < -extent[2]))
    return false;

  return true;
}

//==============================================================================
template <typename S>
OBB<S>& OBB<S>::operator +=(const Vector3<S>& p)
{
  OBB<S> bvp(axis, p, Vector3<S>::Zero());
  *this += bvp;

  return *this;
}

//==============================================================================
template <typename S>
OBB<S>& OBB<S>::operator +=(const OBB<S>& other)
{
  *this = *this + other;

  return *this;
}

//==============================================================================
template <typename S>
OBB<S> OBB<S>::operator +(const OBB<S>& other) const
{
  Vector3<S> center_diff = To - other.To;
  S max_extent = std::max(std::max(extent[0], extent[1]), extent[2]);
  S max_extent2 = std::max(std::max(other.extent[0], other.extent[1]), other.extent[2]);
  if(center_diff.norm() > 2 * (max_extent + max_extent2))
  {
    return merge_largedist(*this, other);
  }
  else
  {
    return merge_smalldist(*this, other);
  }
}

//==============================================================================
template <typename S>
S OBB<S>::width() const
{
  return 2 * extent[0];
}

//==============================================================================
template <typename S>
S OBB<S>::height() const
{
  return 2 * extent[1];
}

//==============================================================================
template <typename S>
S OBB<S>::depth() const
{
  return 2 * extent[2];
}

//==============================================================================
template <typename S>
S OBB<S>::volume() const
{
  return width() * height() * depth();
}

//==============================================================================
template <typename S>
S OBB<S>::size() const
{
  return extent.squaredNorm();
}

//==============================================================================
template <typename S>
const Vector3<S> OBB<S>::center() const
{
  return To;
}

//==============================================================================
template <typename S>
S OBB<S>::distance(const OBB& other, Vector3<S>* P,
                             Vector3<S>* Q) const
{
  FCL_UNUSED(other);
  FCL_UNUSED(P);
  FCL_UNUSED(Q);

  std::cerr << "OBB distance not implemented!\n";
  return 0.0;
}

//==============================================================================

//==============================================================================

//==============================================================================

//==============================================================================
template <typename S, typename Derived>
OBB<S> translate(
    const OBB<S>& bv, const Eigen::MatrixBase<Derived>& t)
{
  OBB<S> res(bv);
  res.To += t;
  return res;
}

//==============================================================================
template <typename S, typename DerivedA, typename DerivedB>
bool overlap(const Eigen::MatrixBase<DerivedA>& R0,
             const Eigen::MatrixBase<DerivedB>& T0,
             const OBB<S>& b1, const OBB<S>& b2)
{
  typename DerivedA::PlainObject R0b2 = R0 * b2.axis;
  typename DerivedA::PlainObject R = b1.axis.transpose() * R0b2;

  typename DerivedB::PlainObject Ttemp = R0 * b2.To + T0 - b1.To;
  typename DerivedB::PlainObject T = Ttemp.transpose() * b1.axis;

  return !obbDisjoint(R, T, b1.extent, b2.extent);
}

//==============================================================================

//==============================================================================

} // namespace fcl

#endif
