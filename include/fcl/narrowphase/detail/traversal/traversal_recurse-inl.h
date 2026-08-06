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
// include/fcl/narrowphase/detail/traversal/traversal_recurse-inl.h.
// Only the distance recursions (and the BVT/BVTQ helpers they use) are kept;
// their bodies are copied verbatim.

#ifndef FCL_TRAVERSAL_TRAVERSALRECURSE_INL_H
#define FCL_TRAVERSAL_TRAVERSALRECURSE_INL_H

#include "fcl/narrowphase/detail/traversal/traversal_recurse.h"

#include <queue>

#include "fcl/common/unused.h"

namespace fcl
{

namespace detail
{
//==============================================================================
template <typename S>
FCL_EXPORT
void distanceRecurse(DistanceTraversalNodeBase<S>* node, int b1, int b2, BVHFrontList* front_list)
{
  bool l1 = node->isFirstNodeLeaf(b1);
  bool l2 = node->isSecondNodeLeaf(b2);

  if(l1 && l2)
  {
    updateFrontList(front_list, b1, b2);

    node->leafTesting(b1, b2);
    return;
  }

  int a1, a2, c1, c2;

  if(node->firstOverSecond(b1, b2))
  {
    a1 = node->getFirstLeftChild(b1);
    a2 = b2;
    c1 = node->getFirstRightChild(b1);
    c2 = b2;
  }
  else
  {
    a1 = b1;
    a2 = node->getSecondLeftChild(b2);
    c1 = b1;
    c2 = node->getSecondRightChild(b2);
  }

  S d1 = node->BVTesting(a1, a2);
  S d2 = node->BVTesting(c1, c2);

  if(d2 < d1)
  {
    if(!node->canStop(d2))
      distanceRecurse(node, c1, c2, front_list);
    else
      updateFrontList(front_list, c1, c2);

    if(!node->canStop(d1))
      distanceRecurse(node, a1, a2, front_list);
    else
      updateFrontList(front_list, a1, a2);
  }
  else
  {
    if(!node->canStop(d1))
      distanceRecurse(node, a1, a2, front_list);
    else
      updateFrontList(front_list, a1, a2);

    if(!node->canStop(d2))
      distanceRecurse(node, c1, c2, front_list);
    else
      updateFrontList(front_list, c1, c2);
  }
}

//==============================================================================
/** @brief Bounding volume test structure */
template <typename S>
struct FCL_EXPORT BVT
{
  /** @brief distance between bvs */
  S d;

  /** @brief bv indices for a pair of bvs in two models */
  int b1, b2;
};

//==============================================================================
/** @brief Comparer between two BVT */
template <typename S>
struct FCL_EXPORT BVT_Comparer
{
  bool operator() (const BVT<S>& lhs, const BVT<S>& rhs) const
  {
    return lhs.d > rhs.d;
  }
};

//==============================================================================
template <typename S>
struct FCL_EXPORT BVTQ
{
  BVTQ() : qsize(2) {}

  bool empty() const
  {
    return pq.empty();
  }

  size_t size() const
  {
    return pq.size();
  }

  const BVT<S>& top() const
  {
    return pq.top();
  }

  void push(const BVT<S>& x)
  {
    pq.push(x);
  }

  void pop()
  {
    pq.pop();
  }

  bool full() const
  {
    return (pq.size() + 1 >= qsize);
  }

  std::priority_queue<BVT<S>, std::vector<BVT<S>>, BVT_Comparer<S>> pq;

  /** @brief Queue size */
  unsigned int qsize;
};

//==============================================================================
template <typename S>
FCL_EXPORT
void distanceQueueRecurse(DistanceTraversalNodeBase<S>* node, int b1, int b2, BVHFrontList* front_list, int qsize)
{
  BVTQ<S> bvtq;
  bvtq.qsize = qsize;

  BVT<S> min_test;
  min_test.b1 = b1;
  min_test.b2 = b2;

  while(1)
  {
    bool l1 = node->isFirstNodeLeaf(min_test.b1);
    bool l2 = node->isSecondNodeLeaf(min_test.b2);

    if(l1 && l2)
    {
      updateFrontList(front_list, min_test.b1, min_test.b2);

      node->leafTesting(min_test.b1, min_test.b2);
    }
    else if(bvtq.full())
    {
      // queue should not get two more tests, recur

      distanceQueueRecurse(node, min_test.b1, min_test.b2, front_list, qsize);
    }
    else
    {
      // queue capacity is not full yet
      BVT<S> bvt1, bvt2;

      if(node->firstOverSecond(min_test.b1, min_test.b2))
      {
        int c1 = node->getFirstLeftChild(min_test.b1);
        int c2 = node->getFirstRightChild(min_test.b1);
        bvt1.b1 = c1;
        bvt1.b2 = min_test.b2;
        bvt1.d = node->BVTesting(bvt1.b1, bvt1.b2);

        bvt2.b1 = c2;
        bvt2.b2 = min_test.b2;
        bvt2.d = node->BVTesting(bvt2.b1, bvt2.b2);
      }
      else
      {
        int c1 = node->getSecondLeftChild(min_test.b2);
        int c2 = node->getSecondRightChild(min_test.b2);
        bvt1.b1 = min_test.b1;
        bvt1.b2 = c1;
        bvt1.d = node->BVTesting(bvt1.b1, bvt1.b2);

        bvt2.b1 = min_test.b1;
        bvt2.b2 = c2;
        bvt2.d = node->BVTesting(bvt2.b1, bvt2.b2);
      }

      bvtq.push(bvt1);
      bvtq.push(bvt2);
    }

    if(bvtq.empty())
      break;
    else
    {
      min_test = bvtq.top();
      bvtq.pop();

      if(node->canStop(min_test.d))
      {
        updateFrontList(front_list, min_test.b1, min_test.b2);
        break;
      }
    }
  }
}

} // namespace detail
} // namespace fcl

#endif
