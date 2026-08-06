/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  Copyright (c) 2020, Toyota Research Institute, Inc.
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
 *   * Neither the name of the copyright holder nor the names of its
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

// fcl_distance: hand-written stand-in for the CMake-generated fcl/config.h,
// so that the extraction needs no configure step.
//
// FCL_HAVE_OCTOMAP is 0: this extraction does not carry the OcTree geometry
// or the octree traversal nodes, so it behaves exactly like an upstream FCL
// 0.7.0 build configured with -DFCL_WITH_OCTOMAP=OFF.  Do not define it to 1
// without also adding fcl/geometry/octree/** and
// fcl/narrowphase/detail/traversal/octree/**.

#ifndef FCL_CONFIG_H_
#define FCL_CONFIG_H_

#define FCL_VERSION "0.7.0"
#define FCL_MAJOR_VERSION 0
#define FCL_MINOR_VERSION 7
#define FCL_PATCH_VERSION 0

#ifndef FCL_HAVE_SSE
  #define FCL_HAVE_SSE 0
#endif

#ifndef FCL_HAVE_OCTOMAP
  #define FCL_HAVE_OCTOMAP 0
#endif

#ifndef FCL_ENABLE_PROFILING
  #define FCL_ENABLE_PROFILING 0
#endif

// Detect the operating systems
#if defined(__APPLE__)
  #define FCL_OS_MACOS
#elif defined(__gnu_linux__)
  #define FCL_OS_LINUX
#elif defined(_WIN32)
  #define FCL_OS_WINDOWS
#endif

// Detect the compiler
#if defined(__clang__)
  #define FCL_COMPILER_CLANG
#elif defined(__GNUC__) || defined(__GNUG__)
  #define FCL_COMPILER_GCC
#elif defined(_MSC_VER)
  #define FCL_COMPILER_MSVC
#endif

#endif  // FCL_CONFIG_H_
