/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  Copyright (c) 2026, fcl_distance port authors
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

// fcl_distance: static replacement for the CMake-generated fcl/export.h.
// The library is header-only, so all shared-library visibility macros
// expand to nothing; the deprecation macros keep their meaning.

#ifndef FCL_EXPORT_H_
#define FCL_EXPORT_H_

#define FCL_EXPORT
#define FCL_NO_EXPORT

#ifndef FCL_DEPRECATED
  #if defined(_MSC_VER)
    #define FCL_DEPRECATED __declspec(deprecated)
  #elif defined(__GNUC__) || defined(__clang__)
    #define FCL_DEPRECATED __attribute__((__deprecated__))
  #else
    #define FCL_DEPRECATED
  #endif
#endif

#ifndef FCL_DEPRECATED_EXPORT
  #define FCL_DEPRECATED_EXPORT FCL_EXPORT FCL_DEPRECATED
#endif

#ifndef FCL_DEPRECATED_NO_EXPORT
  #define FCL_DEPRECATED_NO_EXPORT FCL_NO_EXPORT FCL_DEPRECATED
#endif

#endif  // FCL_EXPORT_H_
