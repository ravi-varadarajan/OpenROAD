/////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, OpenROAD
// All rights reserved.
//
// BSD 3-Clause License
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdlib>
#include <iostream>

#define TEST_FLAG(ARG1, ARG2) (ARG1 & ARG2)
#define SET_FLAG(ARG1, ARG2) ARG2 = (ARG1 | ARG2)
#define RESET_FLAG(ARG1, ARG2) ARG2 = (~ARG1 & ARG2)

static bool SHOW_DEBUG_MSG = std::getenv("SHOW_DEBUG_MSG") != nullptr;

#define DEBUG_PRINT(MSG) \
  if (SHOW_DEBUG_MSG)    \
    std::cout << __FILE__ << " : " << __LINE__ << " > " << MSG << std::endl;
#define DEBUG_PRINT_ALL(MSG) \
  std::cout << __FILE__ << " : " << __LINE__ << " > " << MSG << std::endl;

namespace OpenRoadUI {
#ifndef SWIG
template <class T>
struct PointerComparator
{
  bool operator()(T* arg1, T* arg2) const
  {
    auto arg1Val = reinterpret_cast<std::uintptr_t>(arg1);
    auto arg2Val = reinterpret_cast<std::uintptr_t>(arg2);
    return arg1Val < arg2Val;
  }
};
#endif
}  // namespace OpenRoadUI
