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

#include <string>
#include <unordered_map>

//#include "openroad/OpenRoad.hh"
#include "opendb/db.h"

namespace OpenRoadUI {
// It will store the Base Canvases for Different Blocks, and Tech and Design
// Layers and It will have the containers to store Selected and Highlighted
// shapes in addition to any markers and/or rulers. Any motion object created
// during the draw flow will also be stored here...

class GLShape;
class OpenRoadLayoutContext
{
 public:
  typedef std::unordered_map<odb::dbDatabase*, OpenRoadLayoutContext*>
      ContextColl_t;

  void populateLayers() const;
  void populateCanvases() const;

  static std::string getShapeInfo(
      const GLShape* p_shp,
      uint layerIdx,
      int dbId);  // Get The Shape Info which is present in Layer layerIdx
  static OpenRoadLayoutContext* getContextForDb(odb::dbDatabase* p_db);
  static void deleteAllContexts();

 private:
  OpenRoadLayoutContext(odb::dbDatabase* p_orDb);
  ~OpenRoadLayoutContext();

  void populateDesignLayers() const;
  void populateTechLayers() const;

  odb::dbDatabase* p_ORDb_;

  static ContextColl_t _allContexts;
};
}  // namespace OpenRoadUI
