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

#ifndef SWIG
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include "../include/pygui/openroadGlobals.h"
#endif

#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../include/pygui/openroadGeom.h"
#include "../include/pygui/openroadUiEnums.h"

namespace OpenRoadUI {
// Forward Class Declarations
class GLLayer;
class GLPen;
class GLShape;
class GLCanvasInstShape;
class GLCanvas;
class GLView;

class GLCanvas
{
 public:
  friend GLCanvasInstShape;

  GLCanvas(GLLayer* topLayerNode, const char* cnvName);
  ~GLCanvas();
  // Canvas Populate Functions
  bool addShape(GLShape* shp, uint layerIdx);
  bool removeShape(GLShape* shp, uint layerIdx, bool updateBBox = false);
  uint clearCanvasLayer(int layerIdx = -1, bool destroyShapes = false);
  void setBlockOp(bool val);

#ifndef SWIG
  void setUserData(void* uData) { cnvUserData_ = uData; }
  void* getUserData() const { return cnvUserData_; }
#endif

  // Getters
  std::string getCanvasName() const { return canvasName_; }
  uint getCanvasDepth() const { return canvasDepth_; }

  GLLayer* getTopLayerNode() const { return topLayerTreeNode_; }

  // Layer Index = -1, indicates all the layers of the canvas
  uint getShapeCountInLayer(int layerIdx) const;
  uint getShapeCountFromAllLayers() const;

  // If the layer vector is empty,use all the layers of the canvas
#ifndef SWIG
  ORRect_t getCanvasBBox(const std::vector<uint>& includeLayers,
                         bool useAllLayers = false) const;
  ORRect_t getFullBBox() const;
#endif
  // For Python
  GLRectangle getCanvasBBoxPy(const std::vector<uint>& includeLayers,
                              bool useAllLayers = false) const;
  GLRectangle getFullBBoxPy() const;

  // Query Functions
  // Wghen Search Region is Null, search the whole canvas bbox
#ifndef SWIG
  GLShape* findFirstMatchingShape(const std::vector<uint>& searchLayers,
                                  void* matchingUserData,
                                  ORRect_t* searchRegion = nullptr) const;
  void findShapesInRegion(const std::vector<uint>& searchLayers,
                          std::vector<GLShape*>& shapesInRegion,
                          ORRect_t* searchRegion = nullptr) const;
  uint popuateShapesFromRegion(const std::vector<uint>& searchLayers,
                               SearchTree* shapeColl,
                               ORRect_t* searchRegion = nullptr) const;
#endif
  std::vector<GLShape*> findShapesInRegionPy(
      const std::vector<uint>& searchLayers,
      GLRectangle* searchRegion = nullptr) const;

#ifndef SWIG
  uint drawCanvas(GLView* view, const ORRect_t& visArea);
#endif
  unsigned int drawCanvasPy(GLView* view, GLRectangle visArea);

  void markOutlineLayer(int layerIdx) { outlineLayer_ = layerIdx; }
  int getOutlineLayer() const { return outlineLayer_; }

  static GLCanvas* getCanvas(const char* canvasName);
  static GLCanvas* getDummyCanvas();

 private:
  std::string canvasName_;
  GLLayer* topLayerTreeNode_;  // Why do we need this?
  void* cnvUserData_;
  std::unordered_map<GLCanvas*, int> masterCanvases_;

  std::set<uint> layersWithCanvasInsts_;

  GLRectangle* canvasBBox_;

  bool canvasBBoxUpToDate_;
  int canvasDepth_;
  SearchTree* shapeColls_;  // Key : Layer Index, Value : Shapes in the Quad
                            // Tree For The Canvas

  bool blockOp_;
  int outlineLayer_;

#ifndef SWIG
  uint drawLayer(int layerIdx,
                 const ORRect_t& visArea,
                 GLView* view,
                 bool propogateLayer = true);
#endif

  static GLCanvas* getDummyEyeCanvas();

  static std::unordered_map<std::string, GLCanvas*> _allCanvases;
};
}  // namespace OpenRoadUI
