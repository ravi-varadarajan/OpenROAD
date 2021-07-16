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

#include <map>
#include <memory>
#include <stack>
#include <utility>
#include <vector>

#include "openroadGeom.h"
#include "openroadUiEnums.h"

#ifndef SWIG
#include <GL/gl.h>
#endif

namespace OpenRoadUI {
// Forward Class Declarations
// class OpenRoadGLDevice ;
class GLTransform;
class GLLayer;
class GLPen;
class GLCanvas;
class GLCanvasInstShape;
class GLMotionShape;

class GLView
{
 public:
  friend class GLCanvasInstShape;
  friend class GLCanvas;

  GLView(std::string viewName);
  ~GLView();

  void setViewPortDimensions(int width, int height);
  int getViewPortWidth() const { return width_; }
  int getViewPortHeight() const { return height_; }

  void setTopCanvas(GLCanvas* cnv);
  GLCanvas* getTopCanvas() const { return topCanvas_; }

  GLPoint2D getWorldCoord(const GLPoint2D& devCoord) const;
  GLPoint2D getDeviceCoord(const GLPoint2D& worldCoord) const;

  GLRectangle transformToWorldRect(const GLRectangle& devRect) const;
  GLRectangle transformToWindowRect(const GLRectangle& worldRect) const;
  GLRectangle getVisibleArea() const;
  GLRectangle getFitVisibleArea() const { return fitVisArea_; }

  double getMinDrawableArea() const { return minDrawableArea_; }
  uint getNumShapesDrawn() const { return numShapesDrawn_; }

  int getMaxViewDepthToDraw() const { return viewDepthToDraw_; }
  int getCurrentViewDepth() const { return curViewDepth_; }

  void applyTransformation();
  bool draw();

  // View's Transformation related functions.
  void zoom(float val);
  void zoomIn();
  void zoomOut();

  // The rectangle should be provided in world co-ord.(Rubberband zoom)
  void zoomRect(const GLRectangle& rect);
  void setVisibleArea(
      const GLRectangle& rect);  // Existing scaleFactor will be used
  void translate(const GLPoint2D& transVec);
  // Key : Left, Right, Top Bot
  void panKey(ArrowKeyType key);
  void panCoord(const GLPoint2D& panVec);

  void fit();
  bool isViewFit() const { return isViewFit_; }

  // Returns the number of shapes selected in the region
  uint selectShapesAt(const GLPoint2D& devCoord,
                      bool deselectEarlier = true,
                      bool selectOneShape = false);
  uint selectShapesInRegion(const GLRectangle& serachArea,
                            bool deselectEarlier = true,
                            bool selectOneShape = false);

  void fitSelectedShapes();
  void fitHighlightedShapes();

  GLRectangle getSelectedShapesBBox() const;
  GLRectangle getHighlightedShapesBBox() const;

  uint getSelectedShapesCount() const;
  uint getHighlightedShapesCount() const;
  std::pair<uint, GLShape*> getSelectedShapeAt(uint idx)
      const;  // Index will move sequentially across Layers in SearchTree

  // When full is true even the local pixmaps will be thrown
  void refresh(bool full = false);

  // View Shape Operations
  void clearViewShapes(unsigned int shpTypeFlags);
  void clearMatchingViewShapes(std::vector<GLShape*>& shapes,
                               unsigned int shpTypeFlags);
  void clearViewShapesInRegion(const GLRectangle& region,
                               unsigned int shpTypeFlags);
#ifndef SWIG
  void addShapesOnView(const std::map<uint, std::vector<GLShape*>>& shapes,
                       bool deselectPrior = true,
                       ViewOpType opType
                       = SELECT_OBJECT);  // Highlight if false
#endif
  void addShapeOnView(GLShape* shapes,
                      uint layIdx,
                      bool deselectPrior = true,
                      ViewOpType opType = SELECT_OBJECT);  // Highlight if false
  void addMarkerAt(GLPoint2D markerLoc,
                   ORMarkerType markerType = OR_DIAMOND_MARKER,
                   GLPen* pen = nullptr);

  void getViewShapesInArea(std::vector<GLShape*>& shapes,
                           unsigned int shpTypFlags,
                           const GLRectangle& region) const;
  void getViewShapes(unsigned int shpTypeFlags, std::vector<GLShape*>& shps);
  void highlightSelectedShapes();
  bool updateMaxViewDepth(bool incr = true);  // Will be called from User

  void addVisibleRegion(const GLRectangle& region);
  void resetVisibleRegions();

  void setPen(GLPen* pen, ViewOpType opType = SELECT_OBJECT);

  void updateCanvasStack(OpenRoadUI::GLCanvas* cnv, bool pushCanvas = true);
  void resetPixmaps(bool canvasPixmap = true, bool viewPixmap = true);

  void startAnimationObject(DrawMotionShapeType motionType,
                            const GLPoint2D& startCoord);
  void startAnimationObject(GLShape* p_shp,
                            const GLPoint2D& startCoord,
                            uint shpLayer);
  bool drawMotionObjs(const GLPoint2D& curCoord);
  void stopAnimation();
  uint getMotionObjectCount() const { return motionShapes_.size(); }

  void setMouseClickedCoord(GLPoint2D clickCoord)
  {
    mousePressedAt_ = clickCoord;
  }
  void setMouseReleaseCoord(GLPoint2D clickCoord)
  {
    mouseReleasedAt_ = clickCoord;
  }

  void setHoldDrawing(bool val) { holdDrawing_ = val; }
  bool isDrawingOnHold() const { return holdDrawing_; }

  void setWorldViewParams(std::vector<uint> drawLayers,
                          std::vector<uint> selLayers);
  bool isWorldView() const { return worldView_; }
  std::vector<uint> getWorldViewDrawLayers() const
  {
    return worldViewDrawLayers_;
  }
  std::vector<uint> getWorldViewSelectLayers() const
  {
    return worldViewSelectLayers_;
  }

  std::string getViewName() const { return viewName_; }

  static GLView* getView(std::string viewName);

 private:
  int width_;
  int height_;
  std::string viewName_;  // The name of the view should be unique

  GLTransform* viewTransform_;
  double scaleFactor_;
  GLCanvas* topCanvas_;
  GLRectangle clipRect_;
  GLRectangle fitVisArea_;

  GLPixmap canvasPixmap_;
  GLPixmap viewPixmap_;

  bool canvasPixmapDirty_;
  bool viewPixmapDirty_;

  int numShapesDrawn_;
  int viewDepthToDraw_;
  int curViewDepth_;

  double minDrawableArea_;
  bool isViewFit_;  // Check If GLTansform can give you this

  // Store The Shapes like Select, highlight and marker and
  // any other shape
  SearchTree* selectedShapes_;
  SearchTree* highlightedShapes_;

  std::vector<GLMotionShape*> motionShapes_;

  std::map<GLPen*, std::vector<GLShape*>> markers_;
  std::vector<GLRectangle> viewAreasToShow_;  // If the vector is empty, full
                                              // visible area will be shown
  std::stack<GLCanvas*> curDrawingCanvas_;

  GLPen* selectPen_;
  GLPen* highlightPen_;
  GLPen* markerPen_;

  uint drawFlags_;
  bool drawingInProgress_;
  bool holdDrawing_;

  GLPoint2D mousePressedAt_;   // In Device Coords
  GLPoint2D mouseReleasedAt_;  // In Device Coords
  bool motionInProgress_;

  bool worldView_;
  std::vector<uint> worldViewDrawLayers_;
  std::vector<uint> worldViewSelectLayers_;

  bool updateCurrentViewDepth(
      bool ince
      = true);  // Will be updated in draw flow(Drawing CanvasInstShape)
  void calculateMinDrawable();
  void alignCenter();

  bool drawViewRegion(const GLRectangle& region);
  void clearRegion(const GLRectangle& rect);
  bool drawViewShapes(unsigned int shapeTypeFlags,
                      const GLRectangle& drawingRegion);
  void capturePixmap(GLPixmap& pixmap);
  void pastePixmap(GLPixmap& pixmap);
  void setOrthoProjection();

  static std::map<std::string, GLView*> _glViews;
};
}  // namespace OpenRoadUI
