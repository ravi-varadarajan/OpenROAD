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

#include "pygui/openroadView.h"

#include <GL/gl.h>
#include <GL/glu.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <thread>

#include "pygui/openroadCanvas.h"
#include "pygui/openroadGlobals.h"
#include "pygui/openroadLayer.h"
#include "pygui/openroadMotion.h"
#include "pygui/openroadShape.h"
#include "pygui/openroadTransform.h"

namespace OpenRoadUI {
std::map<std::string, GLView*> GLView::_glViews = {};
GLView::GLView(std::string viewName)
    : width_(100),
      height_(100),
      viewName_(viewName),
      viewTransform_(nullptr),
      topCanvas_(nullptr),
      canvasPixmapDirty_(true),
      viewPixmapDirty_(true),
      numShapesDrawn_(0),
      viewDepthToDraw_(1),
      curViewDepth_(1),
      isViewFit_(true),
      selectedShapes_(nullptr),
      highlightedShapes_(nullptr),
      motionShapes_(),
      markers_(),
      viewAreasToShow_(),
      curDrawingCanvas_(),
      selectPen_(nullptr),
      highlightPen_(nullptr),
      drawFlags_(0),
      drawingInProgress_(false),
      holdDrawing_(false),
      mousePressedAt_(),
      mouseReleasedAt_(),
      motionInProgress_(false)
{
  viewTransform_ = new GLTransform();

  std::string selColor("white");
  std::string hltColor("blue");

  SET_FLAG(SELECT_VIEW_SHAPE, drawFlags_);
  SET_FLAG(HIGHLIGHT_VIEW_SHAPE, drawFlags_);
  SET_FLAG(MARKER_VIEW_SHAPE, drawFlags_);
  SET_FLAG(CANVAS_SHAPE, drawFlags_);

  selectedShapes_ = new SearchTree();
  highlightedShapes_ = new SearchTree();

  selectPen_ = new GLPen(selColor, OR_FILL_NONE_PAT);
  ColorParams hltColorParams(0.0f, 0.0f, 1.0f, 0.4f);
  highlightPen_ = new GLPen(hltColorParams, OR_FILL_SOLID_PAT);

  ColorParams mrkColorParams(1.0f, 1.0f, 0.0f);
  markerPen_ = new GLPen(mrkColorParams, OR_FILL_NONE_PAT);

  _glViews[viewName_] = this;
}

GLView::~GLView()
{
  delete selectPen_;
  delete highlightPen_;

  // Should the contained shapes be deleted. Proably not, cause we will only
  // select/highight only a canvas shape
  selectedShapes_->shapeColls_.clear();
  highlightedShapes_->shapeColls_.clear();

  delete selectedShapes_;
  delete highlightedShapes_;

  for (auto& markersData : markers_) {
    for (auto marker : markersData.second)
      delete marker;
  }
  markers_.clear();

  _glViews.erase(viewName_);
}

void GLView::setViewPortDimensions(int width, int height)
{
  width_ = width;
  height_ = height;
  resetPixmaps();
  glViewport(0, 0, width_, height_);
  setOrthoProjection();  // Should be done in the draw flow
}

void GLView::setTopCanvas(GLCanvas* cnv)
{
  topCanvas_ = cnv;
  resetPixmaps();
  uint shpTypeFlags = 0;
  SET_FLAG(SELECT_VIEW_SHAPE, shpTypeFlags);
  SET_FLAG(HIGHLIGHT_VIEW_SHAPE, shpTypeFlags);
  SET_FLAG(MARKER_VIEW_SHAPE, shpTypeFlags);
  clearViewShapes(shpTypeFlags);
}

GLPoint2D GLView::getWorldCoord(const GLPoint2D& devCoord) const
{
  if (!topCanvas_) {
    // Should we not assert here?
    return GLPoint2D();
  }
  GLint viewport[4] = {0};

  GLdouble mvmatrix[16] = {0.0}, projmatrix[16] = {0.0};

  // GLint realy=0;                      /*  OpenGL y coordinate position  */
  GLfloat winX = 0.0, winY = 0.0, winZ = 0.0;
  GLdouble wx = 0.0, wy = 0.0, wz = 0.0; /*  returned world x, y, z coords  */

  glGetIntegerv(GL_VIEWPORT, viewport);
  glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix);
  glGetDoublev(GL_PROJECTION_MATRIX, projmatrix);
  // note viewport[3] is height of window in pixels
  // realy = viewport[3] - (GLint) devY ;

  winX = (float) devCoord.x();
  winY = (float) viewport[3] - (float) devCoord.y();

  glReadPixels(
      (GLint) winX, (GLint) winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
  gluUnProject(winX, winY, winZ, mvmatrix, projmatrix, viewport, &wx, &wy, &wz);

  return GLPoint2D(wx, wy);
}

GLPoint2D GLView::getDeviceCoord(const GLPoint2D& worldCoord) const
{
  if (!topCanvas_) {
    // Should we not assert here?
    return GLPoint2D();
  }
  GLint viewport[4] = {0};

  GLdouble mvmatrix[16] = {0.0}, projmatrix[16] = {0.0};
  GLdouble wx1 = 0.0, wy1 = 0.0,
           wz1 = 0.0; /*  returned window x, y, z coords  */

  glGetIntegerv(GL_VIEWPORT, viewport);
  glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix);
  glGetDoublev(GL_PROJECTION_MATRIX, projmatrix);

  GLdouble worldX = worldCoord.x();
  GLdouble worldY = worldCoord.y();
  gluProject(
      worldX, worldY, 0.0, mvmatrix, projmatrix, viewport, &wx1, &wy1, &wz1);
  return ORPoint_t(wx1, height_ - wy1);
}

GLRectangle GLView::transformToWorldRect(const GLRectangle& devRect) const
{
  if (!topCanvas_) {
    // Should we not assert here?
    return GLRectangle();
  }
  GLint viewport[4] = {0};
  GLdouble mvmatrix[16] = {0.0}, projmatrix[16] = {0.0};

  GLint realy1 = 0, realy2 = 0; /*  OpenGL y coordinate position  */
  GLdouble wx1 = 0.0, wy1 = 0.0,
           wz1 = 0.0; /*  returned world x, y, z coords  */
  GLdouble wx2 = 0.0, wy2 = 0.0,
           wz2 = 0.0; /*  returned world x, y, z coords  */

  glGetIntegerv(GL_VIEWPORT, viewport);
  glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix);
  glGetDoublev(GL_PROJECTION_MATRIX, projmatrix);

  auto minC = devRect.ll();
  auto maxC = devRect.ur();

  // note viewport[3] is height of window in pixels
  realy1 = viewport[3] - (GLint)(minC.y());
  realy2 = viewport[3] - (GLint)(maxC.y());

  GLfloat z1, z2;
  glReadPixels(
      viewport[0], viewport[1], 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z1);
  glReadPixels(viewport[0] + viewport[2],
               viewport[1] + viewport[3],
               1,
               1,
               GL_DEPTH_COMPONENT,
               GL_FLOAT,
               &z2);

  gluUnProject((GLdouble)(minC.x()),
               (GLdouble) realy1,
               z1,
               mvmatrix,
               projmatrix,
               viewport,
               &wx1,
               &wy1,
               &wz1);
  gluUnProject((GLdouble)(maxC.x()),
               (GLdouble) realy2,
               z2,
               mvmatrix,
               projmatrix,
               viewport,
               &wx2,
               &wy2,
               &wz2);

  GLPoint2D devLL(wx1, height_ - wy1);
  GLPoint2D devUR(wx2, height_ - wy2);

  return GLRectangle(devLL.x(), devLL.y(), devUR.x(), devUR.y());
}

GLRectangle GLView::transformToWindowRect(const GLRectangle& worldRect) const
{
  if (!topCanvas_) {
    // Should we not assert here?
    return GLRectangle();
  }
  GLint viewport[4] = {0};

  GLdouble mvmatrix[16] = {0.0}, projmatrix[16] = {0.0};

  GLdouble wx1 = 0.0, wy1 = 0.0,
           wz1 = 0.0; /*  returned window x, y, z coords  */
  GLdouble wx2 = 0.0, wy2 = 0.0,
           wz2 = 0.0; /*  returned window x, y, z coords  */

  GLdouble x1 = 0.0, y1 = 0.0, z1 = 0.0, x2 = 0.0, y2 = 0.0, z2 = 0.0;
  glGetIntegerv(GL_VIEWPORT, viewport);
  glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix);
  glGetDoublev(GL_PROJECTION_MATRIX, projmatrix);

  auto minC = worldRect.ll();
  auto maxC = worldRect.ur();

  x1 = minC.x();
  y1 = minC.y();
  x2 = maxC.x();
  y2 = maxC.y();

  gluProject(x1, y1, z1, mvmatrix, projmatrix, viewport, &wx1, &wy1, &wz1);
  gluProject(x2, y2, z2, mvmatrix, projmatrix, viewport, &wx2, &wy2, &wz2);

  GLPoint2D devLL(wx1, height_ - wy1);
  GLPoint2D devUR(wx2, height_ - wy2);

  return GLRectangle(devLL.x(), devLL.y(), devUR.x(), devUR.y());
}

GLRectangle GLView::getVisibleArea() const
{
  if (!topCanvas_) {
    return GLRectangle();
  }
  GLint viewport[4] = {0};

  GLdouble mvmatrix[16] = {0.0}, projmatrix[16] = {0.0};
  GLdouble wx1 = 0.0, wy1 = 0.0, wz1 = 0.0, wx2 = 0.0, wy2 = 0.0, wz2 = 0.0;

  glGetIntegerv(GL_VIEWPORT, viewport);
  glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix);
  glGetDoublev(GL_PROJECTION_MATRIX, projmatrix);

  gluUnProject((GLdouble) viewport[0],
               (GLdouble) viewport[1],
               0.0,
               mvmatrix,
               projmatrix,
               viewport,
               &wx1,
               &wy1,
               &wz1);
  gluUnProject((GLdouble) viewport[0] + viewport[2],
               (GLdouble) viewport[1] + viewport[3],
               0.0,
               mvmatrix,
               projmatrix,
               viewport,
               &wx2,
               &wy2,
               &wz2);

  return GLRectangle(wx1, wy1, wx2, wy2);
}

void GLView::applyTransformation()
{
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  GLRectangle visArea = getVisibleArea();
  viewTransform_->applyTransform(visArea);
  calculateMinDrawable();
}

bool GLView::draw()
{
  if (topCanvas_ == nullptr)
    return false;
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  setOrthoProjection();
  glLineWidth(1.0);
  applyTransformation();
  ORRect_t visArea = getVisibleArea();
  if (viewTransform_->operationsCount() == 0)
    fitVisArea_ = getVisibleArea();
  // For Now we will support only 1 visible region on the view, which is total
  // Visible area Later we will allow user to chose the region which should be
  // drawn.
  viewAreasToShow_.clear();
  viewAreasToShow_.push_back(visArea);
  // glLineWidth(2.0) ;
  auto retVal = drawViewRegion(visArea);
  return true;
}

void GLView::zoom(float val)
{
  if (!topCanvas_)
    return;
  isViewFit_ = false;
  viewTransform_->scale(val, val);
  resetPixmaps();
  // refresh(true) ; Should be handled through repaint from python
}

void GLView::zoomIn()
{
  if (!topCanvas_)
    return;
  auto visArea = getVisibleArea();
  auto zoomArea = visArea.bloat(0.8);
  zoomRect(zoomArea);
}

void GLView::zoomOut()
{
  if (!topCanvas_)
    return;
  auto visArea = getVisibleArea();
  float bloatValue = 1.2;
  auto zoomArea = visArea.bloat(bloatValue);

  if (getTopCanvas()) {
    auto topRect = GLRectangle(getTopCanvas()->getFullBBox());
    // std::cout << "Zoom out to " << zoomArea.area() << " Fit = " <<
    // topRect.area() << std::endl;
    if (bloatValue * topRect.area() < zoomArea.area()) {
      this->fit();
      return;
    }
  }

  zoomRect(zoomArea);
}

void GLView::zoomRect(const GLRectangle& zRect)
{
  if (!topCanvas_)
    return;
  resetPixmaps();
  viewTransform_->resetTransform();

  ORRect_t curVisArea = fitVisArea_;

  ORRect_t rect = zRect;
  auto scaleRectWidth = rectWidth(rect);
  auto scaleRectHeight = rectHeight(rect);

  auto visRectWidth = rectWidth(curVisArea);
  auto visRectHeight = rectHeight(curVisArea);
  GLfloat scaleFactor
      = static_cast<GLfloat>(std::min(visRectWidth * 1.0 / scaleRectWidth,
                                      visRectHeight * 1.0 / scaleRectHeight));
  if (scaleFactor <= 1) {
    scaleFactor = 1.1;
  }

  ORPoint_t scaleRectCenter;
  bg::centroid(rect, scaleRectCenter);

  ORPoint_t visRectCenter;
  bg::centroid(curVisArea, visRectCenter);

  ORPoint_t centerDist = scaleRectCenter;
  bg::subtract_point(centerDist, visRectCenter);
  ORPoint_t panDims = scaleRectCenter;

  double xTrans = bg::get<0>(panDims) * scaleFactor - visRectCenter.get<0>();
  double yTrans = bg::get<1>(panDims) * scaleFactor - visRectCenter.get<1>();

  viewTransform_->scale(scaleFactor, scaleFactor, 1.0, false);
  viewTransform_->translate(xTrans * -1, yTrans * -1);
  isViewFit_ = false;
  // refresh(true) ; It should be called from python
}

void GLView::translate(const GLPoint2D& transVec)
{
  if (!topCanvas_)
    return;
  isViewFit_ = false;
  viewTransform_->translate(transVec.x(), transVec.y());
  resetPixmaps();
  // refresh(true) ; It Should be called from python
}

void GLView::panKey(ArrowKeyType key)
{
  if (isViewFit_ || !topCanvas_)
    return;
  auto visArea = getVisibleArea();
  auto cnvBBox = GLRectangle(topCanvas_->getFullBBox());

  GLfloat horz = rectWidth(visArea) * 0.1;
  GLfloat vert = rectHeight(visArea) * 0.1;

  auto visMinC = visArea.ll();
  auto visMaxC = visArea.ur();

  auto visLLX = visMinC.x();
  auto visLLY = visMinC.y();
  auto visURX = visMaxC.x();
  auto visURY = visMaxC.y();

  auto cnvMinC = cnvBBox.ll();
  auto cnvMaxC = cnvBBox.ur();

  auto cnvLLX = cnvMinC.x();
  auto cnvLLY = cnvMinC.y();
  auto cnvURX = cnvMaxC.x();
  auto cnvURY = cnvMaxC.y();

  switch (key) {
    case KEY_LEFT: {
      if ((visLLX - cnvLLX) <= 0)
        return;
      viewTransform_->translate(horz, 0);
      break;
    }
    case KEY_UP: {
      if ((visURY - cnvURY) >= 0)
        return;
      viewTransform_->translate(0, -1 * vert);
      break;
    }
    case KEY_RIGHT: {
      if ((visURX - cnvURX) >= 0)
        return;
      viewTransform_->translate(-1 * horz, 0);
      break;
    }
    case KEY_DOWN: {
      if ((visLLY - cnvLLY) <= 0)
        return;
      viewTransform_->translate(0, vert);
      break;
    }
    default:
      return;
  }
  resetPixmaps();
  // refresh(true) ; refresh will happen from python
}

void GLView::panCoord(const GLPoint2D& panVec)
{
  if (isViewFit_ || !topCanvas_)
    return;
  resetPixmaps();
  viewTransform_->translate(panVec.x(), panVec.y());
  // refresh(true) ;  //Refresh will happen from python
}

void GLView::fit()
{
  if (isViewFit_ || !topCanvas_)
    return;
  resetPixmaps();
  viewTransform_->resetTransform();
  isViewFit_ = true;
  // refresh(true) ;   //Refresh will happen from python
}

uint GLView::selectShapesAt(const GLPoint2D& devCoord,
                            bool deselectEarlier,
                            bool selectOneShape)
{
  if (!topCanvas_)
    return 0;
  GLPoint2D worldCoord = getWorldCoord(devCoord);
  // make a small Rectangle containing this point
  double ptx = worldCoord.x();
  double pty = worldCoord.y();
  // ORRect_t searchRect(ORPoint_t(ptx - 0.001, pty - 0.001), ORPoint_t(ptx +
  // 0.001, pty + 0.001)) ;
  GLRectangle searchRect(ptx - 0.001, pty - 0.001, ptx + 0.001, pty + 0.001);
  return selectShapesInRegion(searchRect, deselectEarlier);
}

uint GLView::selectShapesInRegion(const GLRectangle& searchArea,
                                  bool deselectEarlier,
                                  bool selectOneShape)
{
  if (!topCanvas_)
    return 0;
  if (deselectEarlier) {
    resetPixmaps(false, true);
    selectedShapes_->clearTree();
  }
  std::vector<uint> layers;

  topCanvas_->getTopLayerNode()->getChildLayerIdsRecurse(layers);
  ORRect_t searchRect = searchArea;
  uint shapesInReg = topCanvas_->popuateShapesFromRegion(
      layers, selectedShapes_, &searchRect);
  if (shapesInReg)
    resetPixmaps(false, true);
  if (selectOneShape && shapesInReg > 0) {
    bool removeAllShapes = false;
    for (auto& shpData : selectedShapes_->shapeColls_) {
      if (removeAllShapes)
        shpData.second.clear();
      else {
        removeAllShapes = true;
        auto remFrom = shpData.second.begin();
        remFrom++;
        shpData.second.remove(remFrom, shpData.second.end());
      }
    }
    return 1;
  }

  return shapesInReg;
}

void GLView::fitSelectedShapes()
{
  if (!topCanvas_)
    return;
  GLRectangle selShpsBB = getSelectedShapesBBox();
  zoomRect(selShpsBB);
}

void GLView::fitHighlightedShapes()
{
  if (!topCanvas_)
    return;
  GLRectangle selShpsBB = getHighlightedShapesBBox();
  zoomRect(selShpsBB);
}

GLRectangle GLView::getSelectedShapesBBox() const
{
  ORQuadTree<int> boundTree;

  for (auto& shapeColl : selectedShapes_->shapeColls_) {
    if (shapeColl.second.empty() == false)
      boundTree.insert(
          std::make_pair(shapeColl.second.bounds(), shapeColl.first));
  }

  if (boundTree.empty())
    return GLRectangle();
  return boundTree.bounds();
}

GLRectangle GLView::getHighlightedShapesBBox() const
{
  ORQuadTree<int> boundTree;

  for (auto& shapeColl : highlightedShapes_->shapeColls_) {
    if (shapeColl.second.empty() == false)
      boundTree.insert(
          std::make_pair(shapeColl.second.bounds(), shapeColl.first));
  }

  if (boundTree.empty())
    return GLRectangle();
  return boundTree.bounds();
}

uint GLView::getSelectedShapesCount() const
{
  if (!selectedShapes_ || !topCanvas_)
    return 0;
  uint numSelShapes = 0;
  for (auto& shapeData : selectedShapes_->shapeColls_)
    numSelShapes += shapeData.second.size();
  return numSelShapes;
}

uint GLView::getHighlightedShapesCount() const
{
  if (!highlightedShapes_ || !topCanvas_)
    return 0;
  uint numHltShapes = 0;
  for (auto& shapeData : highlightedShapes_->shapeColls_)
    numHltShapes += shapeData.second.size();
  return numHltShapes;
}

std::pair<uint, GLShape*> GLView::getSelectedShapeAt(uint idx) const
{
  if (!selectedShapes_ || !topCanvas_)
    return std::pair<int, GLShape*>(0, nullptr);
  uint reqIdx = idx;
  for (auto& shpData : selectedShapes_->shapeColls_) {
    if (reqIdx >= shpData.second.size()) {
      reqIdx -= shpData.second.size();
      continue;
    }
    auto itr = shpData.second.begin();
    for (int i = 0; i < reqIdx; ++i)
      ++itr;
    GLShape* shp = itr->second;
    return std::pair<int, GLShape*>(shpData.first, shp);
  }
  return std::pair<int, GLShape*>(0, nullptr);
}

void GLView::clearViewShapes(unsigned int shpTypeFlags)
{
  if (TEST_FLAG(SELECT_VIEW_SHAPE, shpTypeFlags) != 0)
    selectedShapes_->clearTree();
  if (TEST_FLAG(HIGHLIGHT_VIEW_SHAPE, shpTypeFlags) != 0)
    highlightedShapes_->clearTree();
  if (TEST_FLAG(MARKER_VIEW_SHAPE, shpTypeFlags) != 0) {
    for (auto& markersData : markers_) {
      for (auto marker : markersData.second)
        delete marker;
    }
    markers_.clear();
  }
  resetPixmaps(false, true);
  // refresh(false) ; //It should be called from python
}

void GLView::clearMatchingViewShapes(std::vector<GLShape*>& shapes,
                                     unsigned int shpTypeFlags)
{
  if (TEST_FLAG(SELECT_VIEW_SHAPE, drawFlags_) != 0) {
    for (auto& shp : shapes) {
      for (auto& shpData : selectedShapes_->shapeColls_)
        shpData.second.remove(std::make_pair(shp->getBBox(), shp));
    }
  } else if (TEST_FLAG(HIGHLIGHT_VIEW_SHAPE, drawFlags_) != 0) {
    for (auto& shp : shapes) {
      for (auto& shpData : highlightedShapes_->shapeColls_)
        shpData.second.remove(std::make_pair(shp->getBBox(), shp));
    }
  } else if (TEST_FLAG(MARKER_VIEW_SHAPE, drawFlags_) != 0) {
    for (auto& shpToDel : shapes) {
      for (auto& markerData : markers_) {
        auto& markersVec = markerData.second;
        markersVec.erase(
            std::remove_if(markersVec.begin(),
                           markersVec.end(),
                           [&](auto& shp) { return shpToDel == shp; }),
            markersVec.end());
      }
    }
  }
  resetPixmaps(false, true);
  // refresh(false) ; //Should be called from Python through repaint flow
}

void GLView::clearViewShapesInRegion(const GLRectangle& visReg,
                                     unsigned int shpTypeFlags)
{
  ORRect_t region = visReg;
  auto removeShapesInArea = [&](ORQuadTree<GLShape*>& tree) {
    std::vector<std::pair<ORRect_t, GLShape*>> shapesInRegion;
    // std::vector<OpenRoadShape*> shapesInRegion ;
    tree.query(bgi::intersects(region), std::back_inserter(shapesInRegion));
    tree.remove(shapesInRegion.begin(), shapesInRegion.end());
  };
  if (TEST_FLAG(SELECT_VIEW_SHAPE, shpTypeFlags) != 0) {
    for (auto& shpData : selectedShapes_->shapeColls_)
      removeShapesInArea(shpData.second);
  }
  if (TEST_FLAG(HIGHLIGHT_VIEW_SHAPE, shpTypeFlags) != 0) {
    for (auto& shpData : highlightedShapes_->shapeColls_)
      removeShapesInArea(shpData.second);
  }
  if (TEST_FLAG(MARKER_VIEW_SHAPE, shpTypeFlags) != 0) {
    ORRect_t resRect;
    for (auto& markerData : markers_) {
      auto& markerShps = markerData.second;
      markerShps.erase(std::remove_if(markerShps.begin(),
                                      markerShps.end(),
                                      [&](GLShape* shp) {
                                        return bg::covered_by(
                                            shp->getBBox().min_corner(),
                                            region);
                                      }),
                       markerShps.end());
    }
  }
  resetPixmaps(false, true);
  // refresh(false) ; //Should be done from Python's repaint flow
}

void GLView::addShapesOnView(
    const std::map<uint, std::vector<GLShape*>>& shapes,
    bool deselectPrior,
    ViewOpType opType)
{
  if (opType == SELECT_OBJECT) {
    if (deselectPrior)
      selectedShapes_->clearTree();
    for (auto& shpData : shapes) {
      uint layIdx = shpData.first;
      for (auto shp : shpData.second)
        selectedShapes_->shapeColls_[layIdx].insert(
            std::make_pair(shp->getBBox(), shp));
    }
  } else if (opType == HIGHLIGHT_OBJECT) {
    for (auto& shpData : shapes) {
      uint layIdx = shpData.first;
      for (auto shp : shpData.second)
        highlightedShapes_->shapeColls_[layIdx].insert(
            std::make_pair(shp->getBBox(), shp));
    }
  } else if (opType == LOCATE_OBJECT) {
    for (auto& shpData : shapes) {
      for (auto& shp : shpData.second)
        addMarkerAt(shp->getBoundingBox().ll());
    }
  }
  for (auto& shpData : shapes) {
    if (shpData.second.empty() == false) {
      resetPixmaps(false, true);
      return;
    }
  }
  // refresh(false) ; //Should be done from Python's repaint flow
}

void GLView::addShapeOnView(GLShape* shp,
                            uint layIdx,
                            bool deselectPrior,
                            ViewOpType opType)
{
  switch (opType) {
    case SELECT_OBJECT: {
      if (deselectPrior)
        selectedShapes_->clearTree();
      selectedShapes_->addShapeInTree(layIdx, shp);
      resetPixmaps(false, true);
    } break;
    case HIGHLIGHT_OBJECT: {
      highlightedShapes_->addShapeInTree(layIdx, shp);
      resetPixmaps(false, true);
    } break;
    case LOCATE_OBJECT: {
      addMarkerAt(shp->getBoundingBox().ll());
      resetPixmaps(false, true);
    } break;
  }
}

void GLView::addMarkerAt(GLPoint2D markerLoc,
                         ORMarkerType markerType,
                         GLPen* pen)
{
  GLShape* shp = new GLMarkerShape(markerLoc, markerType);
  resetPixmaps(false, true);
  if (pen == nullptr)
    pen = markerPen_;
  if (markers_.find(pen) == markers_.end())
    markers_[pen] = std::vector<GLShape*>();
  markers_[pen].push_back(shp);
}

void GLView::getViewShapesInArea(std::vector<GLShape*>& shapesInRegion,
                                 unsigned int shpTypFlags,
                                 const GLRectangle& visReg) const
{
  ORRect_t region = visReg;
  std::vector<std::pair<ORRect_t, GLShape*>> shapesInVisArea;
  if (TEST_FLAG(SELECT_VIEW_SHAPE, shpTypFlags) != 0) {
    for (auto& shpData : selectedShapes_->shapeColls_)
      shpData.second.query(bgi::intersects(region),
                           std::back_inserter(shapesInVisArea));
  }
  if (TEST_FLAG(HIGHLIGHT_VIEW_SHAPE, shpTypFlags) != 0) {
    for (auto& shpData : highlightedShapes_->shapeColls_)
      shpData.second.query(bgi::intersects(region),
                           std::back_inserter(shapesInVisArea));
  }
  if (TEST_FLAG(MARKER_VIEW_SHAPE, shpTypFlags) != 0) {
    for (auto& markerData : markers_) {
      for (auto& shp : markerData.second) {
        GLMarkerShape* markerShp = dynamic_cast<GLMarkerShape*>(shp);
        ORPoint_t markerLoc = markerShp->getMarkerLoc();
        if (bg::covered_by(markerLoc, region))
          shapesInRegion.push_back(shp);
      }
    }
  }
  for (auto& treeItem : shapesInVisArea)
    shapesInRegion.push_back(treeItem.second);
}

void GLView::getViewShapes(unsigned int shpTypeFlags,
                           std::vector<GLShape*>& shapes)
{
  if (TEST_FLAG(SELECT_VIEW_SHAPE, drawFlags_) != 0) {
    for (auto& shpData : selectedShapes_->shapeColls_)
      std::for_each(shpData.second.begin(),
                    shpData.second.end(),
                    [&](auto& treeData) { shapes.push_back(treeData.second); });
  } else if (TEST_FLAG(HIGHLIGHT_VIEW_SHAPE, drawFlags_) != 0) {
    for (auto& shpData : highlightedShapes_->shapeColls_)
      std::for_each(shpData.second.begin(),
                    shpData.second.end(),
                    [&](auto& treeData) { shapes.push_back(treeData.second); });
  } else if (TEST_FLAG(MARKER_VIEW_SHAPE, drawFlags_) != 0) {
    for (auto& markerData : markers_)
      std::copy(markerData.second.begin(),
                markerData.second.end(),
                std::back_inserter(shapes));
  }
}

void GLView::highlightSelectedShapes()
{
  for (auto& shpData : selectedShapes_->shapeColls_) {
    uint layIdx = shpData.first;
    highlightedShapes_->shapeColls_[layIdx].insert(shpData.second.begin(),
                                                   shpData.second.end());
  }
  resetPixmaps(false, true);
  // refresh(false) ; //Should be done from python's repaint flow
}

bool GLView::updateMaxViewDepth(bool incr)
{
  if (!topCanvas_)
    return false;
  if (incr == true) {
    if (topCanvas_->getCanvasDepth() > viewDepthToDraw_) {
      viewDepthToDraw_++;
      resetPixmaps();
      return true;  // As true is returned python should issue a repaint
    }
  } else if (viewDepthToDraw_ > 1) {
    viewDepthToDraw_--;
    resetPixmaps();
    return true;  // As true is returned python should issue a repaint
  }
  // resetPixmaps() ;
  return false;
}

bool GLView::updateCurrentViewDepth(bool incr)
{
  if (!topCanvas_)
    return false;
  if (incr)
    curViewDepth_ += 1;
  else
    curViewDepth_ -= 1;
  // resetPixmaps() ; This would be called from inside the canvas Inst Shape
  // drawing no need to resetPixmap from here...
  return true;
}

void GLView::addVisibleRegion(const GLRectangle& region)
{
  ORRect_t visRegion = region;
  viewAreasToShow_.erase(std::remove_if(viewAreasToShow_.begin(),
                                        viewAreasToShow_.end(),
                                        [&](const ORRect_t& rect) {
                                          return bg::intersects(rect,
                                                                visRegion);
                                        }),
                         viewAreasToShow_.end());
  viewAreasToShow_.push_back(visRegion);
  resetPixmaps();
  // refresh(true) ; //Caller should call refresh if needed
}

void GLView::resetVisibleRegions()
{
  viewAreasToShow_.clear();
  resetPixmaps();
  refresh(true);
}

void GLView::setPen(GLPen* pen, ViewOpType opType)
{
  if (opType == SELECT_OBJECT)
    selectPen_ = pen;
  else if (opType == HIGHLIGHT_OBJECT)
    highlightPen_ = pen;
  else
    markerPen_ = pen;
  viewPixmapDirty_ = true;
  resetPixmaps();
  // refresh(false) ; //Should be called from python's repaint
}

void GLView::updateCanvasStack(OpenRoadUI::GLCanvas* cnv, bool push)
{
  // TBD : Should the current canvas be tested for being in the stack, if so,
  // wont it give rise to infinite-recursive drawing?
  if (push)
    curDrawingCanvas_.push(cnv);
  else
    curDrawingCanvas_.pop();
}

void GLView::resetPixmaps(bool clearCanvasPixmap, bool clearViewPixmap)
{
  if (clearCanvasPixmap) {
    canvasPixmapDirty_ = true;
    canvasPixmap_.clearPixmap();
  }
  if (clearViewPixmap) {
    viewPixmapDirty_ = true;
    viewPixmap_.clearPixmap();

    // for (auto marker : markers_)
    //    delete marker ;
    // markers_.clear() ;
  }
}

void GLView::refresh(bool full)
{
  if (holdDrawing_)
    return;
  if (full) {
    canvasPixmapDirty_ = true;
    viewPixmapDirty_ = true;
  }
  // this->draw() ; //Refresh will be called from Python
}

void GLView::startAnimationObject(DrawMotionShapeType motionType,
                                  const GLPoint2D& startCoord)
{
  static GLPen motionPen("white", OR_FILL_NONE_PAT);
  GLPoint2D motionStartPt = getWorldCoord(startCoord);
  GLMotionShape* shp = nullptr;
  if (motionType == RUBBERBAND_RECT)
    shp = new GLRubberbandRect(motionStartPt, &motionPen);
  else if (motionType == RUBBERBAND_LINE)
    shp = new GLRubberbandLine(motionStartPt, &motionPen);
  else if (motionType == MOTION_OR_SHAPE) {
    if (!topCanvas_)
      return;
    // make a small Rectangle containing this point
    double ptx = motionStartPt.x();
    double pty = motionStartPt.y();
    GLRectangle searchArea(ptx - 0.001, pty - 0.001, ptx + 0.001, pty + 0.001);
    std::vector<uint> layers;
    topCanvas_->getTopLayerNode()->getChildLayerIdsRecurse(layers);
    ORRect_t searchRect = searchArea;
    SearchTree shapesInArea;
    uint shapesInReg = topCanvas_->popuateShapesFromRegion(
        layers, &shapesInArea, &searchRect);
    if (shapesInReg == 0)
      return;
    GLShape* shpToMove = nullptr;
    uint shpLayer;
    for (auto& shpData : shapesInArea.shapeColls_) {
      if (shpData.second.size() > 0) {
        shpLayer = shpData.first;
        shpToMove = shpData.second.begin()->second;
      }
    }
    if (shpToMove == nullptr)
      return;
    shp = new GLAnimationShape(
        motionStartPt, shpToMove, static_cast<int>(shpLayer));
  }
  if (shp)
    motionShapes_.push_back(shp);
  motionInProgress_ = true;
}

void GLView::stopAnimation()
{
  for (auto& motionShp : motionShapes_)
    delete motionShp;
  motionShapes_.clear();
  motionInProgress_ = false;
}

// static
GLView* GLView::getView(std::string viewName)
{
  static bool debuggerAttched = false;
  const char* attachDebugger = std::getenv("OPENROAD_DEBUG");
  if (!debuggerAttched && attachDebugger != nullptr) {
    pid_t procId = getpid();
    std::string debugCmd = "ddd attach " + std::to_string(procId) + " &";
    (void) std::system(debugCmd.c_str());
    std::this_thread::sleep_for(std::chrono::microseconds(5000000));
    debuggerAttched = true;
  }

  auto itr = _glViews.find(viewName);
  if (itr != _glViews.end())
    return (*itr).second;
  auto view = new GLView(viewName);
  return view;
}

void GLView::calculateMinDrawable()
{
  if (!topCanvas_) {
    minDrawableArea_ = 0;
    return;
  }
  GLPoint2D startPoint = getWorldCoord(GLPoint2D(0, 0));
  GLPoint2D endPoint = getWorldCoord(GLPoint2D(2, 2));

  auto minLengthX = abs(endPoint.x() - startPoint.x());
  auto minLengthY = abs(endPoint.y() - startPoint.y());

  minDrawableArea_ = minLengthX * minLengthY;
}

bool GLView::drawViewRegion(const GLRectangle& drawVArea)
{
  if (!topCanvas_)
    return false;
  ORRect_t drawArea = drawVArea;
  drawingInProgress_ = true;

  glEnable(GL_POLYGON_STIPPLE);
  glLogicOp(GL_COPY);

  numShapesDrawn_ = 0;
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE);
  glLogicOp(GL_COPY);
  if (!viewPixmapDirty_) {
    pastePixmap(viewPixmap_);
    return true;
  }
  if (!canvasPixmapDirty_) {
    DEBUG_PRINT("pasting View Pixmap and drawing View Shapes...");
    std::stringstream oss;
    uint shpFlags = 0;
    SET_FLAG(SELECT_VIEW_SHAPE, shpFlags);
    SET_FLAG(HIGHLIGHT_VIEW_SHAPE, shpFlags);
    SET_FLAG(MARKER_VIEW_SHAPE, shpFlags);
    for (auto& curArea : viewAreasToShow_) {
      ORRect_t curVisArea = curArea;
      ORRect_t resDrawArea;
      if (bg::intersection(drawArea, curVisArea, resDrawArea)) {
        drawViewShapes(shpFlags, resDrawArea);
      }
    }
    DEBUG_PRINT("Capturing View Pixmap...");
    viewPixmapDirty_ = false;
    pastePixmap(canvasPixmap_);
    glFlush();
    glFinish();
    this->capturePixmap(viewPixmap_);
    return true;
  } else {
    for (auto& curArea : viewAreasToShow_) {
      ORRect_t curVisArea = curArea;
      ORRect_t resDrawArea;
      if (bg::intersection(drawArea, curVisArea, resDrawArea)) {
        // std::cout << "Drawing Canvas Region : "<< resDrawArea<<std::endl ;
        numShapesDrawn_ = topCanvas_->drawCanvas(this, resDrawArea);
      }
    }
    DEBUG_PRINT("Capturing Canvas Pixmap...");
    this->capturePixmap(canvasPixmap_);
    canvasPixmapDirty_ = false;
  }
  std::stringstream oss;
  oss << "Number of shapes drawn in canvas = " << numShapesDrawn_;
  oss.clear();
  uint shpFlags = 0;
  SET_FLAG(SELECT_VIEW_SHAPE, shpFlags);
  SET_FLAG(HIGHLIGHT_VIEW_SHAPE, shpFlags);
  SET_FLAG(MARKER_VIEW_SHAPE, shpFlags);
  for (auto& curArea : viewAreasToShow_) {
    ORRect_t curVisArea = curArea;
    ORRect_t resDrawArea;
    if (bg::intersection(drawArea, curVisArea, resDrawArea)) {
      drawViewShapes(shpFlags, resDrawArea);
    }
  }
  DEBUG_PRINT("Capturing View Pixmap...");
  this->capturePixmap(viewPixmap_);
  viewPixmapDirty_ = false;

  return true;
}

void GLView::clearRegion(const GLRectangle& rect)
{
  glClear(GL_COLOR_BUFFER_BIT);
  glClear(GL_DEPTH_BUFFER_BIT);
  glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glEnable(GL_POLYGON_STIPPLE);
  auto pattern = GLPen::getPenBytePattern(OR_FILL_SOLID_PAT);
  glPolygonStipple(pattern);

  auto llx = std::min(rect.ll().x(), rect.ur().x());
  auto lly = std::min(rect.ll().y(), rect.ur().y());
  auto urx = std::max(rect.ll().x(), rect.ur().x());
  auto ury = std::max(rect.ll().y(), rect.ur().y());
  glRectf(llx, lly, urx, ury);
  glBegin(GL_LINE_LOOP);
  glVertex2f(llx, lly);
  glVertex2f(urx, lly);
  glVertex2f(urx, ury);
  glVertex2f(llx, ury);
  glEnd();
}

bool GLView::drawMotionObjs(const GLPoint2D& curCoord)
{
  static int animationSkip = 0;
  if (!topCanvas_ || motionShapes_.empty())
    return false;

  if (animationSkip % 8 != 0) {
    animationSkip++;
    return false;
  }
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  setOrthoProjection();
  glLineWidth(2.0);
  this->applyTransformation();

  glEnable(GL_POLYGON_STIPPLE);
  glLogicOp(GL_COPY);

  numShapesDrawn_ = 0;
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE);

  animationSkip = 0;
  auto visArea = getVisibleArea();
  auto curWorldCoord = getWorldCoord(curCoord);
  for (auto& shp : motionShapes_)
    shp->draw(this, curWorldCoord);
  pastePixmap(viewPixmap_);
  glFlush();
  glFinish();

  return true;
}

bool GLView::drawViewShapes(unsigned int shpTypeFlags,
                            const GLRectangle& region)
{
  ORRect_t drawRegion = region;
  auto drawShapesInQTree = [&](int shpIdx,
                               ORQuadTree<GLShape*>& tree,
                               GLPen* pen,
                               bool drawOutline) {
    std::vector<std::pair<ORRect_t, GLShape*>> shapesInRegion;
    tree.query(bgi::intersects(drawRegion), std::back_inserter(shapesInRegion));
    for (auto& treeItem : shapesInRegion) {
      GLShape* shp = treeItem.second;
      shp->draw(this, shpIdx, pen, nullptr, false, drawOutline);
    }
  };
  if (TEST_FLAG(SELECT_VIEW_SHAPE, shpTypeFlags) != 0) {
    selectPen_->setGLPenContext();
    for (auto& shapeData : selectedShapes_->shapeColls_) {
      drawShapesInQTree(shapeData.first, shapeData.second, selectPen_, true);
    }
  } else {
  }
  if (TEST_FLAG(HIGHLIGHT_VIEW_SHAPE, shpTypeFlags) != 0) {
    for (auto& shapeData : highlightedShapes_->shapeColls_) {
      drawShapesInQTree(
          shapeData.first, shapeData.second, highlightPen_, false);
    }
  }
  if (TEST_FLAG(MARKER_VIEW_SHAPE, shpTypeFlags) != 0) {
    for (auto& markerData : markers_) {
      auto pen = markerData.first;
      pen->setGLPenContext();
      for (auto& markerShp : markerData.second)
        markerShp->draw(this, -1);
    }
  }
  return true;
}

void GLView::capturePixmap(GLPixmap& pixmap)
{
  pixmap.clearPixmap();
  pixmap.initPixmap(width_, height_);
  // Should the buffer from where the pixmap is captured should be taken as
  // input to the function?
  glReadBuffer(GL_BACK);

  glPixelStorei(GL_PACK_ROW_LENGTH, width_);
  glPixelStorei(GL_PACK_SKIP_ROWS, 0);
  glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  glFlush();
  glFinish();

  glReadPixels(0, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE, pixmap.bits_);
}

void GLView::pastePixmap(GLPixmap& pixmap)
{
  glDrawPixels((int) pixmap.width_,
               (int) pixmap.height_,
               GL_RGB,
               GL_UNSIGNED_BYTE,
               pixmap.bits_);
}

void GLView::setOrthoProjection()
{
  double ortho_w = width_;
  double ortho_h = height_;

  double aspect = ortho_w / ortho_h;

  GLCanvas* cnv = topCanvas_;

  ORRect_t cnvBBox = cnv->getFullBBox();
  clipRect_ = bloatRect(cnvBBox, 1.05);
  ORRect_t clipRect = clipRect_;

  ORRect_t clipOrtho = clipRect_;

  auto minC = clipRect.min_corner();
  auto maxC = clipRect.max_corner();
  auto rectDims = maxC;

  bg::subtract_point(rectDims, minC);
  auto width = bg::get<0>(rectDims);
  auto height = bg::get<1>(rectDims);

  if (width > height * aspect) {
    // alter the height
    double newHeight = width / aspect;
    double delta = (newHeight - height) / 2.0;

    ORPoint_t newLL(bg::get<0>(minC), bg::get<1>(minC) - delta);
    ORPoint_t newUR(bg::get<0>(maxC), bg::get<1>(maxC) + delta);

    clipOrtho = ORRect_t(newLL, newUR);
  } else {
    // alter the width
    double newWidth = height * aspect;
    double delta = (newWidth - width) / 2.0;

    ORPoint_t newLL(bg::get<0>(minC) - delta, bg::get<1>(minC));
    ORPoint_t newUR(bg::get<0>(maxC) + delta, bg::get<1>(maxC));

    clipOrtho = ORRect_t(newLL, newUR);
  }
  auto minC1 = clipOrtho.min_corner();
  auto maxC1 = clipOrtho.max_corner();

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  auto left = bg::get<0>(minC1);
  auto right = bg::get<0>(maxC1);
  auto bottom = bg::get<1>(minC1);
  auto top = bg::get<1>(maxC1);
  std::stringstream oss;
  oss << "Setting OrthoGraphic Projection as : " << left << ", " << right
      << ", " << bottom << ", " << top;
  DEBUG_PRINT(oss.str());
  oss.clear();
  gluOrtho2D(left, right, bottom, top);

  // Reset now to Model View matrix
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}
}  // namespace OpenRoadUI
