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

#include "pygui/openroadShape.h"

#include <GL/gl.h>

#include <algorithm>
#include <cassert>
#include <sstream>
#include <utility>
#include <vector>

#include "pygui/openroadCanvas.h"
#include "pygui/openroadGlobals.h"
#include "pygui/openroadLayer.h"
#include "pygui/openroadLayoutContext.h"
#include "pygui/openroadTransform.h"
#include "pygui/openroadView.h"

namespace OpenRoadUI {
GLShape::GLShape() : userData_(nullptr)
{
}

GLRectangle GLShape::getBoundingBox() const
{
  return this->getBBox();
}

std::string GLShape::getShapeInfo(uint layerIdx, int dbId) const
{
  return OpenRoadLayoutContext::getShapeInfo(this, layerIdx, dbId);
}

// static
void GLShape::destroyShape(GLShape* p_shp)
{
  delete p_shp;
}

GLCompositeShape::GLCompositeShape(const std::vector<GLShape*>& shapes)
    : GLShape()
{
  std::copy(shapes.begin(), shapes.end(), std::back_inserter(shapeCollection_));
  ORQuadTree<int> boundTree;
  int shpIdx = 0;
  for (auto& shp : shapeCollection_)
    boundTree.insert(std::make_pair(shp->getBBox(), shpIdx++));

  if (boundTree.empty() == false)
    compositeBBox_ = boundTree.bounds();
}

uint GLCompositeShape::draw(GLView* view,
                            int shpLayer,
                            GLPen* pen,
                            ORPoint_t* drawAt,
                            bool drawAnimate,
                            bool drawOutlineOnly)
{
  // TBD : Handle drawAnimate
  uint numShapesDrawn = 0;

  if (pen)
    pen->setGLPenContext();
  if (drawAt) {
    glPushMatrix();
    glTranslatef(
        boost::geometry::get<0>(*drawAt), boost::geometry::get<1>(*drawAt), 0);
  }
  auto minC = compositeBBox_.min_corner();
  auto maxC = compositeBBox_.max_corner();
  auto llx = boost::geometry::get<0>(minC);
  auto lly = boost::geometry::get<1>(minC);
  auto urx = boost::geometry::get<0>(maxC);
  auto ury = boost::geometry::get<0>(maxC);

  auto minDrawArea = view->getMinDrawableArea();

  if (boost::geometry::area(compositeBBox_) <= minDrawArea) {
    glBegin(GL_POINTS);
    glVertex2f(llx, lly);
    glEnd();
    ++numShapesDrawn;
  } else {
    if (!drawOutlineOnly) {
      for (auto& shp : shapeCollection_)
        numShapesDrawn += shp->draw(
            view, shpLayer, nullptr, nullptr, drawAnimate, drawOutlineOnly);
    }
    if (drawOutlineOnly == false)
      glRectf(llx, lly, urx, ury);

    // Now draw the rectangle for the boundary.
    glBegin(GL_LINE_LOOP);
    glVertex2f(llx, lly);
    glVertex2f(urx, lly);
    glVertex2f(urx, ury);
    glVertex2f(llx, ury);
    glEnd();
    numShapesDrawn += 1;
  }
  if (drawAt)
    glPopMatrix();
  return numShapesDrawn;
}

GLShape* GLCompositeShape::clone() const
{
  GLShape* shp = new GLCompositeShape(shapeCollection_);
  shp->setUserData(getUserData());
  return shp;
}

GLDbShape::GLDbShape(uint dbId) : GLShape(), dbId_(dbId)
{
}

ORRect_t GLDbShape::getBBox() const
{
  // TBD
  return ORRect_t();
}

uint GLDbShape::draw(GLView* view,
                     int shpLayer,
                     GLPen* pen,
                     ORPoint_t* drawAt,
                     bool drawAnimate,
                     bool drawOutlineOnly)
{
  // TBD
  return 0;
}

GLShape* GLDbShape::clone() const
{
  GLShape* shp = new GLDbShape(dbId_);
  shp->setUserData(getUserData());
  return shp;
}

GLRectShape::GLRectShape(const GLRectangle& rect) : GLShape(), rectShp_(rect)
{
}

GLRectShape::GLRectShape(const ORRect_t& rect) : GLShape(), rectShp_(rect)
{
}

uint GLRectShape::draw(GLView* view,
                       int shpLayer,
                       GLPen* pen,
                       ORPoint_t* drawAt,
                       bool drawAnimate,
                       bool drawOutlineOnly)
{
  uint numShapesDrawn = 0;

  (void) drawOutlineOnly;
  (void) drawAnimate;  // TBD : Handle Animation
  if (pen != nullptr)
    pen->setGLPenContext();
  if (drawAt != nullptr) {
    glPushMatrix();
    glTranslatef(
        boost::geometry::get<0>(*drawAt), boost::geometry::get<1>(*drawAt), 0);
  }
  auto minC = rectShp_.min_corner();
  auto maxC = rectShp_.max_corner();
  auto llx = boost::geometry::get<0>(minC);
  auto lly = boost::geometry::get<1>(minC);
  auto urx = boost::geometry::get<0>(maxC);
  auto ury = boost::geometry::get<1>(maxC);
  auto minDrawArea = view->getMinDrawableArea();

  if (boost::geometry::area(rectShp_) <= minDrawArea) {
    glBegin(GL_POINTS);
    glVertex2f(llx, lly);
    glEnd();
    numShapesDrawn += 1;
  } else {
    if (!drawOutlineOnly)
      glRectf(llx, lly, urx, ury);

    // Now draw the rectangle for the boundary.
    glBegin(GL_LINE_LOOP);
    glVertex2f(llx, lly);
    glVertex2f(urx, lly);
    glVertex2f(urx, ury);
    glVertex2f(llx, ury);
    glEnd();
    numShapesDrawn += 1;
    std::stringstream oss;
    oss << "Drawn Rect Shape " << this;
    DEBUG_PRINT(oss.str());
  }
  if (drawAt != nullptr)
    glPopMatrix();
  return numShapesDrawn;
}

GLShape* GLRectShape::clone() const
{
  GLShape* shp = new GLRectShape(rectShp_);
  shp->setUserData(getUserData());
  return shp;
}

// static
GLShape* GLRectShape::getGLRectShape(const GLRectangle& rect)
{
  GLShape* shp = new GLRectShape(rect);
  return shp;
}

GLSegmentShape::GLSegmentShape(const GLSegment& seg,
                               float lineWidth,
                               bool smooth,
                               bool dashed)
    : GLShape(),
      segment_(seg),
      lineWidth_(lineWidth),
      smooth_(smooth),
      dashed_(dashed)
{
}

GLSegmentShape::GLSegmentShape(const ORSegment_t& seg,
                               float lineWidth,
                               bool smooth,
                               bool dashed)
    : GLShape(),
      segment_(seg),
      lineWidth_(lineWidth),
      smooth_(smooth),
      dashed_(dashed)
{
}

ORRect_t GLSegmentShape::getBBox() const
{
  double x0 = boost::geometry::get<0, 0>(segment_);
  double y0 = boost::geometry::get<0, 1>(segment_);
  double x1 = boost::geometry::get<1, 0>(segment_);
  double y1 = boost::geometry::get<1, 1>(segment_);
  GLRectangle rect(x0, y0, x1, y1);
  ORRect_t fixRect = rect.getRectWithFixedOrientation();

  if (!isHorizontalSegment() && !isVerticalSegment())
    return fixRect;

  x0 = fixRect.min_corner().get<0>();
  y0 = fixRect.min_corner().get<1>();
  x1 = fixRect.max_corner().get<0>();
  y1 = fixRect.max_corner().get<1>();

  if (isHorizontalSegment())
    return ORRect_t(ORPoint_t(x0, y0 - 1), ORPoint_t(x1, y1 + 1));
  return ORRect_t(ORPoint_t(x0 - 1, y0), ORPoint_t(x1 + 1, y1));
}

bool GLSegmentShape::isHorizontalSegment() const
{
  double x0 = boost::geometry::get<0, 0>(segment_);
  double y0 = boost::geometry::get<0, 1>(segment_);
  double x1 = boost::geometry::get<1, 0>(segment_);
  double y1 = boost::geometry::get<1, 1>(segment_);
  return y0 == y1;
}

bool GLSegmentShape::isVerticalSegment() const
{
  double x0 = boost::geometry::get<0, 0>(segment_);
  double y0 = boost::geometry::get<0, 1>(segment_);
  double x1 = boost::geometry::get<1, 0>(segment_);
  double y1 = boost::geometry::get<1, 1>(segment_);
  return x0 == x1;
}

uint GLSegmentShape::draw(GLView* view,
                          int shpLayer,
                          GLPen* pen,
                          ORPoint_t* drawAt,
                          bool drawAnimate,
                          bool drawOutlineOnly)
{
  if (pen)
    pen->setGLPenContext();
  if (drawAt) {
    glPushMatrix();
    glTranslatef(
        boost::geometry::get<0>(*drawAt), boost::geometry::get<1>(*drawAt), 0);
  }
  auto llx = boost::geometry::get<0, 0>(segment_);
  auto lly = boost::geometry::get<0, 1>(segment_);
  auto urx = boost::geometry::get<1, 0>(segment_);
  auto ury = boost::geometry::get<1, 1>(segment_);
  auto minDrawArea = view->getMinDrawableArea();

  uint numShapesDrawn = 0;
  if (boost::geometry::distance(segment_.first, segment_.second)
      <= minDrawArea) {
    glBegin(GL_POINTS);
    glVertex2f(llx, lly);
    glEnd();
    ++numShapesDrawn;
  } else {
    glLineWidth(lineWidth_);
    if (smooth_ || dashed_)
      glPushAttrib(GL_ALL_ATTRIB_BITS);
    if (smooth_ == true) {
      glDisable(GL_DEPTH_TEST);
      glDisable(GL_LIGHTING);
      glEnable(GL_POINT_SMOOTH);
      glEnable(GL_LINE_SMOOTH);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glDrawBuffer(GL_BACK);
    }

    if (dashed_ == true) {
      glEnable(GL_LINE_STIPPLE);
      // Choose any pattern for now
      glLineStipple(3, 0xAAAA);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glDrawBuffer(GL_BACK);
    }

    glBegin(GL_LINES);
    glVertex2f(llx, lly);
    glVertex2f(urx, ury);
    glEnd();

    if (smooth_ || dashed_)
      glPopAttrib();

    glLineWidth(1.0);  // Reset to default width
    numShapesDrawn++;
  }
  if (drawAt)
    glPopMatrix();
  return numShapesDrawn;
}

GLShape* GLSegmentShape::clone() const
{
  GLShape* shp = new GLSegmentShape(segment_, lineWidth_, smooth_, dashed_);
  shp->setUserData(getUserData());
  return shp;
}

GLCanvasInstShape::GLCanvasInstShape(GLCanvas* cnv,
                                     const GLRectangle& instBBox,
                                     odb::dbOrientType ort)
    : GLShape(), masterCanvas_(cnv), instBBox_(instBBox), cellTx_(nullptr)
{
  // Find out a way to calculate actual Bounding Box of the master canvas with
  // transformations applied
  cnvTxBBox_ = instBBox;
  cellTx_ = new GLCellTransform(instBBox, ort);
}

GLCanvasInstShape::GLCanvasInstShape(GLCanvas* cnv,
                                     const ORRect_t& instBBox,
                                     odb::dbOrientType ort)
    : GLShape(), masterCanvas_(cnv), instBBox_(instBBox), cellTx_(nullptr)
{
  // Find out a way to calculate actual Bounding Box of the master canvas with
  // transformations applied
  cnvTxBBox_ = instBBox;
  cellTx_ = new GLCellTransform(instBBox, ort);
}

GLCanvasInstShape::~GLCanvasInstShape()
{
}

uint GLCanvasInstShape::draw(GLView* view,
                             int shpLayer,
                             GLPen* pen,
                             ORPoint_t* drawAt,
                             bool drawAnimate,
                             bool drawOutlineOnly)
{
  uint numShapesDrawn = 0;
  bool txMatrixPushed = false;

  // if (view->getCurrentViewDepth() >= view->getMaxViewDepthToDraw()) {
  //    return 0 ;
  //}

  if (pen != nullptr)
    pen->setGLPenContext();

  if (drawAt != nullptr) {
    glPushMatrix();
    glTranslatef(
        boost::geometry::get<0>(*drawAt), boost::geometry::get<1>(*drawAt), 0);
    txMatrixPushed = true;
  }

  auto minC = cnvTxBBox_.min_corner();
  auto maxC = cnvTxBBox_.max_corner();
  auto llx = boost::geometry::get<0>(minC);
  auto lly = boost::geometry::get<1>(minC);
  auto urx = boost::geometry::get<0>(maxC);
  auto ury = boost::geometry::get<1>(maxC);
  auto minDrawArea = view->getMinDrawableArea();
  if (boost::geometry::area(cnvTxBBox_) <= minDrawArea && shpLayer == -1) {
    if (shpLayer != -1) {
      if (txMatrixPushed)
        glPopMatrix();
      return 1;
    }
    glBegin(GL_POINTS);
    glVertex2f(llx, lly);
    glEnd();
    numShapesDrawn += 1;
    if (txMatrixPushed)
      glPopMatrix();
    return numShapesDrawn;
  }

  if (view->getCurrentViewDepth() < view->getMaxViewDepthToDraw()
      && !drawOutlineOnly && shpLayer != -1) {
    // OpenRoadPen* cnvInstPen = OpenRoadPen::getCurrentActivePen() ;
    // ShapeLayer == -1, means we are drawing the canvasInst and not its guts
    // glPushAttrib(GL_ALL_ATTRIB_BITS) ;
    if (drawAt
        == nullptr) {  // Else We have already Pushed a Transformation matrix
      glPushMatrix();
      txMatrixPushed = true;
    }
    std::vector<uint> drawLayers;
    auto cnvBBox = masterCanvas_->getCanvasBBox(drawLayers, true);
    glPushMatrix();
    glLoadIdentity();

    view->viewTransform_->applyTransform(cnvTxBBox_);
    cellTx_->applyTransform(cnvBBox);

    view->updateCurrentViewDepth(true);
    numShapesDrawn += masterCanvas_->drawLayer(shpLayer, cnvBBox, view);
    view->updateCurrentViewDepth(false);
    glPopMatrix();
    if (txMatrixPushed)
      glPopMatrix();
    return numShapesDrawn;
  }
  if (drawOutlineOnly == false)
    glRectf(llx, lly, urx, ury);

  // Now draw the rectangle for the boundary.
  glBegin(GL_LINE_LOOP);
  glVertex2f(llx, lly);
  glVertex2f(urx, lly);
  glVertex2f(urx, ury);
  glVertex2f(llx, ury);
  glEnd();
  numShapesDrawn += 1;
  if (txMatrixPushed)
    glPopMatrix();
  return numShapesDrawn;
}

GLShape* GLCanvasInstShape::clone() const
{
  GLShape* shp = new GLCanvasInstShape(
      masterCanvas_, instBBox_, cellTx_->getOrientation());
  shp->setUserData(getUserData());
  return shp;
}

odb::dbOrientType GLCanvasInstShape::getCellOrientation() const
{
  return cellTx_->getOrientation();
}

GLMarkerShape::GLMarkerShape(const GLPoint2D& markerLoc,
                             ORMarkerType markerType)
    : GLShape(), markerBBox_(markerLoc, markerLoc), markerType_(markerType)
{
}

GLMarkerShape::GLMarkerShape(const GLRectangle& markerBBox,
                             ORMarkerType markerType)
    : GLShape(), markerBBox_(markerBBox), markerType_(markerType)
{
}

ORRect_t GLMarkerShape::getBBox() const
{
  ORRect_t bbox = markerBBox_;
  return bbox;
}

uint GLMarkerShape::draw(GLView* view,
                         int shpLayer,
                         GLPen* pen,
                         ORPoint_t* drawAt,
                         bool drawAnimate,
                         bool drawOutlineOnly)
{
  // glPushAttrib(GL_ALL_ATTRIB_BITS) ;

  if (pen)
    pen->setGLPenContext();
  if (drawAt != nullptr) {
    glPushMatrix();
    glTranslatef(
        boost::geometry::get<0>(*drawAt), boost::geometry::get<1>(*drawAt), 0);
  }
  if (markerBBox_.area() != 0)
    drawMarkerBBox(view, drawOutlineOnly);
  if (markerType_ == OR_DIAMOND_MARKER)
    drawDiamondMarker(view);
  else if (markerType_ == OR_PLUS_MARKER)
    drawPlusMarker(view);
  else if (markerType_ == OR_CIRCLE_MARKER)
    drawCircleMarker(view);

  if (drawAt != nullptr)
    glPopMatrix();
  return 1;
}

void GLMarkerShape::drawDiamondMarker(GLView* view)
{
  auto visArea = view->getVisibleArea();
  auto width = visArea.width();
  auto height = visArea.height();

  double markerDim = height / 30.0;
  if (width > height)
    markerDim = width / 30;

  GLfloat markX = static_cast<GLfloat>(markerBBox_.ll().x());
  GLfloat markY = static_cast<GLfloat>(markerBBox_.ll().y());
  GLfloat markZ = 0.0;

  auto factor = markerDim / 6;
  // Draw First Tilted Square
  glBegin(GL_LINE_LOOP);
  glVertex3f(markX, markY + factor, markZ);
  glVertex3f(markX + factor, markY, markZ);
  glVertex3f(markX, markY - factor, markZ);
  glVertex3f(markX - factor, markY, markZ);
  glEnd();

  factor = markerDim / 3;
  // Draw Second Tilted Square
  glBegin(GL_LINE_LOOP);
  glVertex3f(markX, markY + factor, markZ);
  glVertex3f(markX + factor, markY, markZ);
  glVertex3f(markX, markY - factor, markZ);
  glVertex3f(markX - factor, markY, markZ);
  glEnd();

  factor = markerDim / 2;
  // Draw Third Tilted Square
  glBegin(GL_LINE_LOOP);
  glVertex3f(markX, markY + factor, markZ);
  glVertex3f(markX + factor, markY, markZ);
  glVertex3f(markX, markY - factor, markZ);
  glVertex3f(markX - factor, markY, markZ);
  glEnd();

  // glPopAttrib() ;
}

void GLMarkerShape::drawPlusMarker(GLView* view)
{
  // TBD
}

void GLMarkerShape::drawCircleMarker(GLView* view)
{
  auto visArea = view->getFitVisibleArea();
  auto width = visArea.width();
  auto height = visArea.height();

  auto winWidth = view->getViewPortWidth();
  auto winHeight = view->getViewPortHeight();

  double markerDim = winWidth * 0.01;
  if (winHeight > winWidth)
    markerDim = winHeight * 0.01;

  // double markerDim = height/95.0 ;
  // if (width > height)
  //    markerDim = width/95 ;

  GLfloat markX = static_cast<GLfloat>(markerBBox_.ll().x());
  GLfloat markY = static_cast<GLfloat>(markerBBox_.ll().y());

  glPushAttrib(GL_ALL_ATTRIB_BITS);

  glEnable(GL_POINT_SMOOTH);
  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

  glPointSize(markerDim);
  glBegin(GL_POINTS);
  glVertex2f(markX, markY);
  glEnd();

  glPopAttrib();
}

void GLMarkerShape::drawMarkerBBox(GLView* view, bool drawOutlineOnly)
{
  auto llx = markerBBox_.ll().x();
  auto lly = markerBBox_.ll().y();
  auto urx = markerBBox_.ur().x();
  auto ury = markerBBox_.ur().y();
  auto minDrawArea = view->getMinDrawableArea();

  if (markerBBox_.area() <= minDrawArea) {
    glBegin(GL_POINTS);
    glVertex2f(llx, lly);
    glEnd();
  } else {
    if (!drawOutlineOnly)
      glRectf(llx, lly, urx, ury);

    // Now draw the rectangle for the boundary.
    glBegin(GL_LINE_LOOP);
    glVertex2f(llx, lly);
    glVertex2f(urx, lly);
    glVertex2f(urx, ury);
    glVertex2f(llx, ury);
    glEnd();
    std::stringstream oss;
    oss << "Drawn Rect Shape " << this;
    DEBUG_PRINT(oss.str());
  }
}

GLShape* GLMarkerShape::clone() const
{
  GLShape* shp = new GLMarkerShape(*this);
  return shp;
}
}  // namespace OpenRoadUI
