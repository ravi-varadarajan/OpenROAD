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
#include <vector>

#include "../include/pygui/openroadGeom.h"
#include "../include/pygui/openroadUiEnums.h"

namespace OpenRoadUI {
// Forward class Declarations
#ifndef SWIG
class GLTransform;
class GLCellTransform;
#endif
class GLCanvas;
class GLLayer;
class GLPen;
class GLView;

// Base Shape
class GLShape;

// Leaf Shapes
class GLCompositeShape;
class GLRectShape;
class GLLineShape;
class GLPointShape;

// Hierarchical Shape
class GLCanvasInstShape;

// Qualified Shape
class GLCompositeShape;

// Marker Shape
class GLMarkerShape;

class GLShape
{
 public:
  GLShape();
  virtual ~GLShape() {}

#ifndef SWIG
  virtual ORRect_t getBBox() const = 0;
#endif
  virtual ORShapeType getShapeType() const = 0;
  // The return value is number of shapes drawn in the process, in case of
  // Heirarchical canvas it will be more than 1
#ifndef SWIG
  virtual uint draw(GLView* p_view,
                    int shpLayer,
                    GLPen* p_pen = nullptr,
                    ORPoint_t* drawAt = nullptr,
                    bool drawAnimate = false,
                    bool drawOutlineOnly = false)
      = 0;  // drawOutline will not do a fill pattern draw
#endif

  virtual GLShape* clone() const = 0;
  GLRectangle getBoundingBox() const;

  // Shape Transformation is not allowed, if one needs to do it, Remove the
  // shape from the canvas and then re-add a new one. Take care of select and
  // highlight shape vector on the view, in case this shape is added there
#ifndef SWIG
  void* getUserData() const { return userData_; }
  void setUserData(void* userData) { userData_ = userData; }
#endif
  // odbId is the databaseId of the OpenDb, in case the shape is non
  // openDbShape, the dbId shpuld be -1 Or Implementation should match the id
  // form OpenRoadLayoutContext
  std::string getShapeInfo(uint shpInLayer, int odbId = -1) const;

 private:
  void* userData_;
};

class GLCompositeShape : public GLShape
{
 public:
  GLCompositeShape(const std::vector<GLShape*>& shapeColl);
  ~GLCompositeShape() {}

#ifndef SWIG
  ORRect_t getBBox() const override { return compositeBBox_; }
#endif
  ORShapeType getShapeType() const override { return OR_COMPOSITE_SHAPE; }
#ifndef SWIG
  uint draw(GLView* view,
            int shpLayer,
            GLPen* pen = nullptr,
            ORPoint_t* drawAt = nullptr,
            bool drawAnimate = false,
            bool drawOutlineOnly = false);
#endif
  GLShape* clone() const override;

 private:
  std::vector<GLShape*> shapeCollection_;
#ifndef SWIG
  ORRect_t compositeBBox_;
#endif
};

// If the object is not valid OpenDb Objectwith bbox the constructor will assert
class GLDbShape : public GLShape
{
 public:
  GLDbShape(uint dbId);
  ~GLDbShape() {}

#ifndef SWIG
  ORRect_t getBBox() const override;
#endif
  ORShapeType getShapeType() const override { return OR_DB_SHAPE; }
#ifndef SWIG
  uint draw(GLView* view,
            int shpLayer,
            GLPen* pen = nullptr,
            ORPoint_t* drawAt = nullptr,
            bool drawAnimate = false,
            bool drawOutlineOnly = false) override;
#endif

  GLShape* clone() const override;
  uint getOpenDbId() const { return dbId_; }

 private:
  uint dbId_;
};

class GLRectShape : public GLShape
{
 public:
  GLRectShape(const GLRectangle& rect);
#ifndef SWIG
  GLRectShape(const ORRect_t& rect);
#endif
  ~GLRectShape() {}

#ifndef SWIG
  ORRect_t getBBox() const override { return rectShp_; }
#endif
  ORShapeType getShapeType() const override { return OR_RECT_SHAPE; }
#ifndef SWIG
  uint draw(GLView* view,
            int shpLayer,
            GLPen* pen = nullptr,
            ORPoint_t* drawAt = nullptr,
            bool drawAnimate = false,
            bool drawOutlineOnly = false) override;
#endif

  GLShape* clone() const override;

 private:
#ifndef SWIG
  ORRect_t rectShp_;
#endif
};

class GLSegmentShape : public GLShape
{
 public:
  GLSegmentShape(const GLSegment& seg,
                 float lineWidth = 1.0,
                 bool smooth = true,
                 bool dashed = false);
#ifndef SWIG
  GLSegmentShape(const ORSegment_t& seg,
                 float lineWidth = 1.0,
                 bool smooth = true,
                 bool dashed = false);
#endif
  ~GLSegmentShape() {}

#ifndef SWIG
  ORRect_t getBBox()
      const override;  //{ return ORRect_t(segment_.first, segment_.second); }
#endif
  ORShapeType getShapeType() const override { return OR_SEGMENT_SHAPE; }
#ifndef SWIG
  bool isHorizontalSegment() const;
  bool isVerticalSegment() const;

  uint draw(GLView* view,
            int shpLayer,
            GLPen* pen = nullptr,
            ORPoint_t* drawAt = nullptr,
            bool drawAnimate = false,
            bool drawOutlineOnly = false) override;
#endif

  bool isSmooth() const { return smooth_; }
  bool isDashed() const { return dashed_; }

  GLShape* clone() const override;

 private:
#ifndef SWIG
  ORSegment_t segment_;
#endif
  float lineWidth_;
  bool smooth_;
  bool dashed_;
};

class GLCanvasInstShape : public GLShape
{
 public:
  GLCanvasInstShape(GLCanvas* masterCnv,
                    const GLRectangle& instBBox,
                    TxCellOrientType ort = R0);
#ifndef SWIG
  GLCanvasInstShape(GLCanvas* masterCnv,
                    const ORRect_t& instBBox,
                    TxCellOrientType ort
                    = R0);  // BBox is transformed BBox stored in the DB
#endif
  ~GLCanvasInstShape();

#ifndef SWIG
  ORRect_t getBBox() const override { return cnvTxBBox_; }
#endif
  ORShapeType getShapeType() const override { return OR_CANVASINST_SHAPE; }
#ifndef SWIG
  uint draw(GLView* view,
            int shpLayer,
            GLPen* pen = nullptr,
            ORPoint_t* drawAt = nullptr,
            bool drawAnimate = false,
            bool drawOutlineOnly = false) override;
#endif

  GLShape* clone() const override;

#ifndef SWIG
  GLCanvas* getMasterCanvas() const { return masterCanvas_; }
  ORRect_t getInstBBox() const { return instBBox_; }
#endif
  TxCellOrientType getCellOrientation() const;

 private:
  GLCanvas* masterCanvas_;
#ifndef SWIG
  ORRect_t instBBox_;
  GLCellTransform* cellTx_;
  ORRect_t cnvTxBBox_;
#endif
};

class GLMarkerShape : public GLShape
{
 public:
  GLMarkerShape(const GLPoint2D& markerLoc, ORMarkerType markerType);
  GLMarkerShape(const GLRectangle& marker, ORMarkerType getMarkerType);
  ~GLMarkerShape() {}

#ifndef SWIG
  ORRect_t getBBox() const override;
#endif
  ORShapeType getShapeType() const override { return OR_MARKER_SHAPE; }
#ifndef SWIG
  uint draw(GLView* view,
            int shpLayer,
            GLPen* pen = nullptr,
            ORPoint_t* drawAt = nullptr,
            bool drawAnimate = false,
            bool drawOutlineOnly = false) override;
#endif

  GLShape* clone() const override;
  ORMarkerType getMarkerType() const { return markerType_; }
  GLPoint2D getMarkerLoc() const { return markerBBox_.ll(); }

 private:
  GLRectangle markerBBox_;
  ORMarkerType markerType_;

  void drawMarkerBBox(GLView* view, bool drawOutlineOnly);
  void drawDiamondMarker(GLView* view);
  void drawPlusMarker(GLView* view);
  void drawCircleMarker(GLView* view);
};
}  // namespace OpenRoadUI
