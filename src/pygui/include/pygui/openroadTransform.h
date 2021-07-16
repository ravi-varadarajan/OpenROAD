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

#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <vector>

#ifndef SWIG
#include <GL/gl.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#endif

#include "pygui/openroadGeom.h"
#include "pygui/openroadUiEnums.h"
#include "opendb/dbTypes.h"

using namespace OpenRoadUI;

namespace OpenRoadUI {
using namespace std;

#ifndef SWIG
class TxScale;
class TxRotate;
class TxTranslate;

namespace bg = boost::geometry;
#endif
class TxMatrix;
class TxOp;
class GLTransform;
class GLCellTransform;

class TxMatrix
{
 public:
  TxMatrix();
  TxMatrix(const double mat[4][4]);
  ~TxMatrix() {}

  TxMatrix& operator=(const double mat[4][4]);

  // From GL Calls, hence not used ORGeom Objects
  void translate(double transX,
                 double transY,
                 double transZ,
                 bool postApply = false);
  void scale(double scaleX,
             double scaleY,
             double scaleZ,
             bool postApply = false);
  void rotate(double ang,
              double rotX,
              double rotY,
              double rotZ,
              bool postApply = false);
#ifndef SWIG
  ORPoint_t transformPoint(const ORPoint_t& coord) const;
  ORPoint3D_t transformPoint(const ORPoint3D_t& coord) const;
  ORRect_t transformRect(const ORRect_t& rect) const;
#endif
  GLPoint2D transformPointPy(const GLPoint2D& pt2d) const;
  GLRectangle transformRectPy(const GLRectangle& rect) const;

  double get(int row, int col) const { return matrix_[row][col]; }

  void printTransformMatrix() const;

  void resetToGLTransMat();
  void resetTransMat(const double mat[16]);
  TxMatrix multMatrix(const TxMatrix& mat, bool pre = false) const;

  friend class TxOp;

#ifndef SWIG
  friend class TxTranslate;
  friend class TxScale;
  friend class TxRotate;
#endif

 private:
  void preMult(const double transMat[4][4]);   // transMat * matrix_
  void postMult(const double transMat[4][4]);  // matrix_ * transMat

  double matrix_[4][4];
};

// AtrOp : abstract transformation operation
class TxOp
{
 public:
  TxOp(bool inPlace = true) : inPlaceOp_(inPlace) {}
  virtual ~TxOp() {}

  void transformMatrix(TxMatrix& trMat, bool postApply = false) const;
  void getMatrix(double mat[4][4]) const;

  void printOp() const;

  virtual std::string getTxOpTypeName() const
  {
    return std::string("AbstractTx");
  }
  virtual TxOpType getTxOpType() const { return UNKNOWN_TX; }

  virtual void apply() const;
  virtual TxOp* clone() const;

#ifndef SWIG
  virtual void printOperation(std::ostream& os) const;
#endif

  virtual bool isInplaceOp() const { return inPlaceOp_; }
  virtual bool canBeMerged(const TxOp* op) const
  {
    return (op && op->getTxOpType() == this->getTxOpType());
  }
  virtual TxOp* merge(const TxOp* op) const
  {
    (void) op;
    return nullptr;
  }

#ifndef SWIG
  friend ostream& operator<<(std::ostream& os, const TxOp& op)
  {
    op.printOperation(os);
    return os;
  }
#endif
 protected:
  bool inPlaceOp_;
};

#ifndef SWIG
class TxScale : public TxOp
{
 public:
  TxScale(ORPoint3D_t scaleVec, bool inPlace = true)
      : TxOp(inPlace), scaleVec_(scaleVec)
  {
  }
  ~TxScale() {}

  string getTxOpTypeName() const { return std::string("Scale"); }
  TxOpType getTxOpType() const { return SCALE_TX; }

  void apply() const;
  TxOp* clone() const;
  ORPoint3D_t getScaleVec() { return scaleVec_; }

  TxOp* merge(const TxOp* op) const;

  void printOperation(std::ostream& os) const
  {
    os << "Scale Vector : " << scaleVec_ << std::endl;
  }

 private:
  ORPoint3D_t scaleVec_;
};

class TxRotate : public TxOp
{
 public:
  TxRotate(double angle, const ORPoint3D_t& vec, bool inPlace = true)
      : TxOp(inPlace), rotateVec_(vec), angle_(angle)
  {
  }
  ~TxRotate() {}

  string getTxOpTypeName() const
  {
    std::stringstream ss;
    ss << "Rotate By Angle " << angle_ << " At Vector " << rotateVec_;
    return ss.str();
  }
  TxOpType getTxOpType() const { return ROTATION_TX; }

  void apply() const;
  TxOp* clone() const;

  double rotationAngle() const { return angle_; }
  ORPoint3D_t rotationVector() const { return rotateVec_; }

  void printOperation(std::ostream& os) const
  {
    os << "Rotate By Angle " << angle_ << " At Vector " << rotateVec_
       << std::endl;
  }

  bool canBeMerged(const TxOp* op) const
  {
    if (op && op->getTxOpType() == this->getTxOpType()) {
      const TxRotate* rotOp = dynamic_cast<const TxRotate*>(op);
      if (rotOp == nullptr)
        return false;
      auto vec1 = rotOp->rotationVector();
      auto vec2 = rotOp->rotationVector();
      return (bg::get<0>(vec1) == bg::get<0>(vec2)
              && bg::get<1>(vec1) == bg::get<1>(vec2)
              && bg::get<2>(vec1) == bg::get<2>(vec2));
    }
    return false;
  }

  TxOp* merge(const TxOp* op) const;

 private:
  double angle_;
  ORPoint3D_t rotateVec_;
};

class TxTranslate : public TxOp
{
 public:
  TxTranslate(const ORPoint3D_t& transVec)
      : TxOp(false), translateVec_(transVec)
  {
  }
  ~TxTranslate() {}

  string getTxOpTypeName() const { return std::string("Translation"); }
  TxOpType transOpType() const { return TRANSLATION_TX; }

  void apply() const;
  TxOp* clone() const;

  ORPoint3D_t transVector() const { return translateVec_; }

  void printOperation(std::ostream& os) const
  {
    os << "Translation : " << translateVec_ << std::endl;
  }

  bool isInplaceOp() const { return false; }
  TxOp* merge(const TxOp* op) const;

  // Why is the following function needed
  void setTransform(ORPoint3D_t newTransVec) { translateVec_ = newTransVec; }

 private:
  ORPoint3D_t translateVec_;
};
#endif

// AtrTransform: responsible for containing all transform operations and
// performing them
class GLTransform
{
 public:
  GLTransform();
  virtual ~GLTransform();

  virtual ORTransformType getTransformType() const { return OR_VIEW_TRANSFORM; }

  // Assumption is that one is applying the following operation on top of
  // current Transform ...(Post Operations..)

  void rotate(double ang = 0.0,
              double rotX = 0.0,
              double rotY = 0.0,
              double rotZ = 1.0,
              bool inPlace = true);
  void scale(double scaleX = 1.0,
             double scaleY = 1.0,
             double scaleZ = 1.0,
             bool inPlace = true);
  void translate(double tx = 0.0, double ty = 0.0, double tz = 0.0);

#ifndef SWIG
  virtual void applyTransform(const ORRect_t& rect);
#endif
  virtual void applyTransformPy(GLRectangle rect);

  void resetTransform();
  void loadIdentity() const;

  TxMatrix getTransformationMatrix(GLRectangle rect) const;

  size_t operationsCount() const;
  size_t inPlaceOperationsCount() const;

  void printTransformStack() const;

 protected:
#ifndef SWIG
  virtual void populateTransformMatrix(const OpenRoadUI::ORRect_t& rect);
#endif

  std::list<TxOp*> operations_;  // stack of operations
  std::list<TxOp*> inPlaceOperations_;

  TxMatrix currentTXMatrix_;
  GLRectangle currentTxMatrixRect_;  // Rect On which TxMatrix was calculated

  bool repopulateMatrix_;
};

// DEF Transforms are composite transform of a kind, hence special class
class GLCellTransform : public GLTransform
{
 public:
  GLCellTransform(const GLRectangle& rect,
                  odb::dbOrientType ort = odb::dbOrientType::R0);
  GLCellTransform();
  ~GLCellTransform() {}

  ORTransformType getTransformType() const { return OR_CELL_TRANSFORM; }
  odb::dbOrientType getOrientation() const { return cellOrient_; }
  void setTransformParams(GLRectangle rect, odb::dbOrientType ort);

 private:
  void pushTransformOperations();
  GLRectangle cellBBox_;
  odb::dbOrientType cellOrient_;
};
}  // namespace OpenRoadUI
