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

#include "pygui/openroadTransform.h"

#include <GL/gl.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <cassert>
#include <sstream>

#include "pygui/openroadGeom.h"
#include "pygui/openroadGlobals.h"

namespace bg = boost::geometry;

namespace OpenRoadUI {
void TxOp::transformMatrix(TxMatrix& trMat, bool postApply) const
{
  double mat[4][4];
  this->getMatrix(mat);
  if (postApply)
    trMat.postMult(mat);
  else
    trMat.preMult(mat);
}

void TxOp::getMatrix(double mat[4][4]) const
{
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  this->apply();
  double dArray[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, dArray);
  glPopMatrix();

  int j = 0, k = 0;
  for (int i = 0; i < 16; i++) {
    mat[k][j] = dArray[i];
    j = (j + 1) % 4;
    k = j ? k : (k + 1);
  }
}

void TxOp::printOp() const
{
  this->printOperation(std::cout);
}

// virtual
void TxOp::apply() const
{
  assert(false && "Abstract TxOp class can not be applied");
}

// virtual
TxOp* TxOp::clone() const
{
  assert(false && "Abstract TxOp class can not be cloned");
  return nullptr;
}

void TxScale::apply() const
{
  std::stringstream oss;
  oss << "Applying Scale : " << scaleVec_;
  DEBUG_PRINT(oss.str());
  glScalef(bg::get<0>(scaleVec_), bg::get<1>(scaleVec_), bg::get<2>(scaleVec_));
}

TxOp* TxScale::clone() const
{
  return new TxScale(scaleVec_, inPlaceOp_);
}

void TxOp::printOperation(std::ostream& os) const
{
  assert(false && "Abstract Base Class TxOp cant be printed");
  return;
}

TxOp* TxScale::merge(const TxOp* op) const
{
  if (op == nullptr)
    return this->clone();

  const TxScale* scl = dynamic_cast<const TxScale*>(op);

  if (scl == nullptr)
    return nullptr;

  auto scaleX = bg::get<0>(scaleVec_);
  auto scaleY = bg::get<1>(scaleVec_);
  auto scaleZ = bg::get<2>(scaleVec_);

  double sclX = scaleX * bg::get<0>(scl->scaleVec_);
  double sclY = scaleY * bg::get<0>(scl->scaleVec_);
  double sclZ = scaleZ * bg::get<1>(scl->scaleVec_);

  if (sclX == 1.0 && sclY == 1.0 && sclZ == 1.0)
    return nullptr;

  TxOp* ret = new TxScale(ORPoint3D_t(sclX, sclY, sclZ), inPlaceOp_);
  return ret;
}

void TxRotate::apply() const
{
  glRotatef(angle_,
            bg::get<0>(rotateVec_),
            bg::get<1>(rotateVec_),
            bg::get<2>(rotateVec_));
}

TxOp* TxRotate::clone() const
{
  return new TxRotate(angle_, rotateVec_, inPlaceOp_);
}

TxOp* TxRotate::merge(const TxOp* op) const
{
  if (op == nullptr)
    return this->clone();

  const TxRotate* rot = dynamic_cast<const TxRotate*>(op);
  if (rot == nullptr || bg::equals(rotateVec_, rot->rotateVec_) == false)
    return nullptr;

  double rotAngle = rotationAngle() + rot->rotationAngle();
  int rotAngleVal = (int) rotAngle;
  int rotMod360 = rotAngleVal % 360;
  double remainingAngle = rotAngle - (double) rotAngleVal;

  if (rotMod360 == 0 && remainingAngle == 0)
    return nullptr;
  double curAngle = (double) rotMod360 + remainingAngle;
  TxOp* resOp = new TxRotate(curAngle, rotateVec_, inPlaceOp_);

  return resOp;
}

void TxTranslate::apply() const
{
  std::stringstream oss;
  oss << " Applying Translate : " << translateVec_;
  DEBUG_PRINT(oss.str());
  glTranslatef(bg::get<0>(translateVec_),
               bg::get<1>(translateVec_),
               bg::get<2>(translateVec_));
}

TxOp* TxTranslate::clone() const
{
  return new TxTranslate(translateVec_);
}

TxOp* TxTranslate::merge(const TxOp* op) const
{
  if (op == nullptr)
    return this->clone();

  const TxTranslate* trans = dynamic_cast<const TxTranslate*>(op);

  if (trans == nullptr)
    return nullptr;

  double transX = bg::get<0>(translateVec_) + bg::get<0>(trans->translateVec_);
  double transY = bg::get<1>(translateVec_) + bg::get<1>(trans->translateVec_);
  double transZ = bg::get<2>(translateVec_) + bg::get<2>(trans->translateVec_);

  if (transX == 0.0 && transY == 0.0 && transZ == 0.0)
    return nullptr;

  ORPoint3D_t newTranslationVec(transX, transY, transZ);
  TxOp* resOp = new TxTranslate(newTranslationVec);
  return resOp;
}

TxMatrix::TxMatrix()
{
  // Store Identity Matrix
  matrix_[0][0] = 1;
  matrix_[0][1] = 0;
  matrix_[0][2] = 0;
  matrix_[0][3] = 0;

  matrix_[1][0] = 0;
  matrix_[1][1] = 1;
  matrix_[1][2] = 0;
  matrix_[1][3] = 0;

  matrix_[2][0] = 0;
  matrix_[2][1] = 0;
  matrix_[2][2] = 1;
  matrix_[2][3] = 0;

  matrix_[3][0] = 0;
  matrix_[3][1] = 0;
  matrix_[3][2] = 0;
  matrix_[3][3] = 1;
}

TxMatrix::TxMatrix(const double mat[4][4])
{
  for (unsigned int i = 0; i < 4; ++i) {
    for (unsigned int j = 0; j < 4; ++j)
      matrix_[i][j] = mat[i][j];
  }
}

TxMatrix& TxMatrix::operator=(const double mat[4][4])
{
  for (unsigned int i = 0; i < 4; ++i) {
    for (unsigned int j = 0; j < 4; ++j) {
      matrix_[i][j] = mat[i][j];
    }
  }
  return *this;
}

void TxMatrix::translate(double transX,
                         double transY,
                         double transZ,
                         bool postApply)
{
  if (transX == 0 && transY == 0 && transZ == 0)
    return;

  double transMat[4][4];

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glTranslatef(transX, transY, transZ);
  double dArray[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, dArray);
  glPopMatrix();

  int j = 0, k = 0;
  for (int i = 0; i < 16; i++) {
    transMat[k][j] = dArray[i];
    j = (j + 1) % 4;
    k = j ? k : (k + 1);
  }

  if (postApply)
    postMult(transMat);
  else
    preMult(transMat);
}

void TxMatrix::scale(double scaleX,
                     double scaleY,
                     double scaleZ,
                     bool postApply)
{
  if (scaleX == 1 && scaleY == 1 && scaleZ == 1)
    return;

  double transMat[4][4];

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glScalef(scaleX, scaleY, scaleZ);
  double dArray[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, dArray);
  glPopMatrix();

  int j = 0, k = 0;
  for (int i = 0; i < 16; i++) {
    transMat[k][j] = dArray[i];
    j = (j + 1) % 4;
    k = j ? k : (k + 1);
  }

  if (postApply)
    postMult(transMat);
  else
    preMult(transMat);
}

void TxMatrix::rotate(double ang,
                      double rotX,
                      double rotY,
                      double rotZ,
                      bool postApply)
{
  if (ang == 0)
    return;

  double transMat[4][4];

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glRotatef(ang, rotX, rotY, rotZ);
  double dArray[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, dArray);
  glPopMatrix();

  int j = 0, k = 0;
  for (int i = 0; i < 16; i++) {
    transMat[k][j] = dArray[i];
    j = (j + 1) % 4;
    k = j ? k : (k + 1);
  }

  if (postApply)
    postMult(transMat);
  else
    preMult(transMat);
}

void TxMatrix::postMult(const double transMat[4][4])
{
  double resMat[4][4];
  for (int i = 0; i < 4; ++i) {
    for (int k = 0; k < 4; ++k) {
      resMat[i][k] = 0;
      for (int j = 0; j < 4; ++j) {
        resMat[i][k] += matrix_[i][j] * transMat[j][k];
      }
    }
  }
  for (int i = 0; i < 4; ++i)
    for (int k = 0; k < 4; ++k)
      matrix_[i][k] = resMat[i][k];
}

void TxMatrix::preMult(const double transMat[4][4])
{
  double resMat[4][4];
  for (int i = 0; i < 4; ++i) {
    for (int k = 0; k < 4; ++k) {
      resMat[i][k] = 0;
      for (int j = 0; j < 4; ++j) {
        resMat[i][k] += transMat[i][j] * matrix_[j][k];
      }
    }
  }
  for (int i = 0; i < 4; ++i)
    for (int k = 0; k < 4; ++k)
      matrix_[i][k] = resMat[i][k];
}

ORPoint_t TxMatrix::transformPoint(const ORPoint_t& coord) const
{
  ORPoint3D_t pt3d = appendDimension(coord);
  auto txPt = transformPoint(pt3d);
  return ORPoint_t(bg::get<0>(txPt), bg::get<1>(txPt));
}

ORPoint3D_t TxMatrix::transformPoint(const ORPoint3D_t& coord) const
{
  double xVal = bg::get<0>(coord) * matrix_[0][0]
                + bg::get<1>(coord) * matrix_[1][0]
                + bg::get<2>(coord) * matrix_[2][0] + matrix_[3][0];
  double yVal = bg::get<0>(coord) * matrix_[0][1]
                + bg::get<1>(coord) * matrix_[1][1]
                + bg::get<2>(coord) * matrix_[2][1] + matrix_[3][1];
  double zVal = bg::get<0>(coord) * matrix_[0][2]
                + bg::get<1>(coord) * matrix_[1][2]
                + bg::get<2>(coord) * matrix_[2][2] + matrix_[3][2];

  return ORPoint3D_t(xVal, yVal, zVal);
}

ORRect_t TxMatrix::transformRect(const ORRect_t& rect) const
{
  auto ll = rect.min_corner();
  auto ur = rect.max_corner();

  auto newLL = transformPoint(ll);
  auto newUR = transformPoint(ur);

  return ORRect_t(newLL, newUR);
}

GLPoint2D TxMatrix::transformPointPy(const GLPoint2D& pt2d) const
{
  ORPoint_t pt = pt2d;
  pt = transformPoint(pt);
  GLPoint2D txPt(pt);
  return txPt;
}

GLRectangle TxMatrix::transformRectPy(const GLRectangle& rect) const
{
  ORRect_t inpRect = rect;
  ORRect_t outRect = transformRect(inpRect);
  GLRectangle txRect(outRect);
  return txRect;
}

void TxMatrix::printTransformMatrix() const
{
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j)
      std::cout << matrix_[i][j] << " ";
    std::cout << std::endl;
  }
}

void TxMatrix::resetToGLTransMat()
{
  double dArray[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, dArray);
  this->resetTransMat(dArray);
}

void TxMatrix::resetTransMat(const double dArray[16])
{
  double mat[4][4];

  int j = 0, k = 0;
  for (int i = 0; i < 16; i++) {
    mat[k][j] = dArray[i];
    j = (j + 1) % 4;
    k = j ? k : (k + 1);
  }
  this->operator=(mat);
}

TxMatrix TxMatrix::multMatrix(const TxMatrix& mat, bool pre) const
{
  TxMatrix resMat = this->matrix_;
  if (pre)
    resMat.preMult(mat.matrix_);
  else
    resMat.postMult(mat.matrix_);
  return resMat;
}

GLTransform::GLTransform()
    : operations_(), inPlaceOperations_(), repopulateMatrix_(true)
{
}

GLTransform::~GLTransform()
{
  // TBD : Destroy Transforms Here...
}

void GLTransform::rotate(double ang,
                         double rotX,
                         double rotY,
                         double rotZ,
                         bool inPlace)
{
  ORPoint3D_t rotVec(rotX, rotY, rotZ);
  TxRotate* rotOp = new TxRotate(ang, rotVec, inPlace);
  auto pushOperation = [](std::list<TxOp*>& ops, TxRotate* rotateTx) {
    if (ops.empty() != true
        && ops.front()->getTxOpType() == rotateTx->getTxOpType()) {
      TxOp* frontOp = ops.front();
      TxOp* mergedOp = frontOp->merge(rotateTx);

      TxOp* prev_topOp = ops.front();

      ops.pop_front();
      ops.push_front(mergedOp);

      delete rotateTx;
      delete prev_topOp;
    } else
      ops.push_front(rotateTx);
  };
  if (!inPlace)
    pushOperation(operations_, rotOp);
  else
    pushOperation(inPlaceOperations_, rotOp);

  repopulateMatrix_ = true;
}

void GLTransform::scale(double scaleX,
                        double scaleY,
                        double scaleZ,
                        bool inPlace)
{
  std::stringstream oss;
  oss << "Scaling Transform : " << scaleX << ", " << scaleY << ", " << scaleZ
      << ", inplce = " << inPlace;
  DEBUG_PRINT(oss.str());
  oss.clear();
  ORPoint3D_t scaleVec(scaleX, scaleY, scaleZ);
  TxScale* scaleOp = new TxScale(scaleVec, inPlace);

  auto pushOperation = [](std::list<TxOp*>& ops, TxScale* scaleTx) {
    if (false && ops.empty() != true
        && ops.front()->getTxOpType() == scaleTx->getTxOpType()) {
      TxOp* frontOp = ops.front();
      TxOp* mergedOp = frontOp->merge(scaleTx);

      TxOp* prev_topOp = ops.front();

      ops.pop_front();
      ops.push_front(mergedOp);

      delete scaleTx;
      delete prev_topOp;
    } else
      ops.push_front(scaleTx);
  };
  if (inPlace)
    pushOperation(inPlaceOperations_, scaleOp);
  else
    pushOperation(operations_, scaleOp);
  repopulateMatrix_ = true;
}

void GLTransform::translate(double tx, double ty, double tz)
{
  std::stringstream oss;
  oss << "Translating " << tx << ", " << ty << ", " << tz;
  DEBUG_PRINT(oss.str());
  oss.clear();
  ORPoint3D_t translateVec(tx, ty, tz);
  TxTranslate* transOp = new TxTranslate(translateVec);
  if (false && operations_.empty() != true
      && operations_.front()->getTxOpType() == transOp->getTxOpType()) {
    TxTranslate transOp(translateVec);

    TxOp* frontOp = operations_.front();
    TxOp* mergedOp = frontOp->merge(&transOp);
    TxOp* prev_topOp = operations_.front();
    operations_.pop_front();
    operations_.push_front(mergedOp);
    delete prev_topOp;
  } else
    operations_.push_front(new TxTranslate(translateVec));
  repopulateMatrix_ = true;
}

void GLTransform::applyTransform(const ORRect_t& rect)
{
  std::stringstream oss;
  if (operations_.empty() && inPlaceOperations_.empty()) {
    oss << "No Transformations found, none applied";
    DEBUG_PRINT(oss.str());
    oss.clear();
    return;
  }

  /*
          std::list<TxOp*> oldInPlaceTransStack = inPlaceOperations_ ;
          std::list<TxOp*> newTransforms        = inPlaceOperations_ ;

          TxTranslate* reverseTrans = nullptr ;
          TxTranslate* forwardTrans = nullptr ;
          if (!inPlaceOperations_.empty()) {
              ORPoint_t center ;
              bg::centroid(rect, center) ;
              ORPoint3D_t centerVec = appendDimension(center) ;
              ORPoint3D_t invCenterVec = centerVec ;
              bg::multiply_value(centerVec, -1) ;

              reverseTrans = new TxTranslate(invCenterVec) ;
              forwardTrans = new TxTranslate(centerVec) ;

              newTransforms.push_back(reverseTrans) ;
              newTransforms.push_front(forwardTrans) ;
          }

          std::for_each(newTransforms.begin(), newTransforms.end(), [](TxOp*
     curOp){ curOp->apply();}) ;
  */
  std::for_each(operations_.begin(), operations_.end(), [](TxOp* curOp) {
    curOp->apply();
  });
  /*
          if (reverseTrans) {
              delete reverseTrans ;
              delete forwardTrans ;
          }
  */
  return;
}

void GLTransform::applyTransformPy(GLRectangle rect)
{
  ORRect_t txRect = rect;
  applyTransform(txRect);
}

void GLTransform::resetTransform()
{
  operations_.clear();
  inPlaceOperations_.clear();
  repopulateMatrix_ = true;
  currentTxMatrixRect_
      = ORRect_t(ORPoint_t(0, 0), ORPoint_t(0, 0));  // 0 Size Rect at 0,0
}

void GLTransform::loadIdentity() const
{
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

TxMatrix GLTransform::getTransformationMatrix(GLRectangle rect) const
{
  GLTransform& curTrans = const_cast<GLTransform&>(*this);
  curTrans.populateTransformMatrix(rect);
  return curTrans.currentTXMatrix_;
}

size_t GLTransform::operationsCount() const
{
  return operations_.size();
}

size_t GLTransform::inPlaceOperationsCount() const
{
  return inPlaceOperations_.size();
}

void GLTransform::printTransformStack() const
{
  std::cout << "#####################################\n";
  std::cout << "Transformations of Transform " << this << std::endl;
  std::cout << "In Place Operations in Transforms : \n";
  for (auto& op : inPlaceOperations_) {
    std::cout << "  ";
    op->printOperation(std::cout);
  }
  std::cout << "Operations in Transforms : \n";
  for (auto& op : operations_) {
    std::cout << "  ";
    op->printOperation(std::cout);
  }
}

void GLTransform::populateTransformMatrix(const ORRect_t& visRect)
{
  if (repopulateMatrix_) {
    repopulateMatrix_ = false;
    double dArray[16];

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    applyTransform(visRect);

    glGetDoublev(GL_MODELVIEW_MATRIX, dArray);
    glPopMatrix();
    currentTXMatrix_.resetTransMat(dArray);
  }
  currentTxMatrixRect_ = visRect;
}

GLCellTransform::GLCellTransform(const GLRectangle& rect, TxCellOrientType ort)
    : GLTransform(), cellBBox_(rect), cellOrient_(ort)
{
  pushTransformOperations();
}

GLCellTransform::GLCellTransform()
{
}

void GLCellTransform::setTransformParams(GLRectangle rect, TxCellOrientType ort)
{
  cellBBox_ = rect;
  cellOrient_ = ort;
  pushTransformOperations();
}

void GLCellTransform::pushTransformOperations()
{
  repopulateMatrix_ = true;

  ORRect_t cellBBox = cellBBox_;

  double llx = bg::get<0>(cellBBox.min_corner());
  double lly = bg::get<1>(cellBBox.min_corner());

  double urx = bg::get<0>(cellBBox.max_corner());
  double ury = bg::get<1>(cellBBox.max_corner());

  double width = abs(urx - llx);
  double height = abs(ury - lly);

  switch (cellOrient_) {
    case R0: {
      ORPoint3D_t transVec(llx, lly, 0);
      TxTranslate* transOp1 = new TxTranslate(transVec);
      operations_.push_front(transOp1);
      break;
    }
    case MY: {
      ORPoint3D_t transVec1(llx, lly, 0);
      TxTranslate* transOp1 = new TxTranslate(transVec1);
      ORPoint3D_t scaleVec(-1, 1, 1);
      TxScale* scaleOp = new TxScale(scaleVec, false);

      ORPoint3D_t transVec2(width * -1, 0, 0);
      TxTranslate* transOp2 = new TxTranslate(transVec2);

      operations_.push_front(transOp2);
      operations_.push_front(scaleOp);
      operations_.push_front(transOp1);
      break;
    }
    case R180: {
      ORPoint3D_t transVec1(llx, lly, 0);
      TxTranslate* transOp1 = new TxTranslate(transVec1);

      ORPoint3D_t rotVec(0, 0, 1);
      TxRotate* rotateOp = new TxRotate(180, rotVec, false);

      ORPoint3D_t transVec2(width * -1, height * -1, 0);
      TxTranslate* transOp2 = new TxTranslate(transVec2);

      operations_.push_front(transOp2);
      operations_.push_front(rotateOp);
      operations_.push_front(transOp1);
      break;
    }
    case MX: {
      ORPoint3D_t transVec1(llx, lly, 0);
      TxTranslate* transOp1 = new TxTranslate(transVec1);
      ORPoint3D_t scaleVec(1, -1, 1);
      TxScale* scaleOp = new TxScale(scaleVec, false);

      ORPoint3D_t transVec2(0, -1 * height, 0);
      TxTranslate* transOp2 = new TxTranslate(transVec2);

      operations_.push_front(transOp2);
      operations_.push_front(scaleOp);
      operations_.push_front(transOp1);
      break;
    }
    case R270: {
      ORPoint3D_t rotVec(0, 0, 1);
      TxRotate* rotateOp = new TxRotate(270, rotVec, false);

      ORPoint3D_t transVec2(llx, lly + height, 0);
      TxTranslate* transOp = new TxTranslate(transVec2);

      operations_.push_front(rotateOp);
      operations_.push_front(transOp);
      break;
    }
    case MYR90: {
      ORPoint3D_t rotVec(0, 0, 1);
      TxRotate* rotateOp = new TxRotate(90, rotVec, false);

      ORPoint3D_t scaleVec(-1, 1, 1);
      TxScale* scaleOp = new TxScale(scaleVec, false);

      ORPoint3D_t transVec(width + llx, height + lly, 0);
      TxTranslate* transOp = new TxTranslate(transVec);

      operations_.push_front(scaleOp);
      operations_.push_front(rotateOp);
      operations_.push_front(transOp);
      break;
    }
    case R90: {
      ORPoint3D_t rotVec(0, 0, 1);
      TxRotate* rotateOp = new TxRotate(90, rotVec, false);

      ORPoint3D_t transVec2(width + llx, lly, 0);
      TxTranslate* transOp = new TxTranslate(transVec2);

      operations_.push_front(rotateOp);
      operations_.push_front(transOp);
      break;
    }
    case MXR90: {
      ORPoint3D_t rotVec(0, 0, 1);
      TxRotate* rotateOp = new TxRotate(90, rotVec, false);

      ORPoint3D_t scaleVec(-1, 1, 1);
      TxScale* scaleOp = new TxScale(scaleVec, false);

      ORPoint3D_t transVec(llx + width, lly + height, 0);
      TxTranslate* transOp = new TxTranslate(transVec);

      operations_.push_front(scaleOp);
      operations_.push_front(rotateOp);
      operations_.push_front(transOp);
      break;
    }
    default:
      break;
  }
}
}  // namespace OpenRoadUI
