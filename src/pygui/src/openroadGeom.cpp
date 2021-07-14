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

#include "pygui/openroadGeom.h"

#include <sstream>

#include "pygui/openroadShape.h"

std::ostream& operator<<(std::ostream& os, const OpenRoadUI::ORPoint_t& pt)
{
  os << "[" << bg::get<0>(pt) << ", " << bg::get<1>(pt) << "]";
  return os;
}

std::ostream& operator<<(std::ostream& os, const OpenRoadUI::GLPoint2D& pt)
{
  OpenRoadUI::ORPoint_t pt2d(pt.x_, pt.y_);
  os << pt2d;
  return os;
}

std::ostream& operator<<(std::ostream& os, const OpenRoadUI::ORPoint3D_t& pt)
{
  os << "[" << bg::get<0>(pt) << ", " << bg::get<1>(pt) << ", "
     << bg::get<2>(pt) << "]";
  return os;
}

std::ostream& operator<<(std::ostream& os, const OpenRoadUI::ORRect_t& rect)
{
  auto minC = rect.min_corner();
  auto maxC = rect.max_corner();

  os << "[" << minC << ", " << maxC << "]";

  return os;
}

std::ostream& operator<<(std::ostream& os, const OpenRoadUI::GLRectangle& rect)
{
  OpenRoadUI::ORRect_t rect1(OpenRoadUI::ORPoint_t(rect.ll_),
                             OpenRoadUI::ORPoint_t(rect.ur_));
  os << rect1;
  return os;
}
namespace OpenRoadUI {
GLPoint2D::GLPoint2D(double x, double y) : x_(x), y_(y)
{
}

GLPoint2D::GLPoint2D(const GLPoint2D& pt) : x_(pt.x_), y_(pt.y_)
{
}

GLPoint2D::GLPoint2D(const ORPoint_t& pt)
{
  x_ = pt.get<0>();
  y_ = pt.get<1>();
}

double GLPoint2D::x() const
{
  return x_;
}

double GLPoint2D::y() const
{
  return y_;
}

double GLPoint2D::distance(const GLPoint2D& pt) const
{
  ORPoint_t pt1 = *this;
  ORPoint_t pt2 = pt;
  return bg::distance(pt1, pt2);
}

void GLPoint2D::print() const
{
  ORPoint_t point2d(x_, y_);
  std::cout << point2d;
}

std::string GLPoint2D::repr() const
{
  std::stringstream ss;
  ORPoint_t pt(x_, y_);
  ss << pt;
  return ss.str();
}

GLSegment::GLSegment(const GLPoint2D& pt1, const GLPoint2D& pt2)
    : end1_(pt1), end2_(pt2)
{
}

GLSegment::GLSegment(const GLSegment& seg) : end1_(seg.end1_), end2_(seg.end2_)
{
}

GLSegment::GLSegment(const ORSegment_t& seg)
{
  end1_.x_ = bg::get<0, 0>(seg);
  end1_.y_ = bg::get<0, 1>(seg);

  end2_.x_ = bg::get<1, 0>(seg);
  end2_.y_ = bg::get<1, 1>(seg);
}

GLPoint2D GLSegment::end1() const
{
  return end1_;
}

GLPoint2D GLSegment::end2() const
{
  return end2_;
}

void GLSegment::print() const
{
  ORPoint_t segEnd1(end1_.x_, end1_.y_);
  ORPoint_t segEnd2(end2_.x_, end2_.y_);
  std::cout << segEnd1 << ", " << segEnd2;
}

std::string GLSegment::repr() const
{
  std::stringstream ss;
  ORPoint_t segEnd1(end1_.x_, end1_.y_);
  ORPoint_t segEnd2(end2_.x_, end2_.y_);
  ss << segEnd1 << ", " << segEnd2;
  return ss.str();
}

GLRectangle::GLRectangle(double llx, double lly, double urx, double ury)
    : ll_(GLPoint2D(llx, lly)), ur_(GLPoint2D(urx, ury))
{
}

GLRectangle::GLRectangle(const GLPoint2D& ll, const GLPoint2D& ur)
    : ll_(ll), ur_(ur)
{
}

GLRectangle::GLRectangle(const GLRectangle& rect) : ll_(rect.ll_), ur_(rect.ur_)
{
}

GLRectangle::GLRectangle(const ORRect_t& rect)
{
  auto minC = rect.min_corner();
  auto maxC = rect.max_corner();

  ll_ = minC;
  ur_ = maxC;
}

GLRectangle& GLRectangle::operator=(const GLRectangle& rect)
{
  ll_ = rect.ll_;
  ur_ = rect.ur_;
  return *this;
}

GLRectangle GLRectangle::getRectWithFixedOrientation() const
{
  auto x1 = ll_.x();
  auto y1 = ll_.y();
  auto x2 = ur_.x();
  auto y2 = ur_.y();

  GLRectangle retRect(
      std::min(x1, x2), std::min(y1, y2), std::max(x1, x2), std::max(y1, y2));
  return retRect;
}

GLPoint2D GLRectangle::ll() const
{
  return ll_;
}

GLPoint2D GLRectangle::ur() const
{
  return ur_;
}

void GLRectangle::print() const
{
  ORPoint_t ll(ll_.x_, ll_.y_);
  ORPoint_t ur(ll_.x_, ll_.y_);
  ORRect_t rect(ll, ur);
  std::cout << rect;
}

std::string GLRectangle::repr() const
{
  std::stringstream ss;
  ORPoint_t ll(ll_.x_, ll_.y_);
  ORPoint_t ur(ur_.x_, ur_.y_);
  ORRect_t rect(ll, ur);
  ss << rect;
  return ss.str();
}

GLRectangle GLRectangle::bloat(float bloatFactor)
{
  ORRect_t curRect = *this;
  auto outRect = bloatRect(curRect, bloatFactor);
  GLRectangle retRect(outRect);
  return retRect;
}

void SearchTree::addShapeInTree(uint layIdx, GLShape* p_shp)
{
  ORRect_t shpBBox = p_shp->getBoundingBox();
  if (shapeColls_.find(layIdx) == shapeColls_.end())
    shapeColls_[layIdx] = ORQuadTree<GLShape*>();
  shapeColls_[layIdx].insert(std::make_pair(shpBBox, p_shp));
}

void SearchTree::addShapesInTree(uint layIdx,
                                 const std::vector<GLShape*>& shapes)
{
  if (shapeColls_.find(layIdx) == shapeColls_.end())
    shapeColls_[layIdx] = ORQuadTree<GLShape*>();
  for (auto& p_shp : shapes) {
    ORRect_t shpBBox = p_shp->getBoundingBox();
    shapeColls_[layIdx].insert(std::make_pair(shpBBox, p_shp));
  }
}

void SearchTree::clearTree()
{
  shapeColls_.clear();
}

void SearchTree::clearAndDestroyTree()
{
  for (auto& shpData : shapeColls_)
    std::for_each(shpData.second.begin(),
                  shpData.second.end(),
                  [&](auto& treeData) { delete treeData.second; });
  shapeColls_.clear();
}

void SearchTree::clearTreeLayers(const std::vector<uint>& layers)
{
  for (auto& layIdx : layers)
    shapeColls_.erase(layIdx);
}

void SearchTree::clearAndDestroyTreeLayers(const std::vector<uint>& layers)
{
  for (auto& layIdx : layers) {
    if (shapeColls_.find(layIdx) != shapeColls_.end())
      std::for_each(shapeColls_[layIdx].begin(),
                    shapeColls_[layIdx].end(),
                    [&](auto& shpData) { delete shpData.second; });
    shapeColls_.erase(layIdx);
  }
}

ORPoint3D_t appendDimension(const ORPoint_t& pt)
{
  ORPoint3D_t pt3d(bg::get<0>(pt), bg::get<1>(pt), 0);
  return pt3d;
}

ORRect_t bloatRect(const ORRect_t& inpRect, float bloatFactor)
{
  auto ll = inpRect.min_corner();
  auto ur = inpRect.max_corner();

  auto rWidth = rectWidth(inpRect);
  auto rLength = rectHeight(inpRect);
  auto rectArea = bg::area(inpRect);

  auto aspectRatio = rLength / rWidth;

  auto newArea = rectArea * bloatFactor;
  auto newLength = sqrt(newArea * aspectRatio);
  auto newWidth = newLength / aspectRatio;

  auto lengthIncr = newLength - rLength;
  auto widthIncr = newWidth - rWidth;

  auto llx = ll.get<0>();
  auto lly = ll.get<1>();
  auto urx = ur.get<0>();
  auto ury = ur.get<1>();

  llx = llx - (widthIncr / 2.0);
  urx = urx + (widthIncr / 2.0);
  lly = lly - (lengthIncr / 2.0);
  ury = ury + (lengthIncr / 2.0);

  return ORRect_t(ORPoint_t(llx, lly), ORPoint_t(urx, ury));
}

double rectWidth(const ORRect_t& inpRect)
{
  auto minC = inpRect.min_corner();
  auto maxC = inpRect.max_corner();
  bg::subtract_point(maxC, minC);
  auto width = std::abs(maxC.get<0>());
  return width;
}

double rectHeight(const ORRect_t& inpRect)
{
  auto minC = inpRect.min_corner();
  auto maxC = inpRect.max_corner();
  bg::subtract_point(maxC, minC);
  auto height = std::abs(maxC.get<1>());
  return height;
}
}  // namespace OpenRoadUI
