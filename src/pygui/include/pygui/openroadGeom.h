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
#include <map>
#include <utility>
#include <vector>

#ifndef SWIG

#include <GL/gl.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point.hpp>

#endif

namespace OpenRoadUI {
class GLShape;
#ifndef SWIG
typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>
    ORPoint_t;
typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>
    ORPoint3D_t;

typedef boost::geometry::model::segment<ORPoint_t> ORSegment_t;
typedef boost::geometry::model::box<ORPoint_t> ORRect_t;

template <typename T>
using value_t = std::pair<ORRect_t, T>;

template <typename T>
using ORQuadTree
    = boost::geometry::index::rtree<value_t<T>,
                                    boost::geometry::index::quadratic<16>>;

template <typename T>
class ORMinSizePredicate
{
 public:
  ORMinSizePredicate(int min_size) : min_size_(min_size) {}
  bool operator()(const std::pair<ORRect_t, T>& o) const
  {
    const ORRect_t& box = o.first;
    const ORPoint_t& ll = box.min_corner();
    const ORPoint_t& ur = box.max_corner();
    int w = boost::geometry::get<0>(ur) - boost::geometry::get<0>(ll);
    int h = boost::geometry::get<1>(ur) - boost::geometry::get<1>(ll);
    return std::max(w, h) >= min_size_;
  }

 private:
  int min_size_;
};

template <typename T>
class ORMinHeightPredicate
{
 public:
  ORMinHeightPredicate(int min_height) : min_height_(min_height) {}
  bool operator()(const std::pair<ORRect_t, T>& o) const
  {
    const ORRect_t& box = o.first;
    const ORPoint_t& ll = box.min_corner();
    const ORPoint_t& ur = box.max_corner();
    int h = boost::geometry::get<1>(ur) - boost::geometry::get<1>(ll);
    return h >= min_height_;
  }

 private:
  int min_height_;
};

ORPoint3D_t appendDimension(const ORPoint_t& pt);
ORRect_t bloatRect(const ORRect_t& inpRect,
                   float bloatFactor);  // Bloats the rectangle by the factor
double rectWidth(const ORRect_t& inpRect);
double rectHeight(const ORRect_t& inpRect);

#endif
struct GLPixmap
{
  unsigned char* bits_;
  unsigned int width_;
  unsigned int height_;

  GLPixmap() : bits_(nullptr), width_(0), height_(0) {}
  ~GLPixmap() { delete[] bits_; }

  void clearPixmap()
  {
    if (!bits_)
      return;
    delete[] bits_;
    width_ = height_ = 0;
    bits_ = nullptr;
  }

  void initPixmap(int width, int height)
  {
    bits_ = new unsigned char[width * height * 3];
    width_ = width;
    height_ = height;
  }
};

class GLPoint2D
{
 public:
  GLPoint2D(double x = 0, double y = 0);
  GLPoint2D(const GLPoint2D& pt);
#ifndef SWIG
  GLPoint2D(const ORPoint_t& pt);
  operator ORPoint_t() const { return ORPoint_t(x_, y_); }
#endif
  bool operator==(const GLPoint2D& pt) const
  {
    return x_ == pt.x_ && y_ == pt.y_;
  }
  ~GLPoint2D() {}

  double x() const;
  double y() const;

  double distance(const GLPoint2D& pt) const;

  void print() const;
  std::string repr() const;

 public:
  double x_;
  double y_;
};

class GLSegment
{
 public:
  GLSegment(const GLPoint2D& end1, const GLPoint2D& end2);
  GLSegment(const GLSegment& seg);
  ~GLSegment() {}
#ifndef SWIG
  GLSegment(const ORSegment_t& seg);
  operator ORSegment_t() const { return ORSegment_t(end1_, end2_); }
#endif
  bool operator==(const GLSegment& seg) const
  {
    return end1_ == seg.end1_ && end2_ == seg.end2_;
  }
  GLPoint2D end1() const;
  GLPoint2D end2() const;

  void print() const;
  std::string repr() const;

 public:
  GLPoint2D end1_;
  GLPoint2D end2_;
};

class GLRectangle
{
 public:
  GLRectangle(double llx = 0, double lly = 0, double urx = 0, double ury = 0);
  GLRectangle(const GLPoint2D& ll, const GLPoint2D& ur);
  GLRectangle(const GLRectangle& pt);
#ifndef SWIG
  GLRectangle(const ORRect_t& pt);
  operator ORRect_t() const { return ORRect_t(ll_, ur_); }
#endif
  bool operator==(const GLRectangle& rect) const
  {
    return ll_ == rect.ll_ && ur_ == rect.ur_;
  }
  GLRectangle& operator=(const GLRectangle& rect);
  ~GLRectangle() {}

  GLRectangle getRectWithFixedOrientation()
      const;  // Rectangle in the form of ll to ur

  GLPoint2D ll() const;
  GLPoint2D ur() const;
  GLPoint2D center() const;

  double width() const { return (ur_.x() - ll_.x()); }
  double height() const { return (ur_.y() - ll_.y()); }

  void print() const;
  std::string repr() const;

  GLRectangle bloat(float bloatFactor);
  double area() { return (ur_.x() - ll_.x()) * (ur_.y() - ll_.y()); }
  GLRectangle translate(float transX, float transY) const;

 public:
  GLPoint2D ll_;
  GLPoint2D ur_;
};

struct SearchTree
{
#ifndef SWIG
  std::map<uint, ORQuadTree<GLShape*>> shapeColls_;
#endif
  void addShapeInTree(uint layIdx, GLShape*);
  void addShapesInTree(uint layIdx, const std::vector<GLShape*>& shapes);

  void clearTree();
  void clearAndDestroyTree();

  void clearTreeLayers(const std::vector<uint>& layers);
  void clearAndDestroyTreeLayers(const std::vector<uint>& layers);

  void dumpTree();
};
}  // namespace OpenRoadUI

#ifndef SWIG
std::ostream& operator<<(std::ostream& os, const OpenRoadUI::ORPoint_t& pt);
std::ostream& operator<<(std::ostream& os, const OpenRoadUI::GLPoint2D& pt);

std::ostream& operator<<(std::ostream& os, const OpenRoadUI::ORPoint3D_t& pt);

std::ostream& operator<<(std::ostream& os, const OpenRoadUI::ORRect_t& pt);
std::ostream& operator<<(std::ostream& os, const OpenRoadUI::GLRectangle& rect);
#endif
