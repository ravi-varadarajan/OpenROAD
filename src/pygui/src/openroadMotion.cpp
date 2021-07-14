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

#include "pygui/openroadMotion.h"

#include <boost/geometry.hpp>
#include <cassert>

#include "pygui/openroadGeom.h"
#include "pygui/openroadLayer.h"
#include "pygui/openroadShape.h"

namespace OpenRoadUI {
// static Member Initailaization
GLPen* GLMotionShape::_defaultMotionPen
    = new OpenRoadUI::GLPen("red", OR_FILL_SOLID_PAT);

GLMotionShape::GLMotionShape(const GLPoint2D& motionStartCoord,
                             GLPen* motionPen)
    : motionStart_(motionStartCoord), motionPen_(motionPen)
{
}

GLMotionShape::~GLMotionShape()
{
  // TBD
}

void GLMotionShape::draw(GLView* view, const GLPoint2D& curWorldCoord)
{
  (void) view;
  (void) curWorldCoord;
  assert(false && "Abstract draw function for Base OpenRoadMotionShape Class to create Python Swig Bindings for OpenRoadMotionShape") ;
  return;
}

GLRubberbandRect::GLRubberbandRect(const GLPoint2D& motionStartCoord,
                                   GLPen* motionPen)
    : GLMotionShape(motionStartCoord, motionPen)
{
}

GLRubberbandRect::~GLRubberbandRect()
{
}

void GLRubberbandRect::draw(GLView* view, const GLPoint2D& curCoord)
{
  (void) view;
  GLMotionShape::_defaultMotionPen->setGLPenContext();
  float startX = static_cast<float>(motionStart_.x());
  float startY = static_cast<float>(motionStart_.y());
  float curX = static_cast<float>(curCoord.x());
  float curY = static_cast<float>(curCoord.y());
  // std::cout << "Drawing Rubberband Rect ("<<startX<<", "<<startY<<"),
  // ("<<curX<<", "<<curY<<")\n" ;
  glBegin(GL_LINE_LOOP);
  glVertex2f(startX, startY);
  glVertex2f(curX, startY);
  glVertex2f(curX, curY);
  glVertex2f(startX, curY);
  glEnd();
}

GLRubberbandLine::GLRubberbandLine(const GLPoint2D& motionStartCoord,
                                   GLPen* motionPen)
    : GLMotionShape(motionStartCoord, motionPen)
{
}

GLRubberbandLine::~GLRubberbandLine()
{
}

void GLRubberbandLine::draw(GLView* view, const GLPoint2D& curCoord)
{
  (void) view;
  GLMotionShape::_defaultMotionPen->setGLPenContext();
  float startX = static_cast<float>(motionStart_.x());
  float startY = static_cast<float>(motionStart_.y());
  float curX = static_cast<float>(curCoord.x());
  float curY = static_cast<float>(curCoord.y());
  glBegin(GL_LINES);
  glVertex2f(startX, startY);
  glVertex2f(curX, curY);
  glEnd();
}

GLAnimationShape::GLAnimationShape(const GLPoint2D& motionStartCoord,
                                   GLShape* motionShape,
                                   int shpLayer,
                                   GLPen* motionPen)
    : GLMotionShape(motionStartCoord, motionPen),
      motionShape_(motionShape),
      shpLayer_(shpLayer)
{
}

GLAnimationShape::~GLAnimationShape()
{
}

void GLAnimationShape::draw(GLView* view, const GLPoint2D& curWorldCoord)
{
  ORPoint_t motionStart = motionStart_;
  ORPoint_t motionPt = curWorldCoord;

  boost::geometry::subtract_point(motionPt, motionStart);
  motionShape_->draw(
      view, shpLayer_, GLMotionShape::_defaultMotionPen, &motionPt, true, true);
}
}  // namespace OpenRoadUI
