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

#include "openroadGeom.h"

namespace OpenRoadUI {
// Forward class Declarations
class GLPen;
class GLShape;
class GLView;
class GLMotionShape
{
 public:
  GLMotionShape(const GLPoint2D& motionStartCoord, GLPen* motionPen = nullptr);
  virtual ~GLMotionShape();

  virtual void draw(GLView* view, const GLPoint2D& curWorldCoord);

  static GLPen* _defaultMotionPen;

 protected:
  GLPoint2D motionStart_;
  GLPen* motionPen_;
};

class GLRubberbandRect : public GLMotionShape
{
 public:
  GLRubberbandRect(const GLPoint2D& motionStartCoord,
                   GLPen* motionPen = nullptr);
  ~GLRubberbandRect();

  void draw(GLView* view, const GLPoint2D& curWorldCoord);

 private:
};

class GLRubberbandLine : public GLMotionShape
{
 public:
  GLRubberbandLine(const GLPoint2D& motionStartCoord,
                   GLPen* motionPen = nullptr);
  ~GLRubberbandLine();

  void draw(GLView* view, const GLPoint2D& curWorldCoord);

 private:
};

class GLAnimationShape : public GLMotionShape
{
 public:
  GLAnimationShape(const GLPoint2D& motionStartCoord,
                   GLShape* motionShape,
                   int shpLayer = -1,
                   GLPen* motionPen = nullptr);
  ~GLAnimationShape();

  void draw(GLView* view, const GLPoint2D& curWorldCoord);

 private:
  GLShape* motionShape_;
  int shpLayer_;
};
}  // namespace OpenRoadUI
