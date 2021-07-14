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

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "../include/pygui/openroadUiEnums.h"

#ifndef SWIG
#include <GL/gl.h>
#endif

namespace OpenRoadUI {
// Forward Class Declarations
class GLPen;
class GLLayer;

class ColorParams
{
 public:
  ColorParams(float r = 0, float g = 0, float b = 0, float a = 1.0)
      : red_(r), green_(g), blue_(b), alpha_(a)
  {
  }
  ~ColorParams() {}

  float red() const;
  float green() const;
  float blue() const;
  float alpha() const;

 private:
  float red_;
  float green_;
  float blue_;
  float alpha_;
};

typedef std::unordered_map<std::string, ColorParams> NamedColorMap_t;
typedef std::unordered_map<std::string, GLPen> NamedPenMap_t;
typedef std::unordered_map<std::string, GLLayer*> LayerColl_t;

class GLPen
{
 public:
  GLPen();
  GLPen(const ColorParams& colorVal, ORPenPatternType pat = OR_FILL_NONE_PAT);
  GLPen(const std::string& colorName, ORPenPatternType pat = OR_FILL_NONE_PAT);
  ~GLPen();

  ColorParams getPenColorVals() const;
  void setPenColorVals(ColorParams params);

  ORPenPatternType getPenPattern() const;
  void setPenPattern(ORPenPatternType pat);

  bool isNamedColor() const;
  bool setColor(const std::string& colorName);
  bool isPenValid() const;

  bool setGLPenContext();

  static bool getParamsForColor(const std::string& colorName,
                                ColorParams& params);
  static GLPen getPen(
      std::string penColor);  // For Invalid color it will return a black pen
  static GLPen* getCurrentActivePen() { return _currentActivePen; }
#ifndef SWIG
  static GLubyte* getPenBytePattern(unsigned int patType);
#endif

 private:
  ColorParams penColorParams_;
  ORPenPatternType penPattern_;

  std::string colorName_;
  bool penValid_;

  static GLPen* _currentActivePen;
  static NamedColorMap_t _namedColors;
  static NamedPenMap_t _namedPens;  // Colored Pen with Fill_None Pattern
};

class GLLayer
{
 public:
  GLLayer(const std::string& layerName, int layerIdx, ORLayerType layerType);
  ~GLLayer();

  void addChildLayer(GLLayer* layer);

  std::string getLayerName() const;
  ORLayerType getLayerType() const;
  int getLayerIdx() const;
  int getChildCount() const;
  GLLayer* getChildLayerAt(uint idx) const;
  void getChildLayerIdsRecurse(std::vector<uint>& childLayerIds,
                               bool checkSelectable = true,
                               bool checkVisibilty = true) const;

  void setLayerVisible(bool val);
  void setLayerSelectable(bool val);
  bool isLayerVisible() const { return visible_; }
  bool isLayerSelectable() const { return selectable_; }

  void setLayerPen(GLPen* pen);
  GLPen* getLayerPen() const { return layerPen_; }
  void dumpLayers(GLLayer* layer = nullptr, int level = 0);

  static GLLayer* getLayerAt(uint idx);
  static uint allLayerCount();
  static uint leafLayersCount();
  static void setRootLayer(GLLayer* root);
  static GLLayer* getRootLayer();

  static GLLayer* createDummyLayerTree();  // Test Function to be removed later

 private:
  std::string layerName_;
  // Unique Index Maintained By Drawing Subsystem, even the non leaf layer will
  // have a unique index.
  int layerIdx_;
  ORLayerType layerType_;
  bool selectable_;
  bool visible_;

  // Following members can be NULL, please check before using
  void* layerUserData_;
  GLPen* layerPen_;

  GLLayer* parent_;
  std::vector<GLLayer*> childLayers_;

  static GLLayer* _rootLayer;
  static std::unordered_map<int, GLLayer*> _allLayers;
};
}  // namespace OpenRoadUI
