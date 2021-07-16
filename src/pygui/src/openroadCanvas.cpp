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

#include "pygui/openroadCanvas.h"

#include <algorithm>
#include <boost/function_output_iterator.hpp>
#include <iostream>
#include <sstream>

#include "opendb/dbTypes.h"
#include "pygui/openroadGeom.h"
#include "pygui/openroadGlobals.h"
#include "pygui/openroadIntf.h"
#include "pygui/openroadLayer.h"
#include "pygui/openroadShape.h"
#include "pygui/openroadTransform.h"
#include "pygui/openroadView.h"
// namespace bg = boost::geometry;

namespace OpenRoadUI {
std::unordered_map<std::string, GLCanvas*> GLCanvas::_allCanvases
    = std::unordered_map<std::string, GLCanvas*>();
const char* leftEyeName = "Dummy's Left Eye";
const char* rightEyeName = "Dummy's Right Eye";
const char* noseName = "Dummy's Nose";
const char* lipName = "Dummy's Lips";
const char* tongueName = "Dummy's Tongue";
const char* faceName = "Dummy's Face";
const char* xAxisName = "X-Axis";
const char* yAxisName = "Y-Axis";

OpenRoadUI::GLCanvas::GLCanvas(OpenRoadUI::GLLayer* topLayerNode,
                               const char* cnvName)
    : canvasName_(cnvName),
      topLayerTreeNode_(topLayerNode),
      cnvUserData_(nullptr),
      canvasBBox_(nullptr),
      canvasBBoxUpToDate_(false),
      canvasDepth_(1),
      shapeColls_(nullptr),
      blockOp_(false),
      outlineLayer_(-1)
{
  _allCanvases[canvasName_] = this;
  canvasBBox_ = new GLRectangle();
  shapeColls_ = new SearchTree();
  // std::cout << "Came to Build Canvas : "<<this<<std::endl<<std::flush ;
}

GLCanvas::~GLCanvas()
{
  std::stringstream oss;
  oss << "Came to Destroy Canvas : " << this;
  DEBUG_PRINT(oss.str());
  _allCanvases.erase(canvasName_);
  delete canvasBBox_;
  delete shapeColls_;
}

bool GLCanvas::addShape(GLShape* shp, uint layerIdx)
{
  if (shp->getShapeType() == OR_CANVASINST_SHAPE) {
    GLCanvasInstShape* cnvInstShp = dynamic_cast<GLCanvasInstShape*>(shp);
    if (cnvInstShp) {
      if (masterCanvases_.find(cnvInstShp->getMasterCanvas())
          == masterCanvases_.end())
        masterCanvases_[cnvInstShp->getMasterCanvas()] = 0;
      masterCanvases_[cnvInstShp->getMasterCanvas()] += 1;
      if (canvasDepth_ == 1)
        canvasDepth_ += cnvInstShp->getMasterCanvas()->getCanvasDepth();
      else {
        if ((cnvInstShp->getMasterCanvas()->getCanvasDepth() + 1)
            > canvasDepth_)
          canvasDepth_ = cnvInstShp->getMasterCanvas()->getCanvasDepth() + 1;
      }
    }
    layersWithCanvasInsts_.insert(layerIdx);
  }
  auto shpBBox = shp->getBBox();
  shapeColls_->shapeColls_[layerIdx].insert(std::make_pair(shpBBox, shp));
  canvasBBoxUpToDate_ = false;
  if (blockOp_)
    return true;

  *canvasBBox_ = getFullBBox();
  std::stringstream oss;
  oss << "Adding Shape with bbox " << shpBBox
      << ", the updated canvas BBox = " << *canvasBBox_;
  DEBUG_PRINT(oss.str());
  canvasBBoxUpToDate_ = true;
  return true;
}

bool GLCanvas::removeShape(GLShape* shp, uint layerIdx, bool updateBBox)
{
  if (shp->getShapeType() == OR_CANVASINST_SHAPE) {
    GLCanvasInstShape* cnvInstShape = dynamic_cast<GLCanvasInstShape*>(shp);
    if (cnvInstShape) {
      if (masterCanvases_.find(cnvInstShape->getMasterCanvas())
          != masterCanvases_.end()) {
        if (masterCanvases_[cnvInstShape->getMasterCanvas()] != 0)
          masterCanvases_[cnvInstShape->getMasterCanvas()] -= 0;
      }
    }
  }
  if (!updateBBox)
    return true;
  // Unfortunately there is no direct way
  canvasBBoxUpToDate_ = false;
  std::vector<uint> layers;
  *canvasBBox_ = getCanvasBBox(layers, true);
  canvasBBoxUpToDate_ = true;

  return true;
}

uint GLCanvas::clearCanvasLayer(int layerIdx, bool destroyShapes)
{
  uint numShapesCleared = 0;
  for (auto& shapeColl : shapeColls_->shapeColls_) {
    if (layerIdx > 0 && shapeColl.first != layerIdx)
      continue;
    numShapesCleared += shapeColl.second.size();
    if (destroyShapes) {
      ORQuadTree<GLShape*>& shapeTree = shapeColl.second;
      for (auto& treeItem : shapeTree) {
        GLShape* shp = treeItem.second;
        delete shp;
      }
    }
    shapeColls_->shapeColls_[shapeColl.first] = ORQuadTree<GLShape*>();
  }
  std::vector<uint> layers;
  *canvasBBox_ = getCanvasBBox(layers, true);
  canvasBBoxUpToDate_ = true;

  return numShapesCleared;
}

void GLCanvas::setBlockOp(bool val)
{
  if (val) {
    blockOp_ = true;
  } else {
    blockOp_ = false;
    if (!canvasBBoxUpToDate_) {
      std::vector<uint> layers;
      *canvasBBox_ = getCanvasBBox(layers, true);
      canvasBBoxUpToDate_ = true;
    }
  }
}

uint GLCanvas::getShapeCountInLayer(int layerIdx) const
{
  uint shapeCount = 0;
  if (layerIdx != -1)
    return shapeColls_->shapeColls_[layerIdx].size();
  for (auto& shapeColl : shapeColls_->shapeColls_) {
    // if (layerIdx != -1 && layerIdx != shapeColl.first)
    //    continue ;
    shapeCount += shapeColl.second.size();
  }
  return shapeCount;
}

uint GLCanvas::getShapeCountFromAllLayers() const
{
  return getShapeCountInLayer(-1);
}

ORRect_t GLCanvas::getCanvasBBox(const std::vector<uint>& layers,
                                 bool useAllLayers) const
{
  ORQuadTree<int> boundTree;
  if (!useAllLayers) {
    for (auto layIdx : layers) {
      auto itr = shapeColls_->shapeColls_.find(layIdx);
      if (itr != shapeColls_->shapeColls_.end()) {
        auto& shapeColl = *itr;
        if (shapeColl.second.empty() == false)
          boundTree.insert(std::make_pair(shapeColl.second.bounds(), layIdx));
      }
    }
  } else {
    if (canvasBBoxUpToDate_)
      return *canvasBBox_;
    for (auto& shapeColl : shapeColls_->shapeColls_) {
      if (shapeColl.second.empty() == false)
        boundTree.insert(
            std::make_pair(shapeColl.second.bounds(), shapeColl.first));
    }
    GLCanvas* cnv = const_cast<GLCanvas*>(this);
    cnv->canvasBBoxUpToDate_ = true;
    *(cnv->canvasBBox_) = boundTree.bounds();
  }
  if (boundTree.empty())
    return ORRect_t();
  return boundTree.bounds();
}

GLRectangle GLCanvas::getCanvasBBoxPy(const std::vector<uint>& includeLayers,
                                      bool useAllLayers) const
{
  ORRect_t bbox = getCanvasBBox(includeLayers, useAllLayers);
  return GLRectangle(bbox);
}

ORRect_t GLCanvas::getFullBBox() const
{
  std::vector<uint> includeLayers;
  return getCanvasBBox(includeLayers, true);
}

GLRectangle GLCanvas::getFullBBoxPy() const
{
  std::vector<uint> includeLayers;
  ORRect_t cnvBBox = getCanvasBBox(includeLayers, true);
  GLRectangle bbox(cnvBBox);
  std::stringstream oss;
  oss << "Bounding Box returning to python : " << bbox << std::endl;
  DEBUG_PRINT(oss.str());
  return bbox;
}

GLShape* GLCanvas::findFirstMatchingShape(const std::vector<uint>& searchLayers,
                                          void* matchingUserData,
                                          ORRect_t* searchRegion) const
{
  for (auto layIdx : searchLayers) {
    auto itr = shapeColls_->shapeColls_.find(layIdx);
    if (itr == shapeColls_->shapeColls_.end())
      continue;
    auto& layerShapes = (*itr).second;
    if (searchRegion != nullptr) {
      std::vector<std::pair<ORRect_t, GLShape*>> shapesInRegion;
      layerShapes.query(boost::geometry::index::intersects(*searchRegion),
                        std::back_inserter(shapesInRegion));
      for (auto& treeItem : shapesInRegion) {
        GLShape* shp = treeItem.second;
        if (shp->getUserData() == matchingUserData)
          return shp;
      }
    } else {
      for (auto& treeItem : layerShapes) {
        GLShape* shp = treeItem.second;
        if (shp->getUserData() == matchingUserData)
          return shp;
      }
    }
  }
  return nullptr;
}

void GLCanvas::findShapesInRegion(const std::vector<uint>& searchLayers,
                                  std::vector<GLShape*>& shapesInRegion,
                                  ORRect_t* searchRegion) const
{
  for (auto layIdx : searchLayers) {
    auto itr = shapeColls_->shapeColls_.find(layIdx);
    if (itr == shapeColls_->shapeColls_.end())
      continue;
    auto& layerShapes = (*itr).second;
    if (searchRegion != nullptr) {
      std::vector<std::pair<ORRect_t, GLShape*>> shapesInTree;
      layerShapes.query(boost::geometry::index::intersects(*searchRegion),
                        std::back_inserter(shapesInTree));
      std::for_each(
          shapesInTree.begin(), shapesInTree.end(), [&](auto& layerShp) {
            shapesInRegion.push_back(layerShp.second);
          });
    } else {
      std::for_each(
          layerShapes.begin(), layerShapes.end(), [&](auto& layerShp) {
            shapesInRegion.push_back(layerShp.second);
          });
    }
  }
  return;
}

uint GLCanvas::popuateShapesFromRegion(const std::vector<uint>& searchLayers,
                                       SearchTree* shapes,
                                       ORRect_t* searchRegion) const
{
  uint numShapesInTheRegion = 0;
  for (auto layIdx : searchLayers) {
    auto itr = shapeColls_->shapeColls_.find(layIdx);
    if (itr == shapeColls_->shapeColls_.end())
      continue;
    if (shapes->shapeColls_.find(layIdx) == shapes->shapeColls_.end())
      shapes->shapeColls_[layIdx] = ORQuadTree<GLShape*>();
    auto& qTree = shapes->shapeColls_[layIdx];
    auto& layerShapes = (*itr).second;
    if (searchRegion != nullptr) {
      std::vector<std::pair<ORRect_t, GLShape*>> shapesInTree;
      layerShapes.query(boost::geometry::index::intersects(*searchRegion),
                        std::back_inserter(shapesInTree));
      numShapesInTheRegion += shapesInTree.size();
      std::for_each(shapesInTree.begin(),
                    shapesInTree.end(),
                    [&](auto& layerShpVal) { qTree.insert(layerShpVal); });
    } else {
      numShapesInTheRegion += layerShapes.size();
      std::for_each(layerShapes.begin(),
                    layerShapes.end(),
                    [&](auto& layerShpVal) { qTree.insert(layerShpVal); });
    }
  }
  return numShapesInTheRegion;
}

std::vector<GLShape*> GLCanvas::findShapesInRegionPy(
    const std::vector<uint>& searchLayers,
    GLRectangle* searchRegion) const
{
  std::vector<GLShape*> shapesInRegion;
  ORRect_t searchArea = *searchRegion;
  findShapesInRegion(searchLayers, shapesInRegion, &searchArea);
  return shapesInRegion;
}

uint GLCanvas::drawCanvas(GLView* view, const ORRect_t& visArea)
{
  auto minDrawArea = view->getMinDrawableArea();
  auto maxViewDepth = view->getMaxViewDepthToDraw();
  auto curViewDepth = view->getCurrentViewDepth();
  std::stringstream oss;
  oss << "Drawing Canvas, " << canvasName_
      << " whose visible Area :  " << visArea
      << " MinDrawableArea : " << minDrawArea
      << " Max View Depth : " << maxViewDepth
      << " Current View Depth : " << curViewDepth;
  DEBUG_PRINT(oss.str());
  uint numShapesDrawn = 0;

  std::vector<uint> childLayers;
  if (view->isWorldView() == false)
    topLayerTreeNode_->getChildLayerIdsRecurse(childLayers, false, true);
  else
    childLayers = view->getWorldViewDrawLayers();

  std::set<uint> layersToDraw;
  numShapesDrawn = 0;
  layersToDraw.insert(childLayers.begin(), childLayers.end());
  bool drawGuts = !view->isWorldView();
  auto intfInst = OpenRoadIntf::getOpenRoadIntfInst();
  for (auto& layIdx : layersToDraw) {
    GLLayer* layer = GLLayer::getLayerAt(layIdx);
    if (layer->isInternalLayer())
      continue;
    GLPen* layerPen = layer->getLayerPen();
    std::stringstream oss;
    oss << "Setting the pen for Layer : " << layIdx << " whose pattern is "
        << layerPen->getPenPattern();
    DEBUG_PRINT(oss.str());
    oss.clear();
    layerPen->setGLPenContext();
    if (shapeColls_->shapeColls_.find(layIdx)
        != shapeColls_->shapeColls_.end()) {
      numShapesDrawn += drawLayer(
          layIdx, visArea, view, false);  // Draw the leaf shapes in this layer
    }
    if (drawGuts
        && view->getCurrentViewDepth()
               < view->getMaxViewDepthToDraw()) {  // Draw Guts only when needed
      for (auto& cnvInstLayer : layersWithCanvasInsts_) {
        GLLayer* cnvLayer = GLLayer::getLayerAt(cnvInstLayer);
        auto& shapeQTree = shapeColls_->shapeColls_[cnvInstLayer];
        std::vector<std::pair<ORRect_t, GLShape*>> shapesInRegion;
        shapesInRegion.reserve(shapeQTree.size());
        shapeQTree.query(boost::geometry::index::intersects(visArea),
                         std::back_inserter(shapesInRegion));
        for (auto& shpItem : shapesInRegion) {
          GLShape* shp = shpItem.second;
          if (shp->getShapeType() == OR_CANVASINST_SHAPE) {
            GLCanvasInstShape* cnvInstShp
                = dynamic_cast<GLCanvasInstShape*>(shp);
            auto masterCnv = cnvInstShp->getMasterCanvas();
            if (masterCnv->getShapeCountInLayer(layIdx) > 0
                && layIdx != masterCnv->getOutlineLayer())
              numShapesDrawn += shp->draw(view, layIdx);
          }
        }
      }
    }
    intfInst->drawExternalRenderersForLayer(view, layer, this);
    oss.clear();
    oss << "Number of shapes Drawn in layer : " << layIdx
        << " with pattern = " << numShapesDrawn;
    DEBUG_PRINT(oss.str());
  }
  intfInst->drawExternalRenderersForObjects(view, this);
  return numShapesDrawn;
}

unsigned int GLCanvas::drawCanvasPy(GLView* view, GLRectangle rect)
{
  ORRect_t visRect = rect;
  return drawCanvas(view, visRect);
}

uint GLCanvas::drawLayer(int layerIdx,
                         const ORRect_t& visArea,
                         GLView* view,
                         bool propogateLayer)
{
  uint numShapesDrawn = 0;

  auto& shapeQTree = shapeColls_->shapeColls_[layerIdx];
  std::vector<std::pair<ORRect_t, GLShape*>> shapesInRegion;
  shapesInRegion.reserve(shapeQTree.size());
  shapeQTree.query(boost::geometry::index::intersects(visArea),
                   std::back_inserter(shapesInRegion));
  int layIdx = propogateLayer == true ? layerIdx : -1;
  for (auto& shpItem : shapesInRegion) {
    GLShape* shp = shpItem.second;
    numShapesDrawn += shp->draw(view, layIdx);
  }
  return numShapesDrawn;
}

// static
GLCanvas* GLCanvas::getCanvas(const char* canvasName)
{
  auto itr = _allCanvases.find(canvasName);
  if (itr == _allCanvases.end())
    return nullptr;
  GLCanvas* cnv = (*itr).second;
  return cnv;
}

// static
GLCanvas* GLCanvas::getDummyCanvas(bool onlyOutline)
{
  static std::string dummyCnvName = "DummyCanvas";
  GLCanvas* cnv = GLCanvas::getCanvas(dummyCnvName.c_str());
  if (cnv != nullptr)
    return cnv;
  std::stringstream oss;

  GLCanvas* dummyEyeCnv = GLCanvas::getDummyEyeCanvas();

  GLLayer* topLayer = GLLayer::getLayerAt(0);
  cnv = new GLCanvas(topLayer, dummyCnvName.c_str());

  if (onlyOutline) {
    ORRect_t faceRect(ORPoint_t(130, 440), ORPoint_t(590, 830));
    GLShape* faceShp = new GLRectShape(faceRect);
    faceShp->setUserData((void*) faceName);
    (void) cnv->addShape(faceShp, 7);
    return cnv;
  }

  // ORRect_t (ORPoint_t(32, 48), ORPoint_t(40, 51)) ;
  ORRect_t lowerLipRect(ORPoint_t(320, 480), ORPoint_t(400, 510));
  ORRect_t upperLipRect(ORPoint_t(280, 510), ORPoint_t(440, 540));
  ORRect_t noseRect(ORPoint_t(340, 580), ORPoint_t(380, 690));
  ORRect_t leftEyeRect(ORPoint_t(170, 710), ORPoint_t(235, 790));
  ORRect_t rightEyeRect(ORPoint_t(470, 710), ORPoint_t(535, 790));

  ORRect_t faceRect(ORPoint_t(130, 440), ORPoint_t(590, 830));

  ORSegment_t xAxisSeg(ORPoint_t(-120, 0), ORPoint_t(700, 0));
  ORSegment_t yAxisSeg(ORPoint_t(0, 850), ORPoint_t(0, -120));

  GLShape* leftEyeShp = nullptr;
  GLShape* rightEyeShp = nullptr;
  bool useCanvasInst = true;

  GLShape* lowerLipShp = new GLRectShape(lowerLipRect);
  GLShape* upperLipShp = new GLRectShape(upperLipRect);
  GLShape* noseShp = new GLRectShape(noseRect);
  if (!useCanvasInst || dummyEyeCnv == nullptr) {
    leftEyeShp = new GLRectShape(leftEyeRect);
    rightEyeShp = new GLRectShape(rightEyeRect);
  } else {
    leftEyeShp = new GLCanvasInstShape(
        dummyEyeCnv, leftEyeRect, odb::dbOrientType::R0);
    ORRect_t rightEyeRect90
        = ORRect_t(ORPoint_t(470, 710),
                   ORPoint_t(550, 775));  // For 90 * (2n-1) rotation family
    // rightEyeShp = new OpenRoadCanvasInstShape(dummyEyeCnv, rightEyeRect90,
    // MXR90) ;
    rightEyeShp = new GLCanvasInstShape(
        dummyEyeCnv, rightEyeRect90, odb::dbOrientType::MXR90);
  }

  GLShape* faceShp = new GLRectShape(faceRect);

  GLShape* xAxisShp = new GLSegmentShape(xAxisSeg);
  GLShape* yAxisShp = new GLSegmentShape(yAxisSeg);

  upperLipShp->setUserData((void*) lipName);
  lowerLipShp->setUserData((void*) tongueName);
  leftEyeShp->setUserData((void*) leftEyeName);
  rightEyeShp->setUserData((void*) rightEyeName);
  noseShp->setUserData((void*) noseName);
  faceShp->setUserData((void*) faceName);
  xAxisShp->setUserData((void*) xAxisName);
  yAxisShp->setUserData((void*) yAxisName);

  cnv->setBlockOp(true);
  (void) cnv->addShape(lowerLipShp, 1);
  (void) cnv->addShape(upperLipShp, 2);
  (void) cnv->addShape(noseShp, 3);
  (void) cnv->addShape(leftEyeShp, 4);
  (void) cnv->addShape(rightEyeShp, 4);
  (void) cnv->addShape(faceShp, 6);
  (void) cnv->addShape(xAxisShp, 5);
  (void) cnv->addShape(yAxisShp, 5);
  cnv->setBlockOp(false);

  oss << "Created a dummy canvas " << cnv << " with "
      << cnv->getShapeCountFromAllLayers()
      << " shapes, its BBox = " << cnv->getFullBBox();
  DEBUG_PRINT(oss.str());

  return cnv;
}

// Static
GLCanvas* GLCanvas::getDummyEyeCanvas()
{
  static std::string dummyEyeCnvName = "DummyEyeCanvas";
  GLCanvas* cnv = GLCanvas::getCanvas(dummyEyeCnvName.c_str());
  if (cnv != nullptr)
    return cnv;

  std::stringstream oss;
  GLLayer* topLayer = GLLayer::getLayerAt(0);
  cnv = new GLCanvas(topLayer, dummyEyeCnvName.c_str());

  ORRect_t outlineRect(ORPoint_t(0, 0), ORPoint_t(65, 80));
  ORRect_t shape1_Rect(ORPoint_t(20, 10), ORPoint_t(25, 60));
  ORRect_t shape2_Rect(ORPoint_t(30, 40), ORPoint_t(50, 50));
  ORRect_t shape3_Rect(ORPoint_t(40, 60), ORPoint_t(60, 75));

  GLShape* outlineShp = new GLRectShape(outlineRect);
  GLShape* shape1Shp = new GLRectShape(shape1_Rect);
  GLShape* shape2Shp = new GLRectShape(shape2_Rect);
  GLShape* shape3Shp = new GLRectShape(shape3_Rect);

  cnv->setBlockOp(true);
  cnv->addShape(outlineShp, 6);
  cnv->addShape(shape1Shp, 2);
  cnv->addShape(shape2Shp, 5);
  cnv->addShape(shape3Shp, 3);

  cnv->markOutlineLayer(6);

  oss << "Created Dummy's Eye canvas " << cnv << " with "
      << cnv->getShapeCountFromAllLayers()
      << " shapes, its BBox = " << cnv->getFullBBox();
  DEBUG_PRINT(oss.str());

  return cnv;
}
}  // end namespace OpenRoadUI
