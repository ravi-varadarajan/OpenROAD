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

#include "pygui/openroadLayoutContext.h"

#include <iostream>

#include "opendb/db.h"
#include "pygui/openroadLayer.h"
#include "pygui/openroadShape.h"

namespace OpenRoadUI {
OpenRoadLayoutContext::ContextColl_t OpenRoadLayoutContext::_allContexts
    = OpenRoadLayoutContext::ContextColl_t();

void OpenRoadLayoutContext::populateLayers() const
{
  GLLayer* root
      = new GLLayer(std::string("ROOT"), 10000, ORLayerType::DESIGN_LAYER);
  GLLayer::setRootLayer(root);

  populateTechLayers();
  populateDesignLayers();

  // root->dumpLayers(nullptr, 0);
}

void OpenRoadLayoutContext::populateTechLayers() const
{
  GLLayer* root = GLLayer::getRootLayer();
  if (!root)
    return;

  GLLayer* top
      = new GLLayer(std::string("Technology"), 1000, ORLayerType::METAL_LAYER);
  root->addChildLayer(top);

  odb::dbTech* tech = p_ORDb_->getTech();

  const int numRoutePatterns = 2;
  const int numViaPatterns = 2;
  const int numPenColors = 18;

  const ORPenPatternType penRoutePatterns[]
      = {OpenRoadUI::OR_LEFT_DIAG_SPARSE_PAT,
         OpenRoadUI::OR_RIGHT_DIAG_SPARSE_PAT};
  const ORPenPatternType penViaPatterns[]
      = {OpenRoadUI::OR_HORIZONTAL_PAT, OpenRoadUI::OR_VERTICAL_PAT};
  const std::string penColors[] = {std::string("blue"),
                                   std::string("blue"),
                                   std::string("red"),
                                   std::string("red"),
                                   std::string("green"),
                                   std::string("green"),
                                   std::string("cyan"),
                                   std::string("cyan"),
                                   std::string("lilac"),
                                   std::string("lilac"),
                                   std::string("baby blue"),
                                   std::string("baby blue"),
                                   std::string("orange"),
                                   std::string("orange"),
                                   std::string("yellow"),
                                   std::string("yellow"),
                                   std::string("purple"),
                                   std::string("purple")};

  int routingLayerCount = 0;
  int viaLayerCount = 0;
  for (auto layer : tech->getLayers()) {
    uint id = layer->getId();
    std::string name = layer->getName();

    // std::cout << "Adding layer " << name << " id = " << id << std::endl;

    odb::dbTechLayerType layerType = layer->getType();
    ORPenPatternType pattern = OpenRoadUI::OR_CROSS_PAT;
    int colorIdx = (routingLayerCount + viaLayerCount) % numPenColors;
    int routePatternIdx = routingLayerCount % numRoutePatterns;
    int viaPatternIdx = viaLayerCount % numViaPatterns;
    // OpenRoadPen::_penPatterns
    switch (layerType) {
      case odb::dbTechLayerType::ROUTING: {
        GLLayer* tl = new GLLayer(name, id, ORLayerType::METAL_LAYER);
        top->addChildLayer(tl);

        GLPen* tpen
            = new GLPen(penColors[colorIdx], penRoutePatterns[routePatternIdx]);
        tl->setLayerPen(tpen);

        routingLayerCount++;

        break;
      }
      case odb::dbTechLayerType::CUT: {
        GLLayer* tl = new GLLayer(name, id, ORLayerType::VIA_LAYER);
        top->addChildLayer(tl);

        GLPen* tpen
            = new GLPen(penColors[colorIdx], penViaPatterns[viaPatternIdx]);
        tl->setLayerPen(tpen);

        viaLayerCount++;

        break;
      }
      case odb::dbTechLayerType::MASTERSLICE:
      case odb::dbTechLayerType::OVERLAP:
      case odb::dbTechLayerType::IMPLANT:
      case odb::dbTechLayerType::NONE: {
        break;
      }
    }
  }
  std::cout << "Created " << routingLayerCount + 1 << " Routing Layers, "
            << viaLayerCount << " Via Layers" << std::endl;
}

void OpenRoadLayoutContext::populateDesignLayers() const
{
  GLLayer* root = GLLayer::getRootLayer();
  if (!root)
    return;

  const std::string topLayerNames[] = {"Instance"};
  const std::string majorLayerNames[] = {"CellType"};
  const std::string minorLayerNames[] = {"Std cell",
                                         "Macro",
                                         "Fill Cell",
                                         "Cover cell",
                                         "Physical cell",
                                         "IO Cell",
                                         "Black Box",
                                         "IO Pin"};
  const int minorLayerIds[] = {DESIGN_STD_CELL_LAYER,
                               DESIGN_MACRO_LAYER,
                               DESIGN_FILLER_CELL_LAYER,
                               DESIGN_COVERCELL_LAYER,
                               DESIGN_PHYSICAL_CELL_LAYER,
                               DESIGN_IO_CELL_LAYER,
                               DESIGN_BLACKBOX_LAYER,
                               DESIGN_IO_LAYER};
  const ORPenPatternType minorLayerPatterns[]
      = {OpenRoadUI::OR_LEFT_DIAG_PAT,
         OpenRoadUI::OR_VERTICAL_SPARSE_PAT,
         OpenRoadUI::OR_RIGHT_DIAG_PAT,
         OpenRoadUI::OR_LEFT_DIAG_SPARSE_PAT,
         OpenRoadUI::OR_VERTICAL_PAT,
         OpenRoadUI::OR_CHECK_PAT,
         OpenRoadUI::OR_HORIZONTAL_PAT,
         OpenRoadUI::OR_CROSS_PAT};

  const std::string minorLayerColors[] = {std::string("light blue"),
                                          std::string("blue"),
                                          std::string("lilac"),
                                          std::string("green"),
                                          std::string("cyan"),
                                          std::string("brown"),
                                          std::string("purple"),
                                          std::string("yellow")};

  for (int t = 0; t < sizeof(topLayerNames) / sizeof(topLayerNames[0]); t++) {
    GLLayer* top = new GLLayer(
        topLayerNames[t], DESIGN_INSTANCE_LAYER, ORLayerType::DESIGN_LAYER);
    root->addChildLayer(top);
    top->setLayerVisible(true);
    top->setLayerSelectable(false);

    GLPen* tpen = new GLPen(std::string("lilac"), OpenRoadUI::OR_FILL_NONE_PAT);
    top->setLayerPen(tpen);

    for (int m = 0; m < sizeof(majorLayerNames) / sizeof(majorLayerNames[0]);
         m++) {
      GLLayer* mid = new GLLayer(
          majorLayerNames[m], DESIGN_CELLTYPE_LAYER, ORLayerType::DESIGN_LAYER);
      top->addChildLayer(mid);
      mid->setLayerVisible(true);
      mid->setLayerSelectable(false);
      GLPen* mpen
          = new GLPen(std::string("bronze"), OpenRoadUI::OR_FILL_NONE_PAT);
      mid->setLayerPen(mpen);
      for (int minor = 0;
           minor < sizeof(minorLayerNames) / sizeof(minorLayerNames[0]);
           minor++) {
        GLLayer* leaf = new GLLayer(minorLayerNames[minor],
                                    minorLayerIds[minor],
                                    ORLayerType::DESIGN_LAYER);
        mid->addChildLayer(leaf);
        if (DESIGN_FILLER_CELL_LAYER == minorLayerIds[minor])
          leaf->setLayerVisible(false);
        else
          leaf->setLayerVisible(true);
        leaf->setLayerSelectable(true);
        GLPen* lpen
            = new GLPen(minorLayerColors[minor], minorLayerPatterns[minor]);
        leaf->setLayerPen(lpen);
      }
    }
  }
}

void OpenRoadLayoutContext::populateCanvases() const
{
  // TBD
}

std::string OpenRoadLayoutContext::getShapeInfo(const GLShape* shp,
                                                uint layerIdx,
                                                int dbId)
{
  (void) dbId;
  GLLayer* layer = GLLayer::getLayerAt(layerIdx);
  auto layerType = layer->getLayerType();
  void* shpUserData = shp->getUserData();
  if (!shpUserData)
    std::cout << "Shape User data is Null\n";
  if (dbId != -1) {
    if (shpUserData != nullptr
        && (layerIdx == DESIGN_STD_CELL_LAYER || layerIdx == DESIGN_MACRO_LAYER
            || layerIdx == DESIGN_FILLER_CELL_LAYER
            || layerIdx == DESIGN_IO_CELL_LAYER)) {
      odb::dbInst* inst = static_cast<odb::dbInst*>(shpUserData);
      return inst->getName();
    } else if (shpUserData != nullptr) {
      odb::dbObject* dbObj = static_cast<odb::dbObject*>(shpUserData);
      std::string objname(dbObj->getObjName());
      odb::dbObjectType type = dbObj->getObjectType();
      switch (type) {
        case odb::dbSWireObj: {
          objname
              = std::string(((odb::dbSWire*) dbObj)->getNet()->getConstName());
          break;
        }
        case odb::dbWireObj: {
          objname
              = std::string(((odb::dbWire*) dbObj)->getNet()->getConstName());
          break;
        }
        case odb::dbNetObj: {
          objname = std::string(((odb::dbNet*) dbObj)->getConstName());
          break;
        }
        case odb::dbBPinObj: {
          objname
              = std::string(((odb::dbBPin*) dbObj)->getBTerm()->getConstName());
          break;
        }
        default:
          break;
      }
      return objname;
    }
  } else if (shpUserData != nullptr
             && (layerType == DUMMY_LAYER_1 || layerType == DUMMY_LAYER_2
                 || layerType == DUMMY_LAYER_3 || layerType == DUMMY_LAYER_4
                 || layerType == DUMMY_LAYER_5 || layerType == DUMMY_LAYER_6)) {
    const char* objName = static_cast<const char*>(shpUserData);
    std::string retName(objName);
    return retName;
  }
  return "No Info";
}

OpenRoadLayoutContext::OpenRoadLayoutContext(odb::dbDatabase* p_orDb)
    : p_ORDb_(p_orDb)
{
  _allContexts[p_orDb] = this;
}

OpenRoadLayoutContext::~OpenRoadLayoutContext()
{
  GLLayer* root = GLLayer::getRootLayer();
  delete root;
  _allContexts[p_ORDb_] = nullptr;
  p_ORDb_ = nullptr;
}

// static
OpenRoadLayoutContext* OpenRoadLayoutContext::getContextForDb(
    odb::dbDatabase* db)
{
  auto itr = _allContexts.find(db);
  if (itr == _allContexts.end()) {
    OpenRoadLayoutContext* p_ctxt = new OpenRoadLayoutContext(db);
  }
  return _allContexts[db];
}
void OpenRoadLayoutContext::deleteAllContexts()
{
  for (auto i : _allContexts) {
    delete (i.second);
  }
}
}  // namespace OpenRoadUI
