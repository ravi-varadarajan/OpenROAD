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

#include "pygui/openroadPyIntf.h"

#include <cassert>
#ifndef SWIG
#include "GL/gl.h"
#include "opendb/db.h"
#include "opendb/dbShape.h"
#include "pygui/openroadIntf.h"
#endif

#include <iostream>

#include "pygui/openroadCanvas.h"
#include "pygui/openroadView.h"

namespace OpenRoadUI {
OpenRoadPythonIntf* OpenRoadPythonIntf::_sOrPyIntf = nullptr;
OpenRoadPythonIntf::OpenRoadPythonIntf()
{
  assert(_sOrPyIntf == nullptr);  // this is a singleton
  _sOrPyIntf = this;
   std::cout << "Python Object Created. Its printed from c++\n" ;
}

OpenRoadPythonIntf::~OpenRoadPythonIntf()
{
  // TBD
}

// Python will be able to call these functions in C++
void OpenRoadPythonIntf::displayMessage(const char* msg)
{
  if (msg)
    std::cout << msg << std::endl << "\n";
}

void OpenRoadPythonIntf::updateCanvas(GLCanvas* cnv)
{
  auto openroadIntf = OpenRoadIntf::getOpenRoadIntfInst();
  if (cnv->getShapeCountFromAllLayers() == 0)
    openroadIntf->populateDbCanvas(cnv);
}

openRoadTclRes OpenRoadPythonIntf::executeTclCommand(const char* cmdStr)
{
  openRoadTclRes retVal;
#ifndef SWIG
  auto openroadIntf = OpenRoadIntf::getOpenRoadIntfInst();
  std::string cmd(cmdStr);
  std::string res;

  retVal = openroadIntf->executeTclCommand(cmd, res);
#endif
  return retVal;
}

void OpenRoadPythonIntf::initOnce()
{
  static bool onceFlag = false;
  if (onceFlag)
    return;
#ifndef SWIG
  auto openroadIntf = OpenRoadIntf::getOpenRoadIntfInst();
  openroadIntf->initOnce();
#endif
  onceFlag = true;
}

bool OpenRoadPythonIntf::showObjectInView(std::string objName,
                                          OpenRoadUI::DBObjectType objType,
                                          OpenRoadUI::ViewOpType viewOp,
                                          OpenRoadUI::GLView* view,
                                          bool deselectPrior)
{
  auto openroadIntf = OpenRoadIntf::getOpenRoadIntfInst();
  auto defUnits = openroadIntf->getDefUnits();
  auto viewCnv = view->getTopCanvas();

  // std::cout << "In OpenRoadPythonIntf::showObjectInView for object :
  // "<<objName<< " Type : "<<objType<<" ViewOp : "<<viewOp << std::endl ;

  switch (objType) {
    case DB_INSTANCE: {
      std::vector<odb::dbInst*> insts;
      openroadIntf->findInstances(objName, insts);
      if (insts.empty())
        return false;
      bool isFirst = true;
      for (auto inst : insts) {
        odb::dbBox* box = inst->getBBox();
        if (!box)
          continue;
        ORRect_t instShpBB(
            ORPoint_t(box->xMin() / defUnits, box->yMin() / defUnits),
            ORPoint_t(box->xMax() / defUnits, box->yMax() / defUnits));
        uint shpLayerIdx = DESIGN_STD_CELL_LAYER;
        if (inst->getMaster()->isBlock())
          shpLayerIdx = DESIGN_MACRO_LAYER;
        else if (inst->getMaster()->isFiller())
          shpLayerIdx = DESIGN_FILLER_CELL_LAYER;
        else if (inst->getMaster()->isPad())
          shpLayerIdx = DESIGN_IO_CELL_LAYER;
        std::vector<uint> shpLayers;
        shpLayers.push_back(shpLayerIdx);
        GLShape* shp = viewCnv->findFirstMatchingShape(
            shpLayers, (void*) inst, &instShpBB);
        view->addShapeOnView(
            shp, shpLayerIdx, isFirst ? deselectPrior : false, viewOp);
        isFirst = false;
      }
      // std::cout << "Number of Selected Shapes =
      // "<<view->getSelectedShapesCount() << std::endl ;
      return true;
    } break;
    case DB_NET: {
      return false;
    } break;
    case DB_PIN: {
      std::vector<odb::dbObject*> pins;
      openroadIntf->findPins(objName, pins);
      if (pins.empty())
        return false;
      bool isFirst = true;
      for (auto pin : pins) {
        if (pin->getObjectType() == odb::dbObjectType::dbBTermObj) {
          odb::dbBTerm* term = (odb::dbBTerm*) pin;
          for (odb::dbBPin* termShape : term->getBPins()) {
            odb::dbPlacementStatus status = termShape->getPlacementStatus();
            if (status == odb::dbPlacementStatus::NONE
                || status == odb::dbPlacementStatus::UNPLACED) {
              continue;
            }
            odb::dbSet<odb::dbBox> boxes = termShape->getBoxes();
            if(boxes.empty())
               continue;

            odb::dbBox* box = *boxes.begin();
            ORRect_t termShapeBB(
                ORPoint_t(box->xMin() / defUnits, box->yMin() / defUnits),
                ORPoint_t(box->xMax() / defUnits, box->yMax() / defUnits));
            uint shpLayerIdx = box->getTechLayer()->getId();
            std::vector<uint> shpLayers;
            shpLayers.push_back(shpLayerIdx);
            GLShape* shp = viewCnv->findFirstMatchingShape(
                shpLayers, (void*) termShape, &termShapeBB);
            view->addShapeOnView(
                shp, shpLayerIdx, isFirst ? deselectPrior : false, viewOp);
            isFirst = false;
          }
        }
      }
      return true;
    } break;
    default:
      return false;
  }
}
}  // namespace OpenRoadUI
