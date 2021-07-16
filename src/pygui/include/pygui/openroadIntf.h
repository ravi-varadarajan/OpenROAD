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

#include <tcl.h>

#include <utility>
#include <vector>

#include "opendb/db.h"
#include "opendb/dbBlockCallBackObj.h"
#include "ord/OpenRoad.hh"
#include "pygui/openroadPyIntf.h"
#include "pygui/openroadUiEnums.h"
#include "pygui/staGui.h"

namespace OpenRoadUI {
class GLCanvas;
class GLView;
class GLLayer;

class OpenRoadIntf : public ord::OpenRoad::Observer,
                     public odb::dbBlockCallBackObj
{
 public:
  friend class OpenRoadPythonIntf;
  ~OpenRoadIntf();

  // From ord::OpenRoad::Observer
  virtual void postReadLef(odb::dbTech* tech, odb::dbLib* library) override;
  virtual void postReadDef(odb::dbBlock* block) override;
  virtual void postReadDb(odb::dbDatabase* db) override;
  virtual void inDbPostMoveInst(odb::dbInst* inst) override;
  void setOpenRoad(ord::OpenRoad* openroad);

  void printMessageInPython(const std::string& msg);
  void displayMessageFromPython(const char* msg);

  Tcl_Interp* getTclInterp();
  openRoadTclRes executeTclCommand(const std::string& cmdStr,
                                   std::string& result);

  void buildDbCanvas(odb::dbDatabase* db);
  void populateDbCanvas(GLCanvas* cnv);

  void findInstances(std::string objName, std::vector<odb::dbInst*>& insts);
  void findPins(std::string objName, std::vector<odb::dbObject*>& pins);

  void drawExternalRenderersForLayer(GLView* view,
                                     GLLayer* layer,
                                     GLCanvas* canvas);
  void drawExternalRenderersForObjects(GLView* view, GLCanvas* canvas);

  double getDefUnits();
  double getLefUnits();

  static void clearGlobals();
  static OpenRoadIntf* getOpenRoadIntfInst();

 private:
  OpenRoadIntf();
  void addDieToCanvas(odb::dbBlock* block, GLCanvas* p_cnv);
  void addMasterToCanvas(odb::dbMaster* master);
  void addViaToCanvas(odb::dbNet* net,
                      odb::dbShape* shape,
                      int x,
                      int y,
                      GLCanvas* p_cnv);

  void addNetToCanvas(odb::dbNet* net, GLCanvas* p_cnv);
  void addSNetToCanvas(odb::dbNet* net, GLCanvas* p_cnv);
  void addTermToCanvas(odb::dbBPin* pin, GLCanvas* p_cnv);
  void addInstToCanvas(odb::dbInst* inst, GLCanvas* p_cnv);
  void addFillsToCanvas(odb::dbBlock* block, GLCanvas* p_cnv);

  static int channelOutput(ClientData instanceData,
                           const char* buf,
                           int toWrite,
                           int* errorCodePtr);

  bool setupTcl();
  void initOnce();

  Tcl_Interp* _mOrTclInterp;
  ord::OpenRoad* _openroad;
  gui::staGui* _dbGuiIntf;

  std::vector<std::pair<std::string, std::string>> _mExecutedCommands;
  static Tcl_ChannelType stdoutChannelType;
  odb::dbBlock* block_;
  odb::dbTech* tech_;
  GLCanvas* _p_cnv;
  int changedInsts_;
};
};  // namespace OpenRoadUI
