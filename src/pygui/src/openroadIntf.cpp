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

#include "pygui/openroadIntf.h"

#include <GL/gl.h>

#include <iostream>

#include "gui/gui.h"
#include "opendb/db.h"
#include "opendb/dbShape.h"
#include "pygui/openroadCanvas.h"
#include "pygui/openroadLayer.h"
#include "pygui/openroadLayoutContext.h"
#include "pygui/openroadPyIntf.h"
#include "pygui/openroadShape.h"
#include "pygui/openroadView.h"
#include "pygui/pygui.h"
#include "pygui/staGui.h"
extern bool pygui_mode; // from Main.cc

// using namespace OpenRoadUI ;
#define SKIP_POWERGROUND 1

int channelClosePy(ClientData instanceData, Tcl_Interp* interp)
{
  // This channel should never be closed
  return EINVAL;
}

void channelWatchPy(ClientData instanceData, int mask)
{
  // watch is not supported inside OpenROAD GUI
}

namespace gui {

class PyGuiPainter : public Painter
{
 public:
  PyGuiPainter(OpenRoadUI::GLView* view, OpenRoadUI::GLCanvas* cnv)
      : view_(view), canvas_(cnv), lastColor_(0, 0, 0)
  {
    auto topLayer = canvas_->getTopLayerNode();
    std::vector<uint> childLayerIds;
    topLayer->getChildLayerIdsRecurse(childLayerIds, false, false);
    for (auto layerIdx : childLayerIds) {
      auto glLayer = OpenRoadUI::GLLayer::getLayerAt(layerIdx);
      if (glLayer->getLayerUserData() == nullptr)
        continue;
      odb::dbTechLayer* techLayer
          = static_cast<odb::dbTechLayer*>(glLayer->getLayerUserData());
      layerMap_[techLayer] = glLayer;
    }
    intfInst_ = OpenRoadUI::OpenRoadIntf::getOpenRoadIntfInst();
    defUnits = intfInst_->getDefUnits();
    defUnitsSq = defUnits * defUnits;
    GLubyte* pattern
        = OpenRoadUI::GLPen::getPenBytePattern(OpenRoadUI::OR_FILL_SOLID_PAT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_POLYGON_STIPPLE);
    glPolygonStipple(pattern);
  }

  Color getPenColor()
  {
    Color c;
    return c;
  }

  void setPen(odb::dbTechLayer* layer, bool cosmetic) override
  {
    // std::cout
    //    << "PyGuiPainter> Came to Set Pen for dbTechLayer with cosmetic = "
    //    << cosmetic << std::endl;
    if (layerMap_.find(layer) == layerMap_.end())
      return;
    auto glLayer = layerMap_[layer];
    glLayer->getLayerPen()->setGLPenContext();
  }

  void setPen(const Color& color, bool cosmetic, int width) override
  {
    // std::cout << "PyGuiPainter> Came to Set Pen with Color and with cosmetic
    // = "
    //          << cosmetic << std::endl;
    // if (color.r == lastColor_.r && color.g == lastColor_.g
    //    && color.b == lastColor_.b && color.a == lastColor_.a)
    //  return;
    // glColor4i(color.r, color.g, color.b, color.a);
    glColor4f(
        color.r / 255.0, color.g / 255.0, color.b / 255.0, color.a / 255.0);
    // OpenRoadUI::ColorParams cParams(
    //    color.r / 255.0, color.g / 255.0, color.b / 255.0, color.a / 255.0);
    // OpenRoadUI::GLPen glPen(cParams, OpenRoadUI::OR_FILL_SOLID_PAT);
    // glPen.setGLPenContext();
  }

  void setPenWidth(int width)
  {

  }
  void setBrush(odb::dbTechLayer* layer, int alpha) override
  {
    // std::cout << "PyGuiPainter> Came to Set Brush for dbTechLayer with alpha
    // = "
    //          << alpha << std::endl;
    if (layerMap_.find(layer) == layerMap_.end())
      return;
    auto glLayer = layerMap_[layer];
    glLayer->getLayerPen()->setGLPenContext();
  }

  void setBrush(const Color& color) override
  {
    // std::cout << "PyGuiPainter> Came to Set Brush with color\n";
    // if (color.r == lastColor_.r && color.g == lastColor_.g
    //    && color.b == lastColor_.b && color.a == lastColor_.a)
    //  return;
    // glColor4i(color.r, color.g, color.b, color.a);
    glColor4f(
        color.r / 255.0, color.g / 255.0, color.b / 255.0, color.a / 255.0);
    // OpenRoadUI::ColorParams cParams(
    //    color.r / 255.0, color.g / 255.0, color.b / 255.0, color.a / 255.0);
    // OpenRoadUI::GLPen glPen(cParams, OpenRoadUI::OR_FILL_SOLID_PAT);
    // glPen.setGLPenContext();
  }

  void drawGeomShape(const odb::GeomShape* shape) override
  {
    std::cout << "PyGuiPainter> Came to Draw GeomShape\n";
    // TBD
  }

  void drawRect(const odb::Rect& rect, int roundX, int roundY) override
  {
    if (false
        && const_cast<odb::Rect&>(rect).area() / defUnitsSq
               <= view_->getMinDrawableArea()) {
      glBegin(GL_POINTS);
      glVertex2f(rect.xMin() / defUnits, rect.yMin() / defUnits);
      glEnd();
      return;
    }
    // glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    auto llx = rect.xMin() / defUnits;
    auto lly = rect.yMin() / defUnits;
    auto urx = rect.xMax() / defUnits;
    auto ury = rect.yMax() / defUnits;

    glRectf(llx, lly, urx, ury);

    // Now draw the rectangle for the boundary.
    // glBegin(GL_LINE_LOOP);
    // glVertex2f(llx, lly);
    // glVertex2f(urx, lly);
    // glVertex2f(urx, ury);
    // glVertex2f(llx, ury);
    // glEnd();
    // auto intfInst = OpenRoadUI::OpenRoadIntf::getOpenRoadIntfInst();
    // OpenRoadUI::GLRectangle glRect(rect.xMin() / intfInst->getDefUnits(),
    //                               rect.yMin() / intfInst->getDefUnits(),
    //                               rect.xMax() / intfInst->getDefUnits(),
    //                               rect.yMax() / intfInst->getDefUnits());
    // std::cout << "PyGuiPainter> Came to Draw Rect : " << glRect.repr();
    // OpenRoadUI::GLRectShape rectShp(glRect);
    // rectShp.draw(view_, -1);
  }

  void drawLine(const odb::Point& p1, const odb::Point& p2) override
  {
    if (false
        && odb::Point::manhattanDistance(p1, p2) / defUnits
               <= view_->getMinDrawableArea()) {
      glBegin(GL_POINTS);
      glVertex2f(p1.getX() / defUnits, p1.getY() / defUnits);
      glEnd();
      return;
    }
    glBegin(GL_LINES);
    glVertex2f(p1.getX() / defUnits, p1.getY() / defUnits);
    glVertex2f(p2.getX() / defUnits, p2.getY() / defUnits);
    glEnd();
    return;
    // auto intfInst = OpenRoadUI::OpenRoadIntf::getOpenRoadIntfInst();
    // OpenRoadUI::GLSegment seg(
    //    OpenRoadUI::GLPoint2D(p1.getX() / intfInst->getDefUnits(),
    //                          p1.getY() / intfInst->getDefUnits()),
    //    OpenRoadUI::GLPoint2D(p2.getX() / intfInst->getDefUnits(),
    //                          p2.getY() / intfInst->getDefUnits()));
    // std::cout << "PyGuiPainter> Came to Draw Line : " << seg.repr();
    // OpenRoadUI::GLSegmentShape segShp(seg);
    // segShp.draw(view_, -1);
  }
  void drawCircle(int x, int y, int r) override
  {

  }

  void drawString(int x, int y, int offset, const std::string& s) override
  {

  }

  void drawRuler(int x0, int y0, int x1, int y1) override
  {

  }

  void setTransparentBrush() override
  {

  }

 private:
  OpenRoadUI::GLView* view_;
  OpenRoadUI::GLCanvas* canvas_;
  Color lastColor_;
  OpenRoadUI::OpenRoadIntf* intfInst_;
  double defUnits;
  double defUnitsSq;
  // GLubyte* pattern_;
  std::map<odb::dbTechLayer*, OpenRoadUI::GLLayer*> layerMap_;
};
}  // end namespace gui

namespace OpenRoadUI {

Tcl_ChannelType OpenRoadIntf::stdoutChannelType = {
    // Tcl stupidly defines this a non-cost char*
    ((char*) "stdout_channel"),  /* typeName */
    TCL_CHANNEL_VERSION_2,       /* version */
    channelClosePy,              /* closeProc */
    nullptr,                     /* inputProc */
    OpenRoadIntf::channelOutput, /* outputProc */
    nullptr,                     /* seekProc */
    nullptr,                     /* setOptionProc */
    nullptr,                     /* getOptionProc */
    channelWatchPy,              /* watchProc */
    nullptr,                     /* getHandleProc */
    nullptr,                     /* close2Proc */
    nullptr,                     /* blockModeProc */
    nullptr,                     /* flushProc */
    nullptr,                     /* handlerProc */
    nullptr,                     /* wideSeekProc */
    nullptr,                     /* threadActionProc */
    nullptr                      /* truncateProc */
};

// Static Function...
OpenRoadIntf* OpenRoadIntf::getOpenRoadIntfInst()
{
  static OpenRoadIntf _sOrIntf;
  return &_sOrIntf;
}

OpenRoadIntf::OpenRoadIntf()
    : _mOrTclInterp(nullptr),
      _openroad(nullptr),
      _dbGuiIntf(nullptr),
      _p_cnv(nullptr),
      changedInsts_(0)
{
  // std::cout << "Built OpenRoadIntf Singleton Object\n" ;
  if (pygui_mode == false)
    return;

  setupTcl();
  auto* open_road = ord::OpenRoad::openRoad();
  open_road->addObserver(this);
  setOpenRoad(open_road);
  // std::cout << "Added Observer To Singleton Object\n" ;
}

OpenRoadIntf::~OpenRoadIntf()
{
  // TBD
  std::cout << "Cleaning context..." << std::endl;
  OpenRoadLayoutContext::deleteAllContexts();
  delete _dbGuiIntf;
}

void OpenRoadIntf::setOpenRoad(ord::OpenRoad* openroad)
{
  _openroad = openroad;
  if (_dbGuiIntf)
    delete _dbGuiIntf;
  _dbGuiIntf = new gui::staGui(_openroad);
}

void OpenRoadIntf::postReadLef(odb::dbTech* tech, odb::dbLib* library)
{
  OpenRoadLayoutContext::deleteAllContexts();
}

void OpenRoadIntf::findInstances(std::string pattern,
                                 std::vector<odb::dbInst*>& insts)
{
  _dbGuiIntf->findInstances(pattern, insts);
  return;
}

void OpenRoadIntf::findPins(std::string pattern,
                            std::vector<odb::dbObject*>& pins)
{
  _dbGuiIntf->findPins(pattern, pins);
  return;
}

void OpenRoadIntf::drawExternalRenderersForLayer(GLView* view,
                                                 GLLayer* layer,
                                                 GLCanvas* canvas)
{
  // std::cout << "Came to Draw Shape from External Renderer for Layer "
  //          << layer->getLayerName() << " in view : " << view->getViewName()
  //          << std::endl
  //          << std::flush;
  auto layerUserData = layer->getLayerUserData();
  if (view->getViewName() != "layoutView" || layerUserData == nullptr) {
    // std::cout << "Skipping External Renderer Draw\n" << std::flush;
    return;
  }
  odb::dbTechLayer* techLayer = static_cast<odb::dbTechLayer*>(layerUserData);
  // Create a Local Painter
  gui::PyGuiPainter guiPainter(view, canvas);
  auto& renderers = gui::Gui::get()->renderers();
  int idx = 1;
  for (auto* renderer : renderers) {
    // std::cout << "DRAWING RENDERER " << idx++
    //          << " For Layer : " << layer->getLayerName() << std::endl
    //          << std::flush;
    renderer->drawLayer(techLayer, guiPainter);
  }
  // TBD
}

void OpenRoadIntf::drawExternalRenderersForObjects(GLView* view,
                                                   GLCanvas* canvas)
{
  if (view->getViewName() != "layoutView") {
    // std::cout << "Skipping External Renderer Draw\n" << std::flush;
    return;
  }
  gui::PyGuiPainter guiPainter(view, canvas);
  auto& renderers = gui::Gui::get()->renderers();
  int idx = 1;
  for (auto* renderer : renderers) {
    // std::cout << "DRAWING RENDERER " << idx++ << " For Objects " << std::endl
    //          << std::flush;
    renderer->drawObjects(guiPainter);
  }
}

void OpenRoadIntf::postReadDef(odb::dbBlock* block)
{
  auto db = block->getChip()->getDb();
  std::cout << "OpenRoadIntf::Observer Got the event postReadDef with dbId = "
            << std::endl;
  delete _p_cnv;
  _p_cnv = nullptr;
  buildDbCanvas(db);
  OpenRoadPythonIntf::get()->setDatabaseId(db->getId());
  addOwner(block);
}

void OpenRoadIntf::postReadDb(odb::dbDatabase* db)
{
  std::cout << "OpenRoadIntf::Observer Got the event postReadDb in C++\n";
  auto dbId = db->getId();
  // std::cout << "OpenRoadIntf::Observer Got the event postReadDb with dbId = "
  // << dbId <<std::endl ;
  buildDbCanvas(db);
  OpenRoadPythonIntf::get()->setDatabaseId(dbId);
  if (db->getChip() && db->getChip()->getBlock())
    addOwner(db->getChip()->getBlock());
  // auto ctxt = OpenRoadLayoutContext::getContext(topDbPtr) ;
  // ctxt->populateLayers() ;
}

void OpenRoadIntf::inDbPostMoveInst(odb::dbInst* inst)
{
  changedInsts_++;
}
// OpenRoadCanvas*
// OpenRoadIntf::getCanvasFor(objectType typ)
// {
//     OpenRoadLayer *layer = getLayerForObjectType(typ);
//     OpenRoadCanvas* p_cnv = new OpenRoadCanvas(layer, "Layout") ;
//     return p_cnv;

// }

void OpenRoadIntf::buildDbCanvas(odb::dbDatabase* db_)
{
  if (!db_) {
    return;
  }

  odb::dbChip* chip = db_->getChip();
  if (!chip) {
    return;
  }

  std::cout << "Building Canvas..." << std::endl;

  block_ = chip->getBlock();
  if (!block_)
    return;
  tech_ = db_->getTech();

  OpenRoadLayoutContext* cntxt = OpenRoadLayoutContext::getContextForDb(db_);
  std::cout << "Created context..." << std::endl;
  cntxt->populateLayers();
  std::cout << "Created Layers..." << std::endl;
  GLLayer* topLayer = GLLayer::getRootLayer();
  if (!topLayer) {
    printf("No layers found to add shapes\n");
    return;
  }

  _p_cnv = new GLCanvas(topLayer, "OR");
  _p_cnv->setUserData((void*) db_);
  std::cout << "Created empty Canvas..." << std::endl;
  populateDbCanvas(_p_cnv);
  return;
}

void OpenRoadIntf::populateDbCanvas(GLCanvas* _p_cnv)
{
  if (_p_cnv->getUserData() == nullptr)
    return;
  odb::dbDatabase* db_ = static_cast<odb::dbDatabase*>(_p_cnv->getUserData());
  odb::dbChip* chip = db_->getChip();
  if (!chip) {
    return;
  }

  _p_cnv->clearCanvasLayer(-1, true);

  std::cout << "Populating Canvas..." << std::endl;

  _p_cnv->setBlockOp(true);
  addDieToCanvas(block_, _p_cnv);

  for (odb::dbNet* net : block_->getNets()) {
    addNetToCanvas(net, _p_cnv);
    //#ifndef SKIP_POWERGROUND
    addSNetToCanvas(net, _p_cnv);
    //#endif
  }

  std::cout << "Added Nets..." << std::endl;

  std::vector<odb::dbMaster*> instMasters;
  block_->getMasters(instMasters);
  for (auto instMaster : instMasters) {
    addMasterToCanvas(instMaster);
  }

  for (odb::dbInst* inst : block_->getInsts()) {
    // if(inst->getMaster()->isFiller())
    //   continue;

    odb::dbPlacementStatus status = inst->getPlacementStatus();
    if (status == odb::dbPlacementStatus::NONE
        || status == odb::dbPlacementStatus::UNPLACED) {
      continue;
    }
    addInstToCanvas(inst, _p_cnv);
  }

  std::cout << "Added Instances..." << std::endl;

  for (odb::dbBTerm* term : block_->getBTerms()) {
    for (odb::dbBPin* pin : term->getBPins()) {
      odb::dbPlacementStatus status = pin->getPlacementStatus();
      if (status == odb::dbPlacementStatus::NONE
          || status == odb::dbPlacementStatus::UNPLACED) {
        continue;
      }
      addTermToCanvas(pin, _p_cnv);
    }
  }
  std::cout << "Added Terms..." << std::endl;

  addFillsToCanvas(block_, _p_cnv);

  _p_cnv->setBlockOp(false);
  std::cout << "Created canvas " << _p_cnv << " with "
            << _p_cnv->getShapeCountFromAllLayers()
            << " shapes, its BBox = " << _p_cnv->getFullBBox() << std::endl;
}

void OpenRoadIntf::addDieToCanvas(odb::dbBlock* block, GLCanvas* cnv)
{
  odb::Rect canvasBox;
  block->getBBox()->getBox(canvasBox);

  odb::Rect dieBox;
  block->getDieArea(dieBox);
  if (dieBox.area() == 0)
    return;
  canvasBox.merge(dieBox);
  ORRect_t shapeToAdd(ORPoint_t(canvasBox.xMin() / getDefUnits(),
                                canvasBox.yMin() / getDefUnits()),
                      ORPoint_t(canvasBox.xMax() / getDefUnits(),
                                canvasBox.yMax() / getDefUnits()));
  GLShape* rectShape = new GLRectShape(shapeToAdd);
  rectShape->setUserData((void*) block);
  (void) cnv->addShape(rectShape, DESIGN_INSTANCE_LAYER);
}

void OpenRoadIntf::addMasterToCanvas(odb::dbMaster* master)
{
  const char* masterName = master->getConstName();
  GLLayer* topLayer = GLLayer::getRootLayer();
  GLCanvas* cnv = new GLCanvas(topLayer, masterName);
  if (!cnv) {
    return;
  }
  cnv->setBlockOp(true);
  odb::Rect boundary;
  master->getPlacementBoundary(boundary);
  double llx = boundary.xMin() / getLefUnits();
  double lly = boundary.yMin() / getLefUnits();
  double urx = boundary.xMax() / getLefUnits();
  double ury = boundary.yMax() / getLefUnits();
  double xshift = (urx - llx) * 0.2;
  double yshift = (ury - lly) * 0.2;
  GLSegment seg(GLPoint2D(llx + xshift, lly), GLPoint2D(llx, lly + yshift));
  GLSegmentShape* segShape = new GLSegmentShape(seg);
  (void) cnv->addShape(segShape, DESIGN_INSTANCE_LAYER);

#ifdef DRAW_MASTER_OUTLINE  // Need this control since conflicts with instance
                            // boundary
  ORRect_t shapeToAdd(ORPoint_t(llx, lly), ORPoint_t(urx, ury));
  GLShape* rectShape = new GLRectShape(shapeToAdd);
  rectShape->setUserData((void*) master);
  (void) p_cnv->addShape(rectShape, DESIGN_INSTANCE_LAYER);
#endif

  for (auto term : master->getMTerms()) {  // dbSet
#ifdef SKIP_POWERGROUND
    if (term->getSigType() == odb::dbSigType::POWER
        || term->getSigType() == odb::dbSigType::GROUND)
      continue;
#endif
    for (auto pin : term->getMPins()) {
      for (auto pinBox : pin->getGeometry()) {
        odb::dbTechLayer* layer = pinBox->getTechLayer();
        if (!layer)
          continue;
        odb::dbTechLayerType type = layer->getType();
        if (type != odb::dbTechLayerType::ROUTING
            && type != odb::dbTechLayerType::CUT)
          continue;

        ORRect_t shapeToAdd(ORPoint_t(pinBox->xMin() / getLefUnits(),
                                      pinBox->yMin() / getLefUnits()),
                            ORPoint_t(pinBox->xMax() / getLefUnits(),
                                      pinBox->yMax() / getLefUnits()));
        GLShape* rectShape = new GLRectShape(shapeToAdd);
        rectShape->setUserData((void*) (pin));
        (void) cnv->addShape(rectShape, layer->getId());
      }
    }
  }

  for (auto obsBox : master->getObstructions()) {
    if (!obsBox->getTechLayer())
      continue;
    if (obsBox->getTechLayer()->getType() != odb::dbTechLayerType::ROUTING
        && obsBox->getTechLayer()->getType() != odb::dbTechLayerType::CUT)
      continue;
    ORRect_t shapeToAdd(ORPoint_t(obsBox->xMin() / getLefUnits(),
                                  obsBox->yMin() / getLefUnits()),
                        ORPoint_t(obsBox->xMax() / getLefUnits(),
                                  obsBox->yMax() / getLefUnits()));
    GLShape* rectShape = new GLRectShape(shapeToAdd);
    rectShape->setUserData((void*) (obsBox));
    (void) cnv->addShape(rectShape, obsBox->getTechLayer()->getId());
  }
  cnv->setBlockOp(false);
  // master->transform(dbTransform& t); To be used if shapes to be transformed
  // before use?
}
void OpenRoadIntf::addViaToCanvas(odb::dbNet* net,
                                  odb::dbShape* shape,
                                  int x,
                                  int y,
                                  GLCanvas* cnv)
{
  if (shape->getType() == odb::dbShape::TECH_VIA) {
    odb::dbTechVia* via = shape->getTechVia();
    for (odb::dbBox* box : via->getBoxes()) {
      if (box->getTechLayer()->getType() != odb::dbTechLayerType::ROUTING
          && box->getTechLayer()->getType() != odb::dbTechLayerType::CUT)
        continue;
      ORRect_t shapeToAdd(ORPoint_t((x + box->xMin()) / getDefUnits(),
                                    (y + box->yMin()) / getDefUnits()),
                          ORPoint_t((x + box->xMax()) / getDefUnits(),
                                    (y + box->yMax()) / getDefUnits()));
      GLShape* rectShape = new GLRectShape(shapeToAdd);
      rectShape->setUserData((void*) net);
      (void) cnv->addShape(rectShape, box->getTechLayer()->getId());
    }
  } else {
    odb::dbVia* via = shape->getVia();
    for (odb::dbBox* box : via->getBoxes()) {
      if (box->getTechLayer()->getType() != odb::dbTechLayerType::ROUTING
          && box->getTechLayer()->getType() != odb::dbTechLayerType::CUT)
        continue;
      ORRect_t shapeToAdd(ORPoint_t((x + box->xMin()) / getDefUnits(),
                                    (y + box->yMin()) / getDefUnits()),
                          ORPoint_t((x + box->xMax()) / getDefUnits(),
                                    (y + box->yMax()) / getDefUnits()));
      GLShape* rectShape = new GLRectShape(shapeToAdd);
      rectShape->setUserData((void*) via);
      (void) cnv->addShape(rectShape, box->getTechLayer()->getId());
    }
  }
}

void OpenRoadIntf::addNetToCanvas(odb::dbNet* net, GLCanvas* cnv)
{
  odb::dbWire* wire = net->getWire();

  if (wire == nullptr)
    return;

  odb::dbWireShapeItr itr;
  odb::dbShape s;

  for (itr.begin(wire); itr.next(s);) {
    int shapeId = itr.getShapeId();
    if (s.isVia()) {
      addViaToCanvas(net, &s, itr._prev_x, itr._prev_y, cnv);
    } else {
      ORRect_t shapeToAdd(
          ORPoint_t(s.xMin() / getDefUnits(), s.yMin() / getDefUnits()),
          ORPoint_t(s.xMax() / getDefUnits(), s.yMax() / getDefUnits()));
      GLShape* rectShape = new GLRectShape(shapeToAdd);
      rectShape->setUserData((void*) net);
      (void) cnv->addShape(rectShape, s.getTechLayer()->getId());
    }
  }
}

void OpenRoadIntf::addSNetToCanvas(odb::dbNet* net, GLCanvas* cnv)
{
  std::vector<odb::dbShape> shapes;
  std::map<int, int> layerObjects;
  for (odb::dbSWire* swire : net->getSWires()) {
    for (odb::dbSBox* box : swire->getWires()) {
      if (box->isVia()) {
        box->getViaBoxes(shapes);
        for (auto shape : shapes) {
          if (shape.getTechLayer()->getType() != odb::dbTechLayerType::ROUTING
              && shape.getTechLayer()->getType() != odb::dbTechLayerType::CUT)
            continue;
          ORRect_t shapeToAdd(ORPoint_t(shape.xMin() / getDefUnits(),
                                        shape.yMin() / getDefUnits()),
                              ORPoint_t(shape.xMax() / getDefUnits(),
                                        shape.yMax() / getDefUnits()));
          GLShape* rectShape = new GLRectShape(shapeToAdd);
          rectShape->setUserData((void*) net);
          (void) cnv->addShape(rectShape, shape.getTechLayer()->getId());
          layerObjects[shape.getTechLayer()->getId()] += 1;
        }
      } else {
        if (box->getTechLayer()->getType() != odb::dbTechLayerType::ROUTING
            && box->getTechLayer()->getType() != odb::dbTechLayerType::CUT)
          continue;
        // std::cout << "Adding wire shapes" << std::endl;
        ORRect_t shapeToAdd(
            ORPoint_t(box->xMin() / getDefUnits(), box->yMin() / getDefUnits()),
            ORPoint_t(box->xMax() / getDefUnits(),
                      box->yMax() / getDefUnits()));
        GLShape* rectShape = new GLRectShape(shapeToAdd);
        rectShape->setUserData((void*) net);
        (void) cnv->addShape(rectShape, box->getTechLayer()->getId());
        layerObjects[box->getTechLayer()->getId()] += 1;
      }
    }
  }
  for (auto layer : layerObjects) {
    if (0 && layer.second > 50000) {
      std::cout << "Disabling visibility of power shape layer " << layer.first
                << " with " << layer.second << " shapes" << std::endl;
      std::cout << "You can enable if needed in layer dialog box" << std::endl;
      cnv->getTopLayerNode()->getLayerAt(layer.first)->setLayerVisible(false);
    }
  }
  // std::cout << "Added " << viaShapes << " Via shapes" << " and " <<
  // wireShapes << " wire shapes" << std::endl;
}

void OpenRoadIntf::addTermToCanvas(odb::dbBPin* pin, GLCanvas* cnv)
{
  odb::dbSet<odb::dbBox> boxes = pin->getBoxes();
  if(boxes.empty())
     return;

  odb::dbBox* box = *boxes.begin();

  GLRectangle shapeRect(
      ORPoint_t(box->xMin() / getDefUnits(), box->yMin() / getDefUnits()),
      ORPoint_t(box->xMax() / getDefUnits(), box->yMax() / getDefUnits()));
  GLShape* rectShape = new GLMarkerShape(shapeRect, OR_CIRCLE_MARKER);
  rectShape->setUserData((void*) pin);
  (void) cnv->addShape(rectShape, box->getTechLayer()->getId());
}

void OpenRoadIntf::addInstToCanvas(odb::dbInst* inst, GLCanvas* cnv)
{
  odb::dbBox* box = inst->getBBox();
  if (!box) {
    return;
  }
  odb::dbMaster* master = inst->getMaster();
  if (!master)
    return;

  int layerIdx = DESIGN_STD_CELL_LAYER;
  if (master->isBlock())
    layerIdx = DESIGN_MACRO_LAYER;
  else if (master->isFiller())
    layerIdx = DESIGN_FILLER_CELL_LAYER;
  else if (master->isPad())
    layerIdx = DESIGN_IO_CELL_LAYER;

  ORRect_t shapeToAdd(
      ORPoint_t(box->xMin() / getDefUnits(), box->yMin() / getDefUnits()),
      ORPoint_t(box->xMax() / getDefUnits(), box->yMax() / getDefUnits()));

  GLCanvas* masterCanvas = GLCanvas::getCanvas(master->getConstName());
  if (!masterCanvas) {
    std::cout << "GUI_INTERNAL_ERROR: Master canvas not found for "
              << master->getConstName() << std::endl;
    return;
  }
  odb::dbOrientType dbOrient = inst->getOrient();
  GLCanvasInstShape* instShape
      = new GLCanvasInstShape(masterCanvas, shapeToAdd, dbOrient);
  instShape->setUserData((void*) inst);
  (void) cnv->addShape(instShape, layerIdx);
}

void OpenRoadIntf::addFillsToCanvas(odb::dbBlock* block, GLCanvas* cnv)
{
  for (odb::dbFill* fill : block->getFills()) {
    odb::Rect rect;
    fill->getRect(rect);
    ORRect_t shapeToAdd(
        ORPoint_t(rect.xMin() / getDefUnits(), rect.yMin() / getDefUnits()),
        ORPoint_t(rect.xMax() / getDefUnits(), rect.yMax() / getDefUnits()));
    GLShape* rectShape = new GLRectShape(shapeToAdd);
    rectShape->setUserData((void*) fill);
    (void) cnv->addShape(rectShape, fill->getTechLayer()->getId());
  }
}

double OpenRoadIntf::getDefUnits()
{
  if (block_ == nullptr)
    return 1.0;
  return 1.0 * (block_->getDbUnitsPerMicron());
}

double OpenRoadIntf::getLefUnits()
{
  return 1.0 * (tech_->getDbUnitsPerMicron());
  // return block_->getDbUnitsPerMicron();
  // return 1.0;
}

void OpenRoadIntf::printMessageInPython(const std::string& msg)
{
  if (OpenRoadPythonIntf::get())
    OpenRoadPythonIntf::get()->printMessage(msg);
}

void OpenRoadIntf::displayMessageFromPython(const char* msg)
{
  if (OpenRoadPythonIntf::get())
    OpenRoadPythonIntf::get()->displayMessage(msg);
}

Tcl_Interp* OpenRoadIntf::getTclInterp()
{
  return _mOrTclInterp;
}

openRoadTclRes OpenRoadIntf::executeTclCommand(const std::string& cmdStr,
                                               std::string& result)
{
  int return_code = Tcl_Eval(_mOrTclInterp, cmdStr.c_str());
  auto resultPtr = Tcl_GetObjResult(_mOrTclInterp);

  bool retCode = return_code == 0 ? true : false;
  openRoadTclRes resObj;
  std::string retStr = "Success : ";
  if (!retCode)
    retStr = "Failed : ";

  resObj.tclRes = retStr + cmdStr;
  resObj.tclRetVal = retCode;

  return resObj;
}

int OpenRoadIntf::channelOutput(ClientData instanceData,
                                const char* buf,
                                int toWrite,
                                int* errorCodePtr)
{
  // Buffer up the output
  OpenRoadIntf* orIntf = (OpenRoadIntf*) instanceData;
  if (OpenRoadPythonIntf::get()) {
    std::string msg = buf;
    OpenRoadPythonIntf::get()->setTclEvalState(true, msg);
  }
  // widget->outputBuffer_.append(QString::fromLatin1(buf, toWrite).trimmed());
  return toWrite;
}

bool OpenRoadIntf::setupTcl()
{
  _mOrTclInterp = Tcl_CreateInterp();
  // std::cout << "Created Tcl Interpreter" << std::endl ;

  Tcl_Channel stdoutChannel = Tcl_CreateChannel(
      &stdoutChannelType, "stdout", (ClientData) this, TCL_WRITABLE);
  if (stdoutChannel) {
    Tcl_SetChannelOption(nullptr, stdoutChannel, "-translation", "lf");
    Tcl_SetChannelOption(nullptr, stdoutChannel, "-buffering", "none");
    Tcl_RegisterChannel(_mOrTclInterp,
                        stdoutChannel);  // per man page: some tcl bug
    Tcl_SetStdChannel(stdoutChannel, TCL_STDOUT);
  }
  // std::cout << "Done Setting Output Channel For Tcl Interpreter" << std::endl
  // ;
  return true;
}

void OpenRoadIntf::initOnce()
{
  // pauser_->setText("Running");
  // pauser_->setStyleSheet("background-color: red");
  ord::tclAppInit(_mOrTclInterp);
  // pauser_->setText("Idle");
  // pauser_->setStyleSheet("");

  // TODO: tclAppInit should return the status which we could
  // pass to updateOutput
  // std::cout << "Initialized OpenRoad tcl\n" ;
  if (OpenRoadPythonIntf::get()) {
    std::string msg = "Successfuly Initialized OpenRoad Tcl";
    OpenRoadPythonIntf::get()->setTclEvalState(true, msg);
  }
}

// static
void OpenRoadIntf::clearGlobals()
{
  // TBD
}
}  // namespace OpenRoadUI
