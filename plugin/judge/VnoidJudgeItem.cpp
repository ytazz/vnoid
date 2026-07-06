#include "VnoidJudgeItem.h"
#include "VnoidJudgeTargetDevice.h"
#include "VnoidJudgeTargetItem.h"
#include <cnoid/ItemManager>
#include <cnoid/ItemTreeView>
#include <cnoid/MenuManager>
#include <cnoid/FileDialog>
#include <cnoid/MainWindow>
#include <cnoid/ProjectManager>
#include <cnoid/SimulatorItem>
#include <cnoid/WorldItem>
#include <cnoid/BodyItem>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/BodyBoundingBox>
#include <cnoid/BoundingBox>
#include <cnoid/EigenArchive>
#include <cnoid/EigenUtil>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/MessageOut>
#include <cnoid/Format>
#include <cnoid/TimeBar>
#include <fstream>
#include <limits>
#include <algorithm>
#include <cmath>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace cnoid::vnoid;

namespace {

constexpr double PostStageDwellSec = 1.0;
constexpr double DefaultPowerAverageWindow = 0.01;

constexpr float DefaultTextHeight = 18.0f;

// Estimated advance widths (as a fraction of text height) used for
// pixel-level positioning of right-aligned or grid-aligned text.
// DejaVuSans is proportional; the value is a conservative upper bound.
// DejaVuSansMono is monospaced with advance/height about 0.6.
constexpr float SansAdvanceRatio = 0.60f;
constexpr float MonoAdvanceRatio = 0.65f;

const Vector3f DefaultTextColor(1.0f, 1.0f, 1.0f);
const Vector3f ColorRed   (1.0f, 0.2f, 0.2f);
const Vector3f ColorGray  (0.7f, 0.7f, 0.7f);

// Colors of the section bounding boxes and the HUD status labels
// corresponding to the display states
const Vector3f SectionColorIdle      (0.4f, 0.4f, 0.4f);
const Vector3f SectionColorPreStage  (0.3f, 0.6f, 1.0f);
const Vector3f SectionColorInProgress(1.0f, 1.0f, 0.0f);
const Vector3f SectionColorPostStage (1.0f, 0.6f, 0.2f);
const Vector3f SectionColorCleared   (0.2f, 1.0f, 0.2f);

constexpr double DefaultHudPanelTransparency = 0.4;
constexpr int HudUpdateIntervalMs = 500;

void setupHudBackground(SgOverlayPanel* panel, double transparency)
{
    if(!panel){
        return;
    }
    panel->setColorMode(SgOverlayPanel::RendererBackgroundColor);
    panel->setTransparency(static_cast<float>(transparency));
}

SgLineSet* createBBoxLineSet(const Vector3& bmin, const Vector3& bmax, SgMaterial* material, float lineWidth)
{
    auto lineSet = new SgLineSet;
    auto vertices = lineSet->getOrCreateVertices();
    vertices->resize(8);
    for(int k = 0; k < 8; ++k){
        vertices->at(k) <<
            static_cast<float>((k & 1) ? bmax.x() : bmin.x()),
            static_cast<float>((k & 2) ? bmax.y() : bmin.y()),
            static_cast<float>((k & 4) ? bmax.z() : bmin.z());
    }
    // 12 edges of the box
    static const int edges[12][2] = {
        {0,1},{2,3},{4,5},{6,7},   // X edges
        {0,2},{1,3},{4,6},{5,7},   // Y edges
        {0,4},{1,5},{2,6},{3,7}    // Z edges
    };
    for(auto& e : edges){
        lineSet->addLine(e[0], e[1]);
    }
    lineSet->setLineWidth(lineWidth);
    lineSet->setMaterial(material);
    return lineSet;
}

bool sectionsEqual(const vector<VnoidJudgeFieldSection>& a, const vector<VnoidJudgeFieldSection>& b)
{
    if(a.size() != b.size()){
        return false;
    }
    for(size_t i = 0; i < a.size(); ++i){
        const auto& sa = a[i];
        const auto& sb = b[i];
        if(sa.name != sb.name ||
           sa.startsNewRow != sb.startsNewRow ||
           sa.preMin != sb.preMin || sa.preMax != sb.preMax ||
           sa.sectionMin != sb.sectionMin || sa.sectionMax != sb.sectionMax ||
           sa.postMin != sb.postMin || sa.postMax != sb.postMax){
            return false;
        }
    }
    return true;
}

}


void VnoidJudgeItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<VnoidJudgeItem, SubSimulatorItem>(N_("VnoidJudgeItem"));
    im.addCreationPanel<VnoidJudgeItem>();

    ItemTreeView::customizeContextMenu<VnoidJudgeItem>(
        [](VnoidJudgeItem* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction){
            menuManager.setPath("/");
            menuManager.addItem(_("Save the judgement result to a file"))->sigTriggered().connect(
                [item]{ item->showDialogToSaveJudgementResult(); });
            menuManager.addSeparator();
            menuFunction.dispatchAs<Item>(item);
        });
}


VnoidJudgeItem::VnoidJudgeItem()
{
    setName("VnoidJudge");
    simulatorItem_ = nullptr;
    postFuncId_ = -1;
    targetBody_ = nullptr;
    targetDevice_ = nullptr;
    powerLimitW_ = VnoidJudgeTargetItem::DefaultPowerLimitW;
    massLowerLimit_ = VnoidJudgeTargetItem::DefaultMassLowerLimit;
    bboxUpperLimit_.setConstant(VnoidJudgeTargetItem::DefaultBBoxUpperLimit);
    totalMass_ = 0.0;
    massOk_ = false;
    powerLimitOverNotified_ = false;
    bboxLimitOverNotified_ = false;
    powerAverageWindow_ = DefaultPowerAverageWindow;

    hudTextSize_ = DefaultTextHeight;
    hudTextColor_ = DefaultTextColor;
    hudPanelTransparency_ = DefaultHudPanelTransparency;
    showSectionBBoxes_ = true;
    powerAndBBoxHudUpdatePending_ = false;
    sceneRoot_ = new SgGroup;
    initializePowerAndBBoxHudUpdateTimer();
    initializeHud();
    initializeSectionBBoxes();
}


VnoidJudgeItem::VnoidJudgeItem(const VnoidJudgeItem& org)
    : SubSimulatorItem(org),
      RenderableItem(),
      powerAverageWindow_(org.powerAverageWindow_)
{
    simulatorItem_ = nullptr;
    postFuncId_ = -1;
    targetBody_ = nullptr;
    targetDevice_ = nullptr;
    powerLimitW_ = VnoidJudgeTargetItem::DefaultPowerLimitW;
    massLowerLimit_ = VnoidJudgeTargetItem::DefaultMassLowerLimit;
    bboxUpperLimit_.setConstant(VnoidJudgeTargetItem::DefaultBBoxUpperLimit);
    totalMass_ = 0.0;
    massOk_ = false;
    powerLimitOverNotified_ = false;
    bboxLimitOverNotified_ = false;

    hudTextSize_ = org.hudTextSize_;
    hudTextColor_ = org.hudTextColor_;
    hudPanelTransparency_ = org.hudPanelTransparency_;
    showSectionBBoxes_ = org.showSectionBBoxes_;
    powerAndBBoxHudUpdatePending_ = false;
    sceneRoot_ = new SgGroup;
    initializePowerAndBBoxHudUpdateTimer();
    initializeHud();
    initializeSectionBBoxes();
}


VnoidJudgeItem::~VnoidJudgeItem()
{
    powerAndBBoxHudUpdateTimer_.stop();
}


void VnoidJudgeItem::setPowerAverageWindow(double t)
{
    powerAverageWindow_ = std::max(0.0, t);
    jointPowerSamples_.clear();
    jointPowerSums_.clear();
}


Item* VnoidJudgeItem::doDuplicate() const
{
    return new VnoidJudgeItem(*this);
}


bool VnoidJudgeItem::initializeSimulation(SimulatorItem* simulatorItem)
{
    simulatorItem_ = simulatorItem;
    targetBody_ = nullptr;
    targetDevice_ = nullptr;
    sectionBBs_.clear();
    sectionPhases_.clear();
    sectionPostEnterTime_.clear();
    powerLimitOverNotified_ = false;
    bboxLimitOverNotified_ = false;

    auto mo = MessageOut::master();

    SimulationBody* targetSimBody = nullptr;
    for(auto* simBody : simulatorItem->simulationBodies()){
        Body* body = simBody->body();
        if(auto dev = body->findDevice<VnoidJudgeTargetDevice>()){
            targetSimBody = simBody;
            targetBody_ = body;
            targetDevice_ = dev;
            break;
        }
    }

    if(!targetDevice_){
        mo->putWarningln(_("VnoidJudgeItem: No VnoidJudgeTargetDevice was found in the simulation bodies."));
        return false;
    }

    // The limit settings are defined by the target item attached to the
    // target body item
    powerLimitW_ = VnoidJudgeTargetItem::DefaultPowerLimitW;
    massLowerLimit_ = VnoidJudgeTargetItem::DefaultMassLowerLimit;
    bboxUpperLimit_.setConstant(VnoidJudgeTargetItem::DefaultBBoxUpperLimit);
    if(auto bodyItem = targetSimBody->bodyItem()){
        if(auto targetItem = bodyItem->findItem<VnoidJudgeTargetItem>()){
            powerLimitW_ = targetItem->powerLimitW();
            massLowerLimit_ = targetItem->massLowerLimit();
            bboxUpperLimit_ = targetItem->bboxUpperLimit();
        }
    }

    targetDevice_->clearState();
    resetPowerAverage();

    bodyBBox_ = std::make_unique<BodyBoundingBox>(targetBody_);

    totalMass_ = 0.0;
    for(int i = 0; i < targetBody_->numLinks(); ++i){
        totalMass_ += targetBody_->link(i)->mass();
    }
    massOk_ = (totalMass_ >= massLowerLimit_);
    mo->putHighlightedln(
        formatR(_("VnoidJudge: total mass = {0:.3f} kg ({1})"),
                totalMass_, massOk_ ? _("OK") : _("BELOW LIMIT")));

    collectSectionBBs(simulatorItem);
    sectionPhases_.assign(sectionBBs_.size(), Idle);
    sectionPostEnterTime_.assign(sectionBBs_.size(), -1.0);

    targetDevice_->notifyStateChange();

    postFuncId_ = simulatorItem->addPostDynamicsFunction(
        [this]{ onPostDynamics(); });

    return true;
}


void VnoidJudgeItem::finalizeSimulation()
{
    bodyBBox_.reset();
    if(simulatorItem_ && postFuncId_ >= 0){
        simulatorItem_->removePostDynamicsFunction(postFuncId_);
    }
    postFuncId_ = -1;

    simulatorItem_ = nullptr;
    targetBody_ = nullptr;
    targetDevice_ = nullptr;
    jointPowerSamples_.clear();
    jointPowerSums_.clear();
}


void VnoidJudgeItem::collectSectionBBs(SimulatorItem* simulatorItem)
{
    for(auto* simBody : simulatorItem->simulationBodies()){
        auto sections = readVnoidJudgeFieldSections(simBody->body());
        if(!sections.empty()){
            sectionBBs_ = std::move(sections);
            break;
        }
    }
    auto mo = MessageOut::master();
    if(sectionBBs_.size() > VnoidJudgeTargetDevice::MaxNumSections){
        mo->putWarningln(
            formatR(_("VnoidJudge: the field defines {0} sections, but only the first {1} sections are judged."),
                    sectionBBs_.size(), VnoidJudgeTargetDevice::MaxNumSections));
        sectionBBs_.resize(VnoidJudgeTargetDevice::MaxNumSections);
    }
    mo->putln(formatR(_("VnoidJudge: loaded {0} section bounding boxes."),
                      sectionBBs_.size()));
}


void VnoidJudgeItem::resetPowerAverage()
{
    jointPowerSamples_.clear();
    jointPowerSums_.clear();
    if(targetBody_){
        const int n = targetBody_->numJoints();
        jointPowerSamples_.resize(n);
        jointPowerSums_.assign(n, 0.0);
    }
}


double VnoidJudgeItem::calcAveragedPowerW(double time, int& out_jointId)
{
    double p = 0.0;
    out_jointId = -1;
    const int n = targetBody_->numJoints();

    if(static_cast<int>(jointPowerSamples_.size()) != n){
        resetPowerAverage();
    }

    if(powerAverageWindow_ <= 0.0){
        for(auto& joint : targetBody_->joints()){
            if(joint->jointId() < 0){
                continue;
            }
            const double jointPower = std::abs(joint->dq() * joint->u());
            if(jointPower > p){
                p = jointPower;
                out_jointId = joint->jointId();
            }
        }
        return p;
    }

    const double cutoff = time - powerAverageWindow_;
    for(int i = 0; i < n; ++i){
        auto* joint = targetBody_->joint(i);
        if(joint->jointId() < 0){
            continue;
        }

        auto& samples = jointPowerSamples_[i];
        double& sum = jointPowerSums_[i];
        const double samplePower = std::abs(joint->dq() * joint->u());

        samples.push_back({ time, samplePower });
        sum += samplePower;

        while(!samples.empty() && samples.front().time <= cutoff){
            sum -= samples.front().power;
            samples.pop_front();
        }

        if(!samples.empty()){
            const double averagedPower = sum / samples.size();
            if(averagedPower > p){
                p = averagedPower;
                out_jointId = joint->jointId();
            }
        }
    }
    return p;
}


bool VnoidJudgeItem::contains(
    const Vector3& outerMin, const Vector3& outerMax,
    const Vector3& innerMin, const Vector3& innerMax)
{
    for(int k = 0; k < 3; ++k){
        if(innerMin[k] < outerMin[k]) return false;
        if(innerMax[k] > outerMax[k]) return false;
    }
    return true;
}


bool VnoidJudgeItem::intersects(
    const Vector3& aMin, const Vector3& aMax,
    const Vector3& bMin, const Vector3& bMax)
{
    for(int k = 0; k < 3; ++k){
        if(bMax[k] < aMin[k]) return false;
        if(bMin[k] > aMax[k]) return false;
    }
    return true;
}


void VnoidJudgeItem::updateSection(int i, const Vector3& bbMin, const Vector3& bbMax, double t)
{
    const auto& s = sectionBBs_[i];
    auto& phase = sectionPhases_[i];

    if(phase == Done){
        return;
    }

    bool inPre  = contains(s.preMin,     s.preMax,     bbMin, bbMax);
    bool inSec  = contains(s.sectionMin, s.sectionMax, bbMin, bbMax);
    bool inPost = contains(s.postMin,    s.postMax,    bbMin, bbMax);

    if(phase != Idle){
        /*
           Invariant for keeping the ongoing attempt: the robot must be fully
           contained in the single stage region it intersects. Intersecting
           two or more regions is regarded as a transitional state of crossing
           a stage boundary and the containment check is exempted, because
           adjacent stage regions do not overlap and the robot cannot be fully
           contained in any of them while crossing the boundary.
        */
        int numIntersected =
            (intersects(s.preMin,     s.preMax,     bbMin, bbMax) ? 1 : 0) +
            (intersects(s.sectionMin, s.sectionMax, bbMin, bbMax) ? 1 : 0) +
            (intersects(s.postMin,    s.postMax,    bbMin, bbMax) ? 1 : 0);
        bool containedInAny = inPre || inSec || inPost;
        if(numIntersected == 0 || (numIntersected == 1 && !containedInAny)){
            phase = Idle;
            targetDevice_->setSectionPreStage(i, false);
            targetDevice_->setSectionInProgress(i, false);
            targetDevice_->setSectionPostStage(i, false);
            sectionPostEnterTime_[i] = -1.0;
        }
    }

    switch(phase){
    case Idle:
        if(inPre){
            phase = AtPreStage;
            targetDevice_->setSectionPreStage(i, true);
        }
        break;
    case AtPreStage:
        if(inSec){
            phase = InSection;
            targetDevice_->setSectionPreStage(i, false);
            targetDevice_->setSectionInProgress(i, true);
        }
        break;
    case InSection:
        if(inPost){
            phase = AtPostStage;
            targetDevice_->setSectionInProgress(i, false);
            targetDevice_->setSectionPostStage(i, true);
            sectionPostEnterTime_[i] = t;
        } else if(inPre){
            // The robot has moved back to the pre stage
            phase = AtPreStage;
            targetDevice_->setSectionInProgress(i, false);
            targetDevice_->setSectionPreStage(i, true);
        }
        break;
    case AtPostStage:
        if(!inPost){
            // The robot has moved back to the section
            phase = InSection;
            targetDevice_->setSectionPostStage(i, false);
            targetDevice_->setSectionInProgress(i, true);
            sectionPostEnterTime_[i] = -1.0;
        } else if(t - sectionPostEnterTime_[i] >= PostStageDwellSec){
            phase = Done;
            targetDevice_->setSectionPostStage(i, false);
            targetDevice_->setSectionCleared(i, true);
            targetDevice_->setSectionClearTime(i, t);
            MessageOut::master()->putHighlightedln(
                formatR(_("VnoidJudge: Section \"{0}\" has been cleared at {1:.2f} [s]."),
                        s.name, t));
        }
        break;
    default:
        break;
    }
}


void VnoidJudgeItem::onPostDynamics()
{
    if(!targetDevice_) return;

    const double t = simulatorItem_->currentTime();
    targetDevice_->setElapsedTime(t);

    // A single traversal produces both the root-frame BB (used for
    // rotation-invariant size reporting and the on-scene overlay) and the
    // world-frame BB (used for section containment).
    auto [rootBBox, worldBBox] = bodyBBox_->compute(
        BodyBoundingBox::RootLocalAABB, BodyBoundingBox::WorldAABB);

    Vector3 rootMin = rootBBox.empty() ? Vector3::Zero() : rootBBox.min();
    Vector3 rootMax = rootBBox.empty() ? Vector3::Zero() : rootBBox.max();
    Vector3 worldMin = worldBBox.empty() ? Vector3::Zero() : worldBBox.min();
    Vector3 worldMax = worldBBox.empty() ? Vector3::Zero() : worldBBox.max();

    targetDevice_->setBBoxRoot(rootMin.data(), rootMax.data());

    // Track the per-axis maximum extent seen so far in this run.
    Vector3 rootSz = rootMax - rootMin;
    const double* prevMax = targetDevice_->bboxMaxSize();
    targetDevice_->setBBoxMaxSize(
        std::max(prevMax[0], rootSz[0]),
        std::max(prevMax[1], rootSz[1]),
        std::max(prevMax[2], rootSz[2]));
    if(!bboxLimitOverNotified_){
        const double* bboxMaxSize = targetDevice_->bboxMaxSize();
        if(bboxMaxSize[0] > bboxUpperLimit_[0] ||
           bboxMaxSize[1] > bboxUpperLimit_[1] ||
           bboxMaxSize[2] > bboxUpperLimit_[2]){
            bboxLimitOverNotified_ = true;
            MessageOut::master()->putHighlightedln(
                formatR(_("VnoidJudge: bounding-box limit exceeded:\n"
                          "Max size = ({0:.3f}, {1:.3f}, {2:.3f}) [m], "
                          "Limit = ({3:.3f}, {4:.3f}, {5:.3f}) [m]"),
                        bboxMaxSize[0], bboxMaxSize[1], bboxMaxSize[2],
                        bboxUpperLimit_[0], bboxUpperLimit_[1], bboxUpperLimit_[2]));
        }
    }

    int powerJointId = -1;
    double p = calcAveragedPowerW(t, powerJointId);
    targetDevice_->setInstantPowerW(p);
    if(p > targetDevice_->maxPowerW()){
        targetDevice_->setMaxPowerW(p);
        targetDevice_->setMaxPowerJointId(powerJointId);
    }
    if(!powerLimitOverNotified_ && targetDevice_->maxPowerW() > powerLimitW_){
        powerLimitOverNotified_ = true;
        const int maxPowerJointId = targetDevice_->maxPowerJointId();
        const string jointIdLabel = maxPowerJointId >= 0 ? formatC("{}", maxPowerJointId) : "--";
        const string& jointName = targetBody_->joint(maxPowerJointId)->jointName();
        MessageOut::master()->putHighlightedln(
            formatR(_("VnoidJudge: POWER limit exceeded in {0} (ID {1}):\n"
                      "Max power = {2:.1f} [W], Limit = {3:.1f} [W]"),
                    jointName, jointIdLabel,
                    targetDevice_->maxPowerW(), powerLimitW_));
    }

    for(size_t i = 0; i < sectionBBs_.size(); ++i){
        updateSection(i, worldMin, worldMax, t);
    }

    targetDevice_->notifyStateChange();
}


void VnoidJudgeItem::showDialogToSaveJudgementResult()
{
    if(!guiDevice_ || !guiTargetBodyItem_.lock()){
        MessageOut::master()->putErrorln(
            _("VnoidJudge: The judgement result cannot be saved because the judgement target is not found."));
        return;
    }

    string projectName = ProjectManager::instance()->currentProjectName();
    string defaultFileName = (projectName.empty() ? "project" : projectName) + "-result.txt";

    FileDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(_("Save the judgement result"));
    dialog.setViewMode(QFileDialog::List);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setLabelText(QFileDialog::Accept, _("Save"));
    dialog.setLabelText(QFileDialog::Reject, _("Cancel"));
    dialog.setNameFilters(QStringList() << _("Text files (*.txt)") << _("Any files (*)"));
    dialog.updatePresetDirectories(true);
    dialog.selectFile(QString::fromStdString(defaultFileName));

    if(dialog.exec() == QDialog::Accepted){
        saveJudgementResult(dialog.selectedFiles().value(0).toStdString());
    }
}


bool VnoidJudgeItem::saveJudgementResult(const std::string& filename)
{
    auto mo = MessageOut::master();

    auto device = guiDevice_;
    auto bodyItem = guiTargetBodyItem_.lock();
    if(!device || !bodyItem){
        mo->putErrorln(
            _("VnoidJudge: The judgement result cannot be saved because the judgement target is not found."));
        return false;
    }
    auto body = bodyItem->body();

    ofstream ofs(filename);
    if(!ofs){
        mo->putErrorln(formatR(_("VnoidJudge: The result file \"{0}\" cannot be opened."), filename));
        return false;
    }

    double powerLimitW = VnoidJudgeTargetItem::DefaultPowerLimitW;
    double massLowerLimit = VnoidJudgeTargetItem::DefaultMassLowerLimit;
    Vector3 bboxUpperLimit;
    bboxUpperLimit.setConstant(VnoidJudgeTargetItem::DefaultBBoxUpperLimit);
    if(auto targetItem = guiTargetItem_.lock()){
        powerLimitW = targetItem->powerLimitW();
        massLowerLimit = targetItem->massLowerLimit();
        bboxUpperLimit = targetItem->bboxUpperLimit();
    }

    double totalMass = 0.0;
    for(int i = 0; i < body->numLinks(); ++i){
        totalMass += body->link(i)->mass();
    }
    bool massOk = (totalMass >= massLowerLimit);

    ofs << "Project: " << ProjectManager::instance()->currentProjectName() << "\n";
    ofs << "Target: " << body->name() << "\n";
    ofs << formatC("Elapsed time: {:.3f} s\n", device->elapsedTime());
    ofs << formatC("Total mass: {:.3f} kg ({})\n", totalMass, massOk ? "OK" : "BELOW LIMIT");
    ofs << "Power average window: " << powerAverageWindow_ << " s\n";

    string maxPowerJointInfo;
    const int maxPowerJointId = device->maxPowerJointId();
    if(maxPowerJointId >= 0){
        if(auto joint = body->joint(maxPowerJointId)){
            maxPowerJointInfo = formatC(" (joint {0}, ID {1})", joint->jointName(), maxPowerJointId);
        }
    }
    bool powerOk = (device->maxPowerW() <= powerLimitW);
    ofs << formatC("Max power: {:.1f} W{} ({})\n",
                   device->maxPowerW(), maxPowerJointInfo, powerOk ? "OK" : "EXCEEDED");

    const double* m = device->bboxMaxSize();
    ofs << formatC("Max bounding box: {:.3f} {:.3f} {:.3f}\n", m[0], m[1], m[2]);
    bool bboxOver =
        m[0] > bboxUpperLimit[0] ||
        m[1] > bboxUpperLimit[1] ||
        m[2] > bboxUpperLimit[2];
    ofs << "BBox: " << (bboxOver ? "EXCEEDED" : "OK") << "\n";

    // Only the cleared sections are listed in the order of the clear times
    vector<pair<double, int>> clearedSections; // (clear time, section index)
    for(size_t i = 0; i < guiSections_.size(); ++i){
        if(device->isSectionCleared(i)){
            clearedSections.emplace_back(device->sectionClearTime(i), i);
        }
    }
    if(clearedSections.empty()){
        ofs << "No section has been cleared.\n";
    } else {
        std::sort(clearedSections.begin(), clearedSections.end());
        for(auto& [clearTime, index] : clearedSections){
            ofs << formatC("Section {0}: CLEAR at {1:.3f} s\n",
                           guiSections_[index].name, clearTime);
        }
    }

    mo->putln(
        formatR(_("VnoidJudge: The judgement result has been saved to \"{0}\"."), filename));

    return true;
}


void VnoidJudgeItem::onTreePathChanged()
{
    updateGuiTargets();
}


void VnoidJudgeItem::updateGuiTargets()
{
    guiConnections_.disconnect();

    BodyItem* robotItem = nullptr;
    VnoidJudgeTargetDevicePtr device;
    BodyItem* fieldItem = nullptr;
    vector<VnoidJudgeFieldSection> sections;

    if(auto worldItem = findOwnerItem<WorldItem>()){
        guiConnections_.add(
            worldItem->sigSubTreeChanged().connect(
                [this]{ updateGuiTargets(); }));

        for(auto& bodyItem : worldItem->descendantItems<BodyItem>()){
            if(!device){
                if(auto dev = bodyItem->body()->findDevice<VnoidJudgeTargetDevice>()){
                    robotItem = bodyItem;
                    device = dev;
                    continue;
                }
            }
            if(!fieldItem){
                auto ss = readVnoidJudgeFieldSections(bodyItem->body());
                if(!ss.empty()){
                    fieldItem = bodyItem;
                    sections = std::move(ss);
                }
            }
        }
        if(robotItem){
            guiConnections_.add(
                robotItem->sigModelUpdated().connect(
                    [this](int){ updateGuiTargets(); }));
        }
        if(fieldItem){
            guiConnections_.add(
                fieldItem->sigModelUpdated().connect(
                    [this](int){ updateGuiTargets(); }));
        }
    }

    if(sections.size() > VnoidJudgeTargetDevice::MaxNumSections){
        sections.resize(VnoidJudgeTargetDevice::MaxNumSections);
    }

    guiTargetBodyItem_ = robotItem;
    guiTargetItem_ = robotItem ? robotItem->findItem<VnoidJudgeTargetItem>() : nullptr;
    guiFieldBodyItem_ = fieldItem;

    if(device != guiDevice_){
        guiDevice_ = device;
        deviceStateConnection_.disconnect();
        if(guiDevice_){
            deviceStateConnection_ = guiDevice_->sigStateChanged().connect(
                [this]{ onGuiDeviceStateChanged(); });
        }
    }

    if(!sectionsEqual(sections, guiSections_)){
        guiSections_ = std::move(sections);
        initializeHud();
        initializeSectionBBoxes();
    }

    updateHud();
    updateSectionBBoxColors(true);
    updateSceneVisibility();
}


void VnoidJudgeItem::onGuiDeviceStateChanged()
{
    updateImmediateHud();
    requestPowerAndBBoxHudUpdate();
    updateSectionBBoxColors(false);
}


SgNode* VnoidJudgeItem::getScene()
{
    updateSceneVisibility();
    if(!checkConnection_.connected()){
        checkConnection_ = sigCheckToggled().connect(
            [this](bool){ updateSceneVisibility(); });
    }
    return sceneRoot_;
}


void VnoidJudgeItem::updateSceneVisibility()
{
    const bool on = isChecked();

    auto setNodeVisible = [this](SgNode* node, bool visible){
        if(!node) return;
        const bool present = (sceneRoot_->findChildIndex(node) >= 0);
        if(visible && !present){
            sceneRoot_->addChild(node);
            sceneRoot_->notifyUpdate(SgUpdate::Added);
        } else if(!visible && present){
            sceneRoot_->removeChild(node);
            sceneRoot_->notifyUpdate(SgUpdate::Removed);
        }
    };

    setNodeVisible(hud_, on);
    setNodeVisible(sectionBBoxGroup_,
                   on && showSectionBBoxes_ && !sectionBBoxGroup_->empty());
}


void VnoidJudgeItem::initializePowerAndBBoxHudUpdateTimer()
{
    powerAndBBoxHudUpdateTimer_.setSingleShot(true);
    powerAndBBoxHudUpdateTimer_.sigTimeout().connect(
        [this]{
            if(powerAndBBoxHudUpdatePending_){
                powerAndBBoxHudUpdatePending_ = false;
                updatePowerAndBBoxHud();
                if(TimeBar::instance()->isDoingPlayback()){
                    powerAndBBoxHudUpdateTimer_.start(HudUpdateIntervalMs);
                }
            }
        });
}


void VnoidJudgeItem::requestPowerAndBBoxHudUpdate()
{
    if(!TimeBar::instance()->isDoingPlayback()){
        powerAndBBoxHudUpdateTimer_.stop();
        powerAndBBoxHudUpdatePending_ = false;
        updatePowerAndBBoxHud();
        return;
    }

    if(!powerAndBBoxHudUpdateTimer_.isActive()){
        updatePowerAndBBoxHud();
        powerAndBBoxHudUpdateTimer_.start(HudUpdateIntervalMs);
    } else {
        powerAndBBoxHudUpdatePending_ = true;
    }
}


void VnoidJudgeItem::initializeHud()
{
    const bool hudWasVisible = hud_ && (sceneRoot_->findChildIndex(hud_) >= 0);
    if(hudWasVisible){
        sceneRoot_->removeChild(hud_);
    }

    hud_ = new SgHudOverlay;

    const float textHeight = static_cast<float>(hudTextSize_);
    const float s = textHeight / DefaultTextHeight;

    // Approximate pixel widths.
    const int monoW = static_cast<int>(textHeight * MonoAdvanceRatio + 0.5f);
    const int sansW = static_cast<int>(textHeight * SansAdvanceRatio + 0.5f);

    // Margins and layout metrics (all scaled).
    const int marginX     = static_cast<int>(20 * s + 0.5f);
    const int rightMargin = static_cast<int>(30 * s + 0.5f);
    const int rowSpacing  = static_cast<int>(30 * s + 0.5f);
    const int bgPadX      = static_cast<int>(8 * s + 0.5f);
    const int bgPadY      = static_cast<int>(8 * s + 0.5f);

    // POWER label (Sans, "POWER [W]" = 9 chars) + numeric value slots (Mono).
    const int powerLabelW  = 9 * sansW;   // reserve for "POWER [W]"
    const int powerLabelYSpacing = static_cast<int>(textHeight + bgPadY + 0.5f);
    const int powerPanelBottomMargin = static_cast<int>(8 * s + 0.5f);

    // Bounding Box label (Sans) + numeric X/Y/Z rows (Mono).
    const int bboxLabelW    = 16 * sansW;   // "Bounding Box [m]"
    const int bboxRowsStart = powerLabelYSpacing + rowSpacing + powerPanelBottomMargin;

    const int powerLabelGap = 2 * monoW;
    const int powerCurrentValueW = 8 * monoW;
    const int powerValueGap = 2 * monoW;
    const int powerMaxValueW = 30 * monoW;   // Includes extra room for glyph margins.
    const int powerValueW = powerCurrentValueW + powerValueGap + powerMaxValueW;
    const int bboxValueW  = 26 * monoW;
    const int powerBgW = powerLabelW + powerLabelGap + powerValueW + bgPadX * 2;
    const int powerBgTop =
        max(0, powerLabelYSpacing - static_cast<int>(textHeight + 0.5f) - bgPadY);
    const int bboxBgTop =
        max(0, bboxRowsStart - static_cast<int>(textHeight + 0.5f) - bgPadY);
    const int powerBgBottom = bboxBgTop;
    setupHudBackground(
        hud_->addPanel(powerBgW, powerBgBottom - powerBgTop,
                       SgHudOverlay::TopLeft, max(0, marginX - bgPadX), powerBgTop),
        hudPanelTransparency_);

    powerLabelText_ = new SgText;
    powerLabelText_->setTextHeight(textHeight);
    powerLabelText_->setColor(hudTextColor_);
    powerLabelText_->setText("POWER [W]");
    hud_->addItem(powerLabelText_, SgHudOverlay::TopLeft, marginX, powerLabelYSpacing);

    powerValueText_ = new SgText;
    powerValueText_->setTextHeight(textHeight);
    powerValueText_->setFontType(SgText::MonoFont);
    powerValueText_->setColor(hudTextColor_);
    hud_->addItem(powerValueText_,
                  SgHudOverlay::TopLeft,
                  marginX + powerLabelW + powerLabelGap, powerLabelYSpacing);

    powerMaxValueText_ = new SgText;
    powerMaxValueText_->setTextHeight(textHeight);
    powerMaxValueText_->setFontType(SgText::MonoFont);
    powerMaxValueText_->setColor(hudTextColor_);
    hud_->addItem(powerMaxValueText_,
                  SgHudOverlay::TopLeft,
                  marginX + powerLabelW + powerLabelGap + powerCurrentValueW + powerValueGap,
                  powerLabelYSpacing);

    bboxLabelText_ = new SgText;
    bboxLabelText_->setTextHeight(textHeight);
    bboxLabelText_->setColor(hudTextColor_);
    bboxLabelText_->setText("Bounding Box [m]");

    const int bboxBgW = max(bboxLabelW, bboxValueW) + bgPadX * 2;
    const int bboxBgBottom = bboxRowsStart + rowSpacing * 3 + bgPadY;
    setupHudBackground(
        hud_->addPanel(bboxBgW, bboxBgBottom - bboxBgTop,
                       SgHudOverlay::TopLeft, max(0, marginX - bgPadX), bboxBgTop),
        hudPanelTransparency_);

    hud_->addItem(bboxLabelText_, SgHudOverlay::TopLeft, marginX, bboxRowsStart);

    for(int i = 0; i < 3; ++i){
        bboxAxisText_[i] = new SgText;
        bboxAxisText_[i]->setTextHeight(textHeight);
        bboxAxisText_[i]->setFontType(SgText::MonoFont);
        bboxAxisText_[i]->setColor(hudTextColor_);
        hud_->addItem(bboxAxisText_[i], SgHudOverlay::TopLeft,
                      marginX, bboxRowsStart + (i + 1) * rowSpacing);
    }

    // TIME label (Sans, "TIME") on the left of the clock, clock value (Mono, "HH:MM:SS")
    // on the right edge. Both anchored at TopRight so they scale together.
    const int timeLabelW = 4 * sansW;                 // "TIME"
    const int timeGap    = static_cast<int>(2.0f * monoW + 0.5f); // gap between "TIME" and clock
    const int clockValueW = 10 * monoW;
    const int timeBgW = timeLabelW + timeGap + clockValueW + bgPadX * 2;
    const int timeBgTop = powerBgTop;
    const int timeBgBottom = powerLabelYSpacing + bgPadY;
    setupHudBackground(
        hud_->addPanel(timeBgW, timeBgBottom - timeBgTop,
                       SgHudOverlay::TopRight, max(0, rightMargin - bgPadX), timeBgTop),
        hudPanelTransparency_);

    timeValueText_ = new SgText;
    timeValueText_->setTextHeight(textHeight);
    timeValueText_->setFontType(SgText::MonoFont);
    timeValueText_->setColor(hudTextColor_);
    hud_->addItem(timeValueText_, SgHudOverlay::TopRight,
                  rightMargin, powerLabelYSpacing, clockValueW);

    timeLabelText_ = new SgText;
    timeLabelText_->setTextHeight(textHeight);
    timeLabelText_->setColor(hudTextColor_);
    timeLabelText_->setText("TIME");
    hud_->addItem(timeLabelText_, SgHudOverlay::TopRight,
                  rightMargin + clockValueW + timeGap, powerLabelYSpacing, timeLabelW);

    // Section status grid at the bottom center. The labels are the section
    // names given by the field model, arranged in up to three columns.
    sectionTexts_.clear();
    const int numSections = static_cast<int>(guiSections_.size());
    if(numSections > 0){
        size_t maxNameLen = 0;
        for(auto& section : guiSections_){
            maxNameLen = max(maxNameLen, section.name.size());
        }
        // Reserve room for the longest status suffix " CLEAR 999.99s" plus an
        // inter-column gap. The suffix consists mainly of capital letters and
        // digits, which are wider than the average advance width, so the
        // reserved width is estimated with a larger character count.
        const int sectionColW = static_cast<int>(maxNameLen + 20) * sansW;

        // Assign a grid cell to each section. Rows are given by the explicit
        // "new_row" entries in the field's section list if any; otherwise the
        // sections are automatically wrapped into rows of three columns.
        bool hasExplicitRows = false;
        for(auto& section : guiSections_){
            if(section.startsNewRow){
                hasExplicitRows = true;
                break;
            }
        }
        std::vector<std::pair<int, int>> cells(numSections);
        int row = 0;
        int col = 0;
        int cols = 0;
        for(int i = 0; i < numSections; ++i){
            if(i > 0){
                if(guiSections_[i].startsNewRow || (!hasExplicitRows && col >= 3)){
                    ++row;
                    col = 0;
                }
            }
            cells[i] = { row, col++ };
            cols = max(cols, col);
        }
        const int rows = row + 1;

        const int sectionBgW = cols * sectionColW + bgPadX * 2;
        const int sectionBgBottom = max(0, static_cast<int>(30 * s + 0.5f) - bgPadY);
        const int sectionTopRowY = static_cast<int>((40 + (rows - 1) * 30) * s + 0.5f);
        const int sectionBgTop =
            static_cast<int>(sectionTopRowY + textHeight + bgPadY + 0.5f);
        setupHudBackground(
            hud_->addPanel(sectionBgW, sectionBgTop - sectionBgBottom,
                           SgHudOverlay::BottomCenter, 0, sectionBgTop),
            hudPanelTransparency_);

        for(int i = 0; i < numSections; ++i){
            auto text = new SgText;
            text->setTextHeight(textHeight);
            text->setColor(hudTextColor_);
            hud_->addItem(text, SgHudOverlay::BottomCenter,
                          bgPadX + cells[i].second * sectionColW,
                          static_cast<int>((40 + (rows - 1 - cells[i].first) * 30) * s + 0.5f),
                          sectionBgW);
            sectionTexts_.push_back(text);
        }
    }

    updateHud();

    if(hudWasVisible){
        sceneRoot_->addChild(hud_);
        sceneRoot_->notifyUpdate(SgUpdate::Added | SgUpdate::Removed);
    }
}


void VnoidJudgeItem::updateHud()
{
    powerAndBBoxHudUpdateTimer_.stop();
    powerAndBBoxHudUpdatePending_ = false;
    updatePowerAndBBoxHud();
    updateImmediateHud();
}


void VnoidJudgeItem::updatePowerAndBBoxHud()
{
    static const char axisChar[3] = { 'X', 'Y', 'Z' };

    if(!guiDevice_ || !guiTargetBodyItem_.lock()){
        powerLabelText_->setColor(ColorGray);
        powerLabelText_->notifyUpdate();
        powerValueText_->setText("    --- ");
        powerValueText_->setColor(ColorGray);
        powerValueText_->notifyUpdate();
        powerMaxValueText_->setText("(max:    ---, joint: --)");
        powerMaxValueText_->setColor(ColorGray);
        powerMaxValueText_->notifyUpdate();

        bboxLabelText_->setColor(ColorGray);
        bboxLabelText_->notifyUpdate();
        for(int i = 0; i < 3; ++i){
            bboxAxisText_[i]->setText(formatC("{}:  ---  (max:  ---)", axisChar[i]));
            bboxAxisText_[i]->setColor(ColorGray);
            bboxAxisText_[i]->notifyUpdate();
        }

        return;
    }

    const auto* d = guiDevice_.get();

    // The limit settings are given by the target item. The default values
    // are used if the item is not found.
    double powerLimitW = VnoidJudgeTargetItem::DefaultPowerLimitW;
    Vector3 bboxUpperLimit;
    bboxUpperLimit.setConstant(VnoidJudgeTargetItem::DefaultBBoxUpperLimit);
    if(auto targetItem = guiTargetItem_.lock()){
        powerLimitW = targetItem->powerLimitW();
        bboxUpperLimit = targetItem->bboxUpperLimit();
    }

    bool powerOver = d->maxPowerW() > powerLimitW;
    powerLabelText_->setColor(powerOver ? ColorRed : hudTextColor_);
    powerLabelText_->notifyUpdate();
    powerValueText_->setText(formatC("{:8.1f}", d->instantPowerW()));
    powerValueText_->setColor(powerOver ? ColorRed : hudTextColor_);
    powerValueText_->notifyUpdate();
    if(d->maxPowerJointId() >= 0){
        powerMaxValueText_->setText(formatC("(max: {:7.1f}, joint: {:2d})",
                                            d->maxPowerW(), d->maxPowerJointId()));
    } else {
        powerMaxValueText_->setText(formatC("(max: {:7.1f}, joint: --)",
                                            d->maxPowerW()));
    }
    powerMaxValueText_->setColor(powerOver ? ColorRed : hudTextColor_);
    powerMaxValueText_->notifyUpdate();

    const double* rmin = d->bboxMinRoot();
    const double* rmax = d->bboxMaxRoot();
    const double sz[3] = {
        rmax[0] - rmin[0], rmax[1] - rmin[1], rmax[2] - rmin[2]
    };
    const double* msz = d->bboxMaxSize();
    bool bboxOver = false;
    for(int i = 0; i < 3; ++i){
        if(msz[i] > bboxUpperLimit[i]){
            bboxOver = true;
        }
    }
    bboxLabelText_->setColor(bboxOver ? ColorRed : hudTextColor_);
    bboxLabelText_->notifyUpdate();
    for(int i = 0; i < 3; ++i){
        bboxAxisText_[i]->setText(
            formatC("{}: {:5.2f} (max: {:5.2f})", axisChar[i], sz[i], msz[i]));
        bboxAxisText_[i]->setColor(bboxOver ? ColorRed : hudTextColor_);
        bboxAxisText_[i]->notifyUpdate();
    }
}


void VnoidJudgeItem::updateImmediateHud()
{
    const auto* d = (guiDevice_ && guiTargetBodyItem_.lock()) ? guiDevice_.get() : nullptr;

    if(!d){
        timeLabelText_->setColor(ColorGray);
        timeLabelText_->notifyUpdate();
        timeValueText_->setText("--:--.--");
        timeValueText_->setColor(ColorGray);
        timeValueText_->notifyUpdate();
    } else {
        // TIME shows minutes:seconds.hundredths, saturated at 99:59.99.
        double t = d->elapsedTime();
        if(t < 0.0) t = 0.0;
        int totalHundredths = static_cast<int>(t * 100.0 + 0.5);
        const int cap = 99 * 60 * 100 + 59 * 100 + 99;
        if(totalHundredths > cap) totalHundredths = cap;
        int mm = totalHundredths / 6000;
        int ss = (totalHundredths / 100) % 60;
        int cc = totalHundredths % 100;
        timeLabelText_->setColor(hudTextColor_);
        timeLabelText_->notifyUpdate();
        timeValueText_->setText(formatC("{:02}:{:02}.{:02}", mm, ss, cc));
        timeValueText_->setColor(hudTextColor_);
        timeValueText_->notifyUpdate();
    }

    for(size_t i = 0; i < sectionTexts_.size(); ++i){
        const string& name = guiSections_[i].name;
        auto& text = sectionTexts_[i];
        if(d && d->isSectionCleared(i)){
            text->setText(formatC("{}: CLEAR {:.2f}s", name, d->sectionClearTime(i)));
            text->setColor(SectionColorCleared);
        } else if(d && d->isSectionPostStage(i)){
            text->setText(formatC("{}: POST STAGE", name));
            text->setColor(SectionColorPostStage);
        } else if(d && d->isSectionInProgress(i)){
            text->setText(formatC("{}: IN SECTION", name));
            text->setColor(SectionColorInProgress);
        } else if(d && d->isSectionPreStage(i)){
            text->setText(formatC("{}: PRE STAGE", name));
            text->setColor(SectionColorPreStage);
        } else {
            text->setText(formatC("{}:", name));
            text->setColor(hudTextColor_);
        }
        text->notifyUpdate();
    }
}


void VnoidJudgeItem::initializeSectionBBoxes()
{
    const bool wasVisible =
        sectionBBoxGroup_ && (sceneRoot_->findChildIndex(sectionBBoxGroup_) >= 0);
    if(wasVisible){
        sceneRoot_->removeChild(sectionBBoxGroup_);
        sceneRoot_->notifyUpdate(SgUpdate::Removed);
    }

    sectionBBoxGroup_ = new SgGroup;
    sectionBBoxGroup_->setName("VnoidJudgeSectionBBoxes");
    sectionMaterials_.clear();
    sectionColorStates_.clear();

    for(auto& section : guiSections_){
        auto material = new SgMaterial;
        sectionMaterials_.push_back(material);
        sectionColorStates_.push_back(-1);
        sectionBBoxGroup_->addChild(
            createBBoxLineSet(section.preMin, section.preMax, material, 1.0f));
        sectionBBoxGroup_->addChild(
            createBBoxLineSet(section.sectionMin, section.sectionMax, material, 2.0f));
        sectionBBoxGroup_->addChild(
            createBBoxLineSet(section.postMin, section.postMax, material, 1.0f));
    }

    updateSectionBBoxColors(true);
    updateSceneVisibility();
}


int VnoidJudgeItem::sectionDisplayState(int index) const
{
    if(guiDevice_ && guiTargetBodyItem_.lock()){
        if(guiDevice_->isSectionCleared(index)){
            return DisplayCleared;
        } else if(guiDevice_->isSectionPostStage(index)){
            return DisplayPostStage;
        } else if(guiDevice_->isSectionInProgress(index)){
            return DisplayInProgress;
        } else if(guiDevice_->isSectionPreStage(index)){
            return DisplayPreStage;
        }
    }
    return DisplayIdle;
}


void VnoidJudgeItem::updateSectionBBoxColors(bool forceUpdate)
{
    for(size_t i = 0; i < sectionMaterials_.size(); ++i){
        const int state = sectionDisplayState(i);
        if(!forceUpdate && state == sectionColorStates_[i]){
            continue;
        }
        sectionColorStates_[i] = state;
        const Vector3f* color = &SectionColorIdle;
        switch(state){
        case DisplayPreStage:   color = &SectionColorPreStage;   break;
        case DisplayInProgress: color = &SectionColorInProgress; break;
        case DisplayPostStage:  color = &SectionColorPostStage;  break;
        case DisplayCleared:    color = &SectionColorCleared;    break;
        default: break;
        }
        auto& material = sectionMaterials_[i];
        material->setDiffuseColor(*color);
        material->setEmissiveColor(*color);
        material->notifyUpdate();
    }
}


void VnoidJudgeItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SubSimulatorItem::doPutProperties(putProperty);
    putProperty(_("Power average window [s]"), powerAverageWindow_,
                [this](double v){ setPowerAverageWindow(v); return true; });
    putProperty(_("Show section bounding boxes"), showSectionBBoxes_,
                [this](bool on){
                    showSectionBBoxes_ = on;
                    updateSceneVisibility();
                    return true;
                });
    putProperty(_("Font size"), hudTextSize_,
                [this](double v){
                    if(v <= 0.0) return false;
                    hudTextSize_ = v;
                    initializeHud();
                    return true;
                });
    putProperty(_("Text color"), str(hudTextColor_),
                [this](const string& value){
                    Vector3f color;
                    if(toVector3(value, color)){
                        hudTextColor_ = color;
                        updateHud();
                        return true;
                    }
                    return false;
                });
    putProperty.range(0.0, 1.0).decimals(2);
    putProperty(_("Panel transparency"), hudPanelTransparency_,
                [this](double v){
                    if(v < 0.0 || v > 1.0) return false;
                    hudPanelTransparency_ = v;
                    initializeHud();
                    return true;
                });
    putProperty.reset();
}


bool VnoidJudgeItem::store(Archive& archive)
{
    if(!SubSimulatorItem::store(archive)) return false;
    archive.write("power_average_window", powerAverageWindow_);
    archive.write("show_section_bounding_boxes", showSectionBBoxes_);
    archive.write("hud_font_size", hudTextSize_);
    write(archive, "hud_text_color", hudTextColor_);
    archive.write("hud_panel_transparency", hudPanelTransparency_);
    return true;
}


bool VnoidJudgeItem::restore(const Archive& archive)
{
    if(!SubSimulatorItem::restore(archive)) return false;
    double powerAverageWindow = powerAverageWindow_;
    if(archive.read("power_average_window", powerAverageWindow)){
        setPowerAverageWindow(powerAverageWindow);
    }
    archive.read("show_section_bounding_boxes", showSectionBBoxes_);

    bool needsHudUpdate = false;
    double v = hudTextSize_;
    if(archive.read("hud_font_size", v) && v > 0.0){
        hudTextSize_ = v;
        needsHudUpdate = true;
    }
    Vector3f color;
    if(read(archive, "hud_text_color", color)){
        hudTextColor_ = color;
        needsHudUpdate = true;
    }
    v = hudPanelTransparency_;
    if(archive.read("hud_panel_transparency", v) && v >= 0.0 && v <= 1.0){
        hudPanelTransparency_ = v;
        needsHudUpdate = true;
    }
    if(needsHudUpdate){
        initializeHud();
    }
    return true;
}
