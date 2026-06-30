#ifndef CNOID_VNOID_JUDGE_ITEM_H
#define CNOID_VNOID_JUDGE_ITEM_H

#include <cnoid/SubSimulatorItem>
#include <cnoid/RenderableItem>
#include <cnoid/SceneOverlay>
#include <cnoid/SceneDrawables>
#include <cnoid/ConnectionSet>
#include <cnoid/Timer>
#include <cnoid/EigenTypes>
#include "VnoidJudgeFieldInfo.h"
#include <vector>
#include <deque>
#include <memory>
#include <string>

namespace cnoid {

class Body;
class BodyItem;
class BodyBoundingBox;
class SimulatorItem;
class MessageOut;

namespace vnoid {

class VnoidJudgeTargetDevice;
typedef ref_ptr<VnoidJudgeTargetDevice> VnoidJudgeTargetDevicePtr;
class VnoidJudgeTargetItem;

/**
   The judge of the competition. This item performs the judgement during
   simulation and displays the judgement status on the scene view.

   As a sub simulator item, the item finds the target robot in the
   simulation bodies by the VnoidJudgeTargetDevice attached by
   VnoidJudgeTargetItem, and judges the robot every simulation step: the
   joint power and the bounding-box size are checked against the limits
   given by VnoidJudgeTargetItem, and the section clear judgement is
   processed based on the section definitions (hvac_field_sections) given
   by the field body. The judgement results are written into the device
   state so that they are recorded and played back frame by frame.

   As a renderable item, the item displays the HUD (power, bounding box,
   time, and section status) and the section bounding boxes colored by the
   section states. For this purpose the item tracks the GUI-side robot and
   field bodies in the world item and observes sigStateChanged() of the
   GUI-side device, so the display works both in live simulation and in
   playback. The section labels shown on the HUD are the section names
   defined by the field model; the item does not assume any particular
   section structure.
*/
class VnoidJudgeItem : public SubSimulatorItem, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    VnoidJudgeItem();
    VnoidJudgeItem(const VnoidJudgeItem& org);
    virtual ~VnoidJudgeItem();

    virtual bool initializeSimulation(SimulatorItem* simulatorItem) override;
    virtual void finalizeSimulation() override;

    virtual SgNode* getScene() override;

    void setPowerAverageWindow(double t);
    double powerAverageWindow() const { return powerAverageWindow_; }

    /**
       Save the judgement result at the current playback state to a file
       selected with a file dialog. This is invoked from the context menu
       of this item. Reproducing the desired state (usually the final state
       of the simulation) with the playback function is up to the user.
    */
    void showDialogToSaveJudgementResult();

    /**
       Save the judgement result at the current playback state to the file
       specified by the filename. This function can be used to automate the
       result saving with a script.
    */
    bool saveJudgementResult(const std::string& filename);

protected:
    virtual Item* doDuplicate() const override;
    virtual void onTreePathChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    enum SectionPhase {
        Idle, AtPreStage, InSection, AtPostStage, Done
    };

    enum SectionDisplayState {
        DisplayIdle, DisplayPreStage, DisplayInProgress, DisplayPostStage, DisplayCleared
    };

    struct PowerSample {
        double time;
        double power;
    };

    // Simulation-side functions
    void onPostDynamics();
    double calcAveragedPowerW(double time, int& out_jointId);
    void resetPowerAverage();
    void collectSectionBBs(SimulatorItem* simulatorItem);
    void updateSection(int i, const Vector3& bbMin, const Vector3& bbMax, double t);
    static bool contains(const Vector3& outerMin, const Vector3& outerMax,
                         const Vector3& innerMin, const Vector3& innerMax);
    static bool intersects(const Vector3& aMin, const Vector3& aMax,
                           const Vector3& bMin, const Vector3& bMax);

    // GUI-side functions
    void updateGuiTargets();
    void onGuiDeviceStateChanged();
    void initializeHud();
    void initializePowerAndBBoxHudUpdateTimer();
    void requestPowerAndBBoxHudUpdate();
    void updateHud();
    void updatePowerAndBBoxHud();
    void updateImmediateHud();
    void initializeSectionBBoxes();
    int sectionDisplayState(int index) const;
    void updateSectionBBoxColors(bool forceUpdate);
    void updateSceneVisibility();

    // Simulation-side members
    SimulatorItem* simulatorItem_;
    int postFuncId_;

    Body* targetBody_;
    VnoidJudgeTargetDevice* targetDevice_;

    std::unique_ptr<BodyBoundingBox> bodyBBox_;

    std::vector<VnoidJudgeFieldSection> sectionBBs_;
    std::vector<SectionPhase> sectionPhases_;
    std::vector<double> sectionPostEnterTime_;

    // Limit settings read from the target item at initializeSimulation
    double powerLimitW_;
    double massLowerLimit_;
    Vector3 bboxUpperLimit_;

    // One-shot status recorded at initializeSimulation. Not part of the
    // device state because it never changes during the run.
    double totalMass_;
    bool massOk_;
    bool powerLimitOverNotified_;
    bool bboxLimitOverNotified_;

    double powerAverageWindow_;
    std::vector<std::deque<PowerSample>> jointPowerSamples_;
    std::vector<double> jointPowerSums_;

    // GUI-side members
    weak_ref_ptr<BodyItem> guiTargetBodyItem_;
    weak_ref_ptr<VnoidJudgeTargetItem> guiTargetItem_;
    VnoidJudgeTargetDevicePtr guiDevice_;
    weak_ref_ptr<BodyItem> guiFieldBodyItem_;
    std::vector<VnoidJudgeFieldSection> guiSections_;

    double hudTextSize_;
    Vector3f hudTextColor_;
    double hudPanelTransparency_;
    bool showSectionBBoxes_;

    SgGroupPtr sceneRoot_;
    SgHudOverlayPtr hud_;
    SgTextPtr powerLabelText_;
    SgTextPtr powerValueText_;
    SgTextPtr powerMaxValueText_;
    SgTextPtr bboxLabelText_;
    SgTextPtr bboxAxisText_[3];
    SgTextPtr timeLabelText_;
    SgTextPtr timeValueText_;
    std::vector<SgTextPtr> sectionTexts_;

    SgGroupPtr sectionBBoxGroup_;
    std::vector<SgMaterialPtr> sectionMaterials_;
    std::vector<int> sectionColorStates_;

    Timer powerAndBBoxHudUpdateTimer_;
    bool powerAndBBoxHudUpdatePending_;

    ScopedConnectionSet guiConnections_;
    ScopedConnection deviceStateConnection_;
    ScopedConnection checkConnection_;
};

typedef ref_ptr<VnoidJudgeItem> VnoidJudgeItemPtr;

}
}

#endif
