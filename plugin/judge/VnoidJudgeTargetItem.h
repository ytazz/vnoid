#ifndef CNOID_VNOID_JUDGE_TARGET_ITEM_H
#define CNOID_VNOID_JUDGE_TARGET_ITEM_H

#include <cnoid/Item>
#include <cnoid/EigenTypes>
#include "VnoidJudgeTargetDevice.h"

namespace cnoid {

class BodyItem;
class ExtensionManager;

namespace vnoid {

/**
   An item that marks the owner body as the judgement target.
   The item attaches a VnoidJudgeTargetDevice to the body and holds
   the limit settings of the judgement. The judgement itself and the
   HUD display are handled by VnoidJudgeItem.
*/
class VnoidJudgeTargetItem : public Item
{
public:
    static constexpr double DefaultPowerLimitW = 1000.0;
    static constexpr double DefaultMassLowerLimit = 30.0;
    static constexpr double DefaultBBoxUpperLimit = 2.0;

    static void initializeClass(ExtensionManager* ext);

    VnoidJudgeTargetItem();
    VnoidJudgeTargetItem(const VnoidJudgeTargetItem& org);
    virtual ~VnoidJudgeTargetItem();

    double powerLimitW() const { return powerLimitW_; }
    void setPowerLimitW(double v) { powerLimitW_ = v; }
    double massLowerLimit() const { return massLowerLimit_; }
    void setMassLowerLimit(double v) { massLowerLimit_ = v; }
    const Vector3& bboxUpperLimit() const { return bboxUpperLimit_; }
    void setBBoxUpperLimit(const Vector3& v) { bboxUpperLimit_ = v; }

protected:
    virtual Item* doDuplicate() const override;
    virtual void onTreePathChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    void initializeDevice();
    void attachToBodyItem(BodyItem* bodyItem);
    void detachFromBodyItem();

    weak_ref_ptr<BodyItem> targetBodyItem_;
    VnoidJudgeTargetDevicePtr device_;
    double powerLimitW_;
    double massLowerLimit_;
    Vector3 bboxUpperLimit_;
};

typedef ref_ptr<VnoidJudgeTargetItem> VnoidJudgeTargetItemPtr;

}
}

#endif
