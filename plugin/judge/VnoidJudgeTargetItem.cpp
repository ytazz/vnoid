#include "VnoidJudgeTargetItem.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace cnoid::vnoid;


void VnoidJudgeTargetItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<VnoidJudgeTargetItem>(N_("VnoidJudgeTargetItem"));
    im.addCreationPanel<VnoidJudgeTargetItem>();
}


VnoidJudgeTargetItem::VnoidJudgeTargetItem()
{
    setName("JudgeTarget");
    powerLimitW_ = DefaultPowerLimitW;
    massLowerLimit_ = DefaultMassLowerLimit;
    bboxUpperLimit_.setConstant(DefaultBBoxUpperLimit);
    initializeDevice();
}


VnoidJudgeTargetItem::VnoidJudgeTargetItem(const VnoidJudgeTargetItem& org)
    : Item(org)
{
    powerLimitW_ = org.powerLimitW_;
    massLowerLimit_ = org.massLowerLimit_;
    bboxUpperLimit_ = org.bboxUpperLimit_;
    initializeDevice();
}


VnoidJudgeTargetItem::~VnoidJudgeTargetItem()
{
    detachFromBodyItem();
}


Item* VnoidJudgeTargetItem::doDuplicate() const
{
    return new VnoidJudgeTargetItem(*this);
}


void VnoidJudgeTargetItem::initializeDevice()
{
    if(!device_){
        device_ = new VnoidJudgeTargetDevice;
        device_->setName("JudgeTarget");
    }
}


void VnoidJudgeTargetItem::onTreePathChanged()
{
    auto newBodyItem = findOwnerItem<BodyItem>();
    if(newBodyItem == targetBodyItem_.lock()){
        return;
    }
    detachFromBodyItem();
    if(newBodyItem){
        attachToBodyItem(newBodyItem);
    }
}


void VnoidJudgeTargetItem::attachToBodyItem(BodyItem* bodyItem)
{
    targetBodyItem_ = bodyItem;
    auto body = bodyItem->body();
    if(!body || !body->rootLink()) return;

    initializeDevice();

    // If a device already exists (e.g., another target item is also attached),
    // reuse it instead of adding a duplicate.
    if(auto existing = body->findDevice<VnoidJudgeTargetDevice>()){
        device_ = existing;
    } else {
        body->addDevice(device_, body->rootLink());
        bodyItem->notifyModelUpdate(BodyItem::DeviceSetUpdate);
    }
}


void VnoidJudgeTargetItem::detachFromBodyItem()
{
    if(auto bodyItem = targetBodyItem_.lock()){
        if(device_){
            auto body = bodyItem->body();
            if(body && body->removeDevice(device_)){
                bodyItem->notifyModelUpdate(BodyItem::DeviceSetUpdate);
            }
        }
    }
    targetBodyItem_.reset();
}


void VnoidJudgeTargetItem::doPutProperties(PutPropertyFunction& putProperty)
{
    Item::doPutProperties(putProperty);
    putProperty(_("Power limit [W]"), powerLimitW_,
                changeProperty(powerLimitW_));
    putProperty(_("Mass lower limit"), massLowerLimit_,
                changeProperty(massLowerLimit_));
    putProperty(_("BBox upper limit X"), bboxUpperLimit_.x(),
                [this](double v){ bboxUpperLimit_.x() = v; return true; });
    putProperty(_("BBox upper limit Y"), bboxUpperLimit_.y(),
                [this](double v){ bboxUpperLimit_.y() = v; return true; });
    putProperty(_("BBox upper limit Z"), bboxUpperLimit_.z(),
                [this](double v){ bboxUpperLimit_.z() = v; return true; });
}


bool VnoidJudgeTargetItem::store(Archive& archive)
{
    if(!Item::store(archive)) return false;
    archive.write("power_limit_w", powerLimitW_);
    archive.write("mass_lower_limit", massLowerLimit_);
    write(archive, "bbox_upper_limit", bboxUpperLimit_);
    return true;
}


bool VnoidJudgeTargetItem::restore(const Archive& archive)
{
    if(!Item::restore(archive)) return false;
    archive.read("power_limit_w", powerLimitW_);
    archive.read("mass_lower_limit", massLowerLimit_);
    read(archive, "bbox_upper_limit", bboxUpperLimit_);
    return true;
}
