#include "VnoidJudgeTargetDevice.h"

using namespace std;
using namespace cnoid;
using namespace cnoid::vnoid;


VnoidJudgeTargetDevice::VnoidJudgeTargetDevice()
{
    clearState();
}


VnoidJudgeTargetDevice::VnoidJudgeTargetDevice(const VnoidJudgeTargetDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
}


const char* VnoidJudgeTargetDevice::typeName() const
{
    return "VnoidJudgeTargetDevice";
}


bool VnoidJudgeTargetDevice::copyFrom(const Device* other)
{
    if(auto otherTarget = dynamic_cast<const VnoidJudgeTargetDevice*>(other)){
        Device::copyFrom(other);
        copyStateFrom(*otherTarget);
        return true;
    }
    return false;
}


void VnoidJudgeTargetDevice::copyStateFrom(const VnoidJudgeTargetDevice& other)
{
    instantPowerW_ = other.instantPowerW_;
    maxPowerW_     = other.maxPowerW_;
    maxPowerJointId_ = other.maxPowerJointId_;
    for(int i = 0; i < 3; ++i){
        bboxMinRoot_[i] = other.bboxMinRoot_[i];
        bboxMaxRoot_[i] = other.bboxMaxRoot_[i];
        bboxMaxSize_[i] = other.bboxMaxSize_[i];
    }
    elapsedTime_ = other.elapsedTime_;
    flags_       = other.flags_;
    for(int i = 0; i < MaxNumSections; ++i){
        sectionClearTime_[i] = other.sectionClearTime_[i];
    }
}


void VnoidJudgeTargetDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(VnoidJudgeTargetDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const VnoidJudgeTargetDevice&>(other));
}


#if CNOID_INTERNAL_VERSION >= 8
DeviceState* VnoidJudgeTargetDevice::cloneState
(DeviceState* /* existingClone */, std::vector<std::function<void()>>* /* completionFunctions */) const
#else
DeviceState* VnoidJudgeTargetDevice::cloneState(DeviceState* /* existingClone */) const
#endif
{
    return new VnoidJudgeTargetDevice(*this, true);
}


Referenced* VnoidJudgeTargetDevice::doClone(CloneMap*) const
{
    return new VnoidJudgeTargetDevice(*this);
}


void VnoidJudgeTargetDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(VnoidJudgeTargetDevice))){
        Device::forEachActualType(func);
    }
}


void VnoidJudgeTargetDevice::clearState()
{
    instantPowerW_ = 0.0;
    maxPowerW_     = 0.0;
    maxPowerJointId_ = -1;
    for(int i = 0; i < 3; ++i){
        bboxMinRoot_[i] = 0.0;
        bboxMaxRoot_[i] = 0.0;
        bboxMaxSize_[i] = 0.0;
    }
    elapsedTime_ = 0.0;
    flags_       = 0;
    for(int i = 0; i < MaxNumSections; ++i){
        sectionClearTime_[i] = -1.0;
    }
}


int VnoidJudgeTargetDevice::stateSize() const
{
    return 3 + 3 + 3 + 3 + 1 + 1 + MaxNumSections;
}


const double* VnoidJudgeTargetDevice::readState(const double* buf, int /* size */)
{
    int i = 0;
    instantPowerW_  = buf[i++];
    maxPowerW_      = buf[i++];
    maxPowerJointId_ = static_cast<int>(buf[i++]);
    for(int k = 0; k < 3; ++k) bboxMinRoot_[k] = buf[i++];
    for(int k = 0; k < 3; ++k) bboxMaxRoot_[k] = buf[i++];
    for(int k = 0; k < 3; ++k) bboxMaxSize_[k] = buf[i++];
    elapsedTime_    = buf[i++];
    flags_          = static_cast<uint64_t>(buf[i++]);
    for(int k = 0; k < MaxNumSections; ++k){
        sectionClearTime_[k] = buf[i++];
    }
    return buf + i;
}


double* VnoidJudgeTargetDevice::writeState(double* out_buf) const
{
    int i = 0;
    out_buf[i++] = instantPowerW_;
    out_buf[i++] = maxPowerW_;
    out_buf[i++] = static_cast<double>(maxPowerJointId_);
    for(int k = 0; k < 3; ++k) out_buf[i++] = bboxMinRoot_[k];
    for(int k = 0; k < 3; ++k) out_buf[i++] = bboxMaxRoot_[k];
    for(int k = 0; k < 3; ++k) out_buf[i++] = bboxMaxSize_[k];
    out_buf[i++] = elapsedTime_;
    out_buf[i++] = static_cast<double>(flags_);
    for(int k = 0; k < MaxNumSections; ++k){
        out_buf[i++] = sectionClearTime_[k];
    }
    return out_buf + i;
}
