#ifndef CNOID_VNOID_JUDGE_TARGET_DEVICE_H
#define CNOID_VNOID_JUDGE_TARGET_DEVICE_H

#include <cnoid/Device>
#include <cstdint>

namespace cnoid {
namespace vnoid {

/**
   A device that holds the time-varying judgement state of the target robot:
   the joint power, the bounding box, the elapsed time, and the section
   states (pre stage / in section / post stage / cleared) with the clear
   times. The state is recorded and played back frame by frame through the
   standard device state mechanism (readState / writeState).

   The device is not defined in the robot model file. VnoidJudgeTargetItem
   attaches this device to the root link of its owner body to mark the body
   as the judgement target. The static judgement parameters such as the
   power and bounding-box limits are competition rules rather than robot
   properties, and are held by VnoidJudgeTargetItem, not by this device.
   This keeps the device a pure state container so that the state clones
   stored in the simulation log do not carry static data.

   During simulation, VnoidJudgeItem finds the body with this device,
   performs the judgement, and writes the results into the device state
   every step. On the GUI side, VnoidJudgeItem observes sigStateChanged()
   of this device to update the HUD and the section bounding-box
   visualization, both in live simulation and in playback.
*/
class VnoidJudgeTargetDevice : public Device
{
public:
    /*
       Capacity of the section state slots. The actual number of sections
       is defined by the field model and must not exceed this value.
       Note that the state size increases with this value because the
       section clear times are also part of the state.
    */
    static const int MaxNumSections = 8;

    /*
       The first bit of each flag group within flags_. The flags are stored
       in a single double slot of the device state, which can represent
       integers exactly up to 2^53, so up to 13 sections (4 x 13 = 52 bits)
       can be handled without loss.
    */
    static constexpr uint64_t SectionCleared0    = 1ULL << 0;
    static constexpr uint64_t SectionInProgress0 = 1ULL << MaxNumSections;
    static constexpr uint64_t SectionPreStage0   = 1ULL << (2 * MaxNumSections);
    static constexpr uint64_t SectionPostStage0  = 1ULL << (3 * MaxNumSections);

    static_assert(4 * MaxNumSections <= 52,
                  "The section state flags must fit in the exact integer range of a double");

    VnoidJudgeTargetDevice();
    VnoidJudgeTargetDevice(const VnoidJudgeTargetDevice& org, bool copyStateOnly = false);

    virtual const char* typeName() const override;
    virtual bool copyFrom(const Device* other) override;
    void copyStateFrom(const VnoidJudgeTargetDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState(DeviceState* existingClone = nullptr) const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf, int size) override;
    virtual double* writeState(double* out_buf) const override;
    virtual void clearState() override;

    double instantPowerW() const { return instantPowerW_; }
    void setInstantPowerW(double v) { instantPowerW_ = v; }
    double maxPowerW() const { return maxPowerW_; }
    void setMaxPowerW(double v) { maxPowerW_ = v; }
    int maxPowerJointId() const { return maxPowerJointId_; }
    void setMaxPowerJointId(int id) { maxPowerJointId_ = id; }

    // Bounding box in the root link's local coordinates. Used both to
    // report the current robot dimensions (as bboxMaxRoot - bboxMinRoot) and
    // to render the BB overlay on top of the robot regardless of the robot's
    // world pose.
    const double* bboxMinRoot() const { return bboxMinRoot_; }
    const double* bboxMaxRoot() const { return bboxMaxRoot_; }
    void setBBoxRoot(const double* min, const double* max) {
        for(int i = 0; i < 3; ++i){
            bboxMinRoot_[i] = min[i]; bboxMaxRoot_[i] = max[i];
        }
    }

    // Per-axis maximum bounding-box extents observed since the run started.
    const double* bboxMaxSize() const { return bboxMaxSize_; }
    void setBBoxMaxSize(double x, double y, double z) {
        bboxMaxSize_[0] = x; bboxMaxSize_[1] = y; bboxMaxSize_[2] = z;
    }

    double elapsedTime() const { return elapsedTime_; }
    void setElapsedTime(double t) { elapsedTime_ = t; }

    bool isSectionCleared(int i) const {
        return (flags_ & (SectionCleared0 << i)) != 0;
    }
    void setSectionCleared(int i, bool on) {
        uint64_t mask = SectionCleared0 << i;
        if(on){ flags_ |= mask; } else { flags_ &= ~mask; }
    }
    bool isSectionInProgress(int i) const {
        return (flags_ & (SectionInProgress0 << i)) != 0;
    }
    void setSectionInProgress(int i, bool on) {
        uint64_t mask = SectionInProgress0 << i;
        if(on){ flags_ |= mask; } else { flags_ &= ~mask; }
    }
    bool isSectionPreStage(int i) const {
        return (flags_ & (SectionPreStage0 << i)) != 0;
    }
    void setSectionPreStage(int i, bool on) {
        uint64_t mask = SectionPreStage0 << i;
        if(on){ flags_ |= mask; } else { flags_ &= ~mask; }
    }
    bool isSectionPostStage(int i) const {
        return (flags_ & (SectionPostStage0 << i)) != 0;
    }
    void setSectionPostStage(int i, bool on) {
        uint64_t mask = SectionPostStage0 << i;
        if(on){ flags_ |= mask; } else { flags_ &= ~mask; }
    }

    double sectionClearTime(int i) const { return sectionClearTime_[i]; }
    void setSectionClearTime(int i, double t) { sectionClearTime_[i] = t; }

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    double instantPowerW_;
    double maxPowerW_;
    int maxPowerJointId_;
    double bboxMinRoot_[3];
    double bboxMaxRoot_[3];
    double bboxMaxSize_[3];
    double elapsedTime_;
    uint64_t flags_;
    double sectionClearTime_[MaxNumSections];
};

typedef ref_ptr<VnoidJudgeTargetDevice> VnoidJudgeTargetDevicePtr;

}
}

#endif
