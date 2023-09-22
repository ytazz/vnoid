#include <cnoid/Body>
#include <cnoid/LinkedJointHandler>
#include <fmt/format.h>

using namespace std;
using namespace cnoid;
using fmt::format;

#define TWO_LINK_CONSTRAINT 1

class SwingBridgeHandler : public LinkedJointHandler
{
public:
    virtual BodyHandler* clone() override;
    virtual bool initialize(Body* body, std::ostream& os) override;
    virtual bool updateLinkedJointDisplacements(
        Link* masterJoint, double masterJointDisplacement) override;

private:
#if TWO_LINK_CONSTRAINT
    static const int NUM_JOINTS = 3;
#else
    static const int NUM_JOINTS = 5;
#endif
    Link* joints[NUM_JOINTS];
};

CNOID_IMPLEMENT_BODY_HANDLER_FACTORY(SwingBridgeHandler)


BodyHandler* SwingBridgeHandler::clone()
{
    return new SwingBridgeHandler(*this);
}


bool SwingBridgeHandler::initialize(Body* body, std::ostream& os)
{
#if TWO_LINK_CONSTRAINT
    const std::vector<std::string> names = {"Link1", "Link3", "MovablePlate"};
#else
    const std::vector<std::string> names = {"Link1",
                                            "Link2",
                                            "Link3",
                                            "Link4",
                                            "MovablePlate"};
#endif
    for (int i = 0; i < NUM_JOINTS; ++i) {
        joints[i] = body->link(names[i]);
        if (!joints[i]) {
            os << names[i] << "is not found." << endl;
            return false;
        }
    }
    return true;
}


bool SwingBridgeHandler::updateLinkedJointDisplacements(
    Link* masterJoint, double masterJointDisplacement)
{
#if TWO_LINK_CONSTRAINT
    if (masterJoint) {
        masterJoint->q() = masterJointDisplacement;
    }
    if (!masterJoint || masterJoint == joints[0]) {
        joints[1]->q() = joints[0]->q();
    } else if (masterJoint == joints[1]) {
        joints[0]->q() = joints[1]->q();
    }
#else
    if (masterJoint) {
        masterJoint->q() = masterJointDisplacement;
    }
    if (!masterJoint || masterJoint == joints[0]) {
        joints[1]->q() = joints[0]->q();
        joints[2]->q() = joints[0]->q();
        joints[3]->q() = joints[0]->q();
    } else if (masterJoint == joints[1]) {
        joints[0]->q() = joints[1]->q();
        joints[2]->q() = joints[1]->q();
        joints[3]->q() = joints[1]->q();
    } else if (masterJoint == joints[2]) {
        joints[0]->q() = joints[2]->q();
        joints[1]->q() = joints[2]->q();
        joints[3]->q() = joints[2]->q();
    } else if (masterJoint == joints[3]) {
        joints[0]->q() = joints[3]->q();
        joints[1]->q() = joints[3]->q();
        joints[2]->q() = joints[3]->q();
    }
#endif
    return true;
}
