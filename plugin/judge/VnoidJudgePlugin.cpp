#include "VnoidJudgeTargetItem.h"
#include "VnoidJudgeItem.h"
#include <cnoid/Plugin>

using namespace cnoid;
using namespace cnoid::vnoid;


class VnoidJudgePlugin : public Plugin
{
public:
    VnoidJudgePlugin() : Plugin("VnoidJudge")
    {
        require("Body");
    }

    virtual bool initialize() override
    {
        VnoidJudgeTargetItem::initializeClass(this);
        VnoidJudgeItem::initializeClass(this);
        return true;
    }
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(VnoidJudgePlugin)
