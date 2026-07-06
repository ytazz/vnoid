#ifndef CNOID_VNOID_JUDGE_FIELD_INFO_H
#define CNOID_VNOID_JUDGE_FIELD_INFO_H

#include <cnoid/EigenTypes>
#include <vector>
#include <string>

namespace cnoid {

class Body;

namespace vnoid {

/**
   Definition of a judgement section given by the field model.
   The section index in the field's section list corresponds to the
   section flag index of VnoidJudgeTargetDevice.
*/
struct VnoidJudgeFieldSection
{
    std::string name;
    Vector3 preMin, preMax;
    Vector3 sectionMin, sectionMax;
    Vector3 postMin, postMax;

    // True if a "new_row" entry precedes this section in the section list.
    // The HUD display starts a new row from this section.
    bool startsNewRow = false;
};

/**
   Read the section definitions from the "hvac_field_sections" node
   of the body info. An empty vector is returned if the body does not
   have valid section information.
*/
std::vector<VnoidJudgeFieldSection> readVnoidJudgeFieldSections(Body* body);

}
}

#endif
