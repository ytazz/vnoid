#include "VnoidJudgeFieldInfo.h"
#include <cnoid/Body>
#include <cnoid/ValueTree>

using namespace std;
using namespace cnoid;
using namespace cnoid::vnoid;

namespace {

bool readBBox(const Listing* listing, Vector3& outMin, Vector3& outMax)
{
    if(!listing || listing->size() < 2) return false;
    auto* a = listing->at(0)->toListing();
    auto* b = listing->at(1)->toListing();
    if(!a || !b || a->size() < 3 || b->size() < 3) return false;
    outMin << a->at(0)->toDouble(), a->at(1)->toDouble(), a->at(2)->toDouble();
    outMax << b->at(0)->toDouble(), b->at(1)->toDouble(), b->at(2)->toDouble();
    return true;
}

}


std::vector<VnoidJudgeFieldSection> cnoid::vnoid::readVnoidJudgeFieldSections(Body* body)
{
    std::vector<VnoidJudgeFieldSection> sections;

    if(!body){
        return sections;
    }
    auto& info = *body->info();
    auto* listing = info.findListing("hvac_field_sections");
    if(!listing || !listing->isValid()){
        return sections;
    }
    bool pendingNewRow = false;
    for(int i = 0; i < listing->size(); ++i){
        auto* node = listing->at(i);
        if(node->isString() && node->toString() == "new_row"){
            pendingNewRow = true;
            continue;
        }
        auto* m = node->toMapping();
        if(!m) continue;
        VnoidJudgeFieldSection s;
        m->read("name", s.name);
        auto* preBB  = m->findListing("pre_stage_bb");
        auto* secBB  = m->findListing("section_bb");
        auto* postBB = m->findListing("post_stage_bb");
        if(!readBBox(preBB,  s.preMin,     s.preMax))     continue;
        if(!readBBox(secBB,  s.sectionMin, s.sectionMax)) continue;
        if(!readBBox(postBB, s.postMin,    s.postMax))    continue;
        s.startsNewRow = pendingNewRow;
        pendingNewRow = false;
        sections.push_back(std::move(s));
    }
    return sections;
}
