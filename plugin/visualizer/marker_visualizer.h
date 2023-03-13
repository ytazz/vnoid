#include <cnoid/Plugin>
#include <cnoid/ConnectionSet>
#include <cnoid/Item>
#include <cnoid/RenderableItem>
#include <cnoid/ItemList>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/SceneDrawables>

#include "../../src/visualizer.h"

namespace cnoid{
namespace vnoid{

class MarkerVisualizerItem : public Item, public RenderableItem
{
public:
    SgPosTransformPtr              sgNode;
    std::vector<SgLineSetPtr>      sgLines;
    std::vector<SgVertexArrayPtr>  sgVtxs;
    std::vector<SgMaterialPtr>     sgLineMaterials;
    bool  ready;

public:
    void Prepare();
    void Sync(Visualizer::Data*  data, int iframe);

    virtual Item* doDuplicate() const override;
    
    virtual SgNode* getScene() override;

    MarkerVisualizerItem();
    MarkerVisualizerItem(const MarkerVisualizerItem& org);
};

class MarkerVisualizerPlugin : public Plugin
{
public:
    ScopedConnectionSet connections;

    void*  file;
	Visualizer::Data*  data;

public:
    bool Open(size_t sz);
    void Close();

    //void onSelectedItemsChanged(ItemList<BodyItem> selectedBodyItems);
    bool onTimeChanged(double time);
    //void onUpdate();
    
    virtual bool initialize() override;

    MarkerVisualizerPlugin();
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(MarkerVisualizerPlugin)

}
}

