#include <cnoid/Plugin>
#include <cnoid/ConnectionSet>
#include <cnoid/Item>
#include <cnoid/RenderableItem>
#include <cnoid/ItemList>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshGenerator>

#include "../../src/visualizer.h"

namespace cnoid{
namespace vnoid{

class MarkerVisualizerItem : public Item, public RenderableItem
{
public:
    struct ShapeInfo{
        SgMaterialPtr  mat;

        void SetMaterial (Visualizer::Shape* shape);
        //void SetInvisible();

        ShapeInfo();
    };
    struct ShapeWithPoseInfo : ShapeInfo{
        SgPosTransformPtr  trans;
        SgShapePtr         shape;
        SgMeshPtr          mesh;

        void SetPose(Visualizer::ShapeWithPose* shape);
        void SetOutOfView();

        ShapeWithPoseInfo();
    };
    struct LinesInfo : ShapeInfo{
        SgLineSetPtr      lines;
        SgVertexArrayPtr  vtx;

        LinesInfo();
    };
    struct SphereInfo : ShapeWithPoseInfo{
        float              radius;

        SphereInfo();
    };
    struct BoxInfo : ShapeWithPoseInfo{
        Vector3           size;

        BoxInfo();
    };
    struct CylinderInfo : ShapeWithPoseInfo{
        float  radius;
        float  length;

        CylinderInfo();
    };

    SgPosTransformPtr          sgNode;
    std::vector<LinesInfo>     linesInfo;
    std::vector<SphereInfo>    sphereInfo;
    std::vector<BoxInfo>       boxInfo;
    std::vector<CylinderInfo>  cylinderInfo;

    MeshGenerator meshGen;
    
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

#ifdef _WIN32
    void*  file;
#else
    int    file;
#endif
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

