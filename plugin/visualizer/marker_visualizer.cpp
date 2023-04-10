#include "marker_visualizer.h"

#include <cnoid/MessageView>
#include <cnoid/ItemManager>
#include <cnoid/TimeBar>

#ifdef _WIN32
# include <windows.h>
#else
# include <sys/mman.h>
# include <sys/stat.h>
# include <fcntl.h>
# include <unistd.h>
#endif


namespace cnoid{
namespace vnoid{

MarkerVisualizerItem::ShapeInfo::ShapeInfo(){
    mat = new SgMaterial();
};

void MarkerVisualizerItem::ShapeInfo::SetMaterial(Visualizer::Shape* shape){
    mat->setDiffuseColor(shape->color);
    mat->setTransparency(shape->alpha);
}

MarkerVisualizerItem::ShapeWithPoseInfo::ShapeWithPoseInfo(){
    trans  = new SgPosTransform();
    shape  = new SgShape();
    trans->addChild(shape);
    shape->setMaterial(mat);
};

void MarkerVisualizerItem::ShapeWithPoseInfo::SetPose(Visualizer::ShapeWithPose* shape){
    trans->setTranslation(shape->pos);
    trans->setRotation   (shape->ori);
}

MarkerVisualizerItem::LinesInfo::LinesInfo(){
    lines = new SgLineSet();
    vtx   = new SgVertexArray();
    lines->setVertices(vtx);
    lines->setMaterial(mat);
};

MarkerVisualizerItem::SphereInfo::SphereInfo(){
    radius = 0.0f;
};

MarkerVisualizerItem::BoxInfo::BoxInfo(){
    size = Vector3(0.0, 0.0, 0.0);
};

MarkerVisualizerItem::MarkerVisualizerItem()
{
    ready = false;
}

MarkerVisualizerItem::MarkerVisualizerItem(const MarkerVisualizerItem& org) : Item(org)
{
}

Item* MarkerVisualizerItem::doDuplicate() const
{
    return new MarkerVisualizerItem(*this);
}

void MarkerVisualizerItem::Prepare(){
    sgNode = new SgPosTransform();

    ready = true;
}

void MarkerVisualizerItem::Sync(Visualizer::Data*  data, int iframe){
    Visualizer::Frame* fr = data->GetFrame(iframe);

    while(linesInfo.size() < fr->numLines){
        linesInfo.push_back(LinesInfo());
        sgNode->addChild(linesInfo.back().lines);
    }
    while(sphereInfo.size() < fr->numSpheres){
        sphereInfo.push_back(SphereInfo());
        sgNode->addChild(sphereInfo.back().trans);
    }
    while(boxInfo.size() < fr->numBoxes){
        boxInfo.push_back(BoxInfo());
        sgNode->addChild(boxInfo.back().trans);
    }

    for(int i = 0; i < fr->numLines; i++){
        Visualizer::Lines* lines = data->GetLines(iframe, i);
        LinesInfo& li = linesInfo[i];

        Vector3f* pvtx = data->GetLineVertices(iframe, i);
        int*      pidx = data->GetLineIndices (iframe, i);

        li.vtx->resize(lines->numVertices);
        std::copy(pvtx, pvtx + lines->numVertices, li.vtx->begin());
        li.lines->lineVertexIndices() = std::vector<int>(pidx, pidx + lines->numIndices);

        li.SetMaterial(lines);

        li.lines->notifyUpdate();
    }
    for(int i = 0; i < fr->numSpheres; i++){
        Visualizer::Sphere* sphere = data->GetSphere(iframe, i);
        SphereInfo& si = sphereInfo[i];

        if(si.radius != sphere->radius){
            si.mesh = meshGen.generateSphere(sphere->radius);
            si.radius = sphere->radius;
            si.shape->setMesh(si.mesh);
        }

        si.SetPose(sphere);
        si.SetMaterial(sphere);
        
        si.trans->notifyUpdate();
    }
    for(int i = 0; i < fr->numBoxes; i++){
        Visualizer::Box* box = data->GetBox(iframe, i);
        BoxInfo& bi = boxInfo[i];

        if(bi.size != box->size){
            bi.mesh = meshGen.generateBox(box->size);
            bi.size = box->size;
            bi.shape->setMesh(bi.mesh);
        }

        bi.SetPose(box);
        bi.SetMaterial(box);

        bi.shape->notifyUpdate();
    }
}

SgNode* MarkerVisualizerItem::getScene(){
    if(!ready)
        Prepare();

    return sgNode;
}

MarkerVisualizerPlugin::MarkerVisualizerPlugin() : Plugin("MarkerVisualizer")
{
    require("Body");
}

bool MarkerVisualizerPlugin::initialize()
{
    itemManager()
        .registerClass<MarkerVisualizerItem>("MarkerVisualizerItem")
        .addCreationPanel<MarkerVisualizerItem>();

	/*
    connections.add(
        RootItem::instance()->sigSelectedItemsChanged().connect(
            [this](const ItemList<>& selectedItems){
                onSelectedItemsChanged(selectedItems);
            }));
    */
    connections.add(
        TimeBar::instance()->sigTimeChanged().connect(
            [this](double time){ return onTimeChanged(time); })
        );

    //MessageView::instance()->putln("Hello World!");

    file = 0;
    data = 0;

    return true;
}

bool MarkerVisualizerPlugin::Open(size_t sz){
   	const char* name = VisualizerSharedMemoryName;

#ifdef _WIN32
	file = OpenFileMappingA(FILE_MAP_WRITE, FALSE, name);
	data = (Visualizer::Data*)MapViewOfFile(file, FILE_MAP_WRITE, 0, (DWORD)0, (DWORD)sz);
#else
	// append '/' in front of shared memory name
	char n[256];
	sprintf(n, "/%s", name);
	file = shm_open(n, O_RDWR, 0644);
	data = (Visualizer::Data*) mmap(NULL, sz, PROT_READ|PROT_WRITE, MAP_SHARED, file, 0); 
#endif

	if(!file || !data)
		return false;

    return true;
}

void MarkerVisualizerPlugin::Close(){
#ifdef _WIN32
	if(data)
		UnmapViewOfFile(data);

	if(file)
		CloseHandle(file);
#else
	close(file);
#endif

	file = 0;
	data = 0;
}

bool MarkerVisualizerPlugin::onTimeChanged(double time){
    if(!data){
        if(!Open(sizeof(Visualizer::Header)))
            return true;

        size_t sz = data->szTotal;
        Close();

        if(!Open(sz))
            return true;
    }

    int iframe = -1;
    for(int k = data->numFrames-1; k >= 0; k--){
        if(data->GetFrame(k)->time <= time){
            iframe = k;
            break;
        }
    }
    if(iframe == -1)
        return false;
    
    printf("%d %d %f %f\n", data->numFrames, iframe, data->GetFrame(iframe)->time, time);

    for(auto& item : RootItem::instance()->checkedItems<MarkerVisualizerItem>()){
        item->Sync(data, iframe);
        item->notifyUpdate();
    }

    return true;
}

//void MarkerVisualizerPlugin::onUpdate(){
//}

}
}
