#include "marker_visualizer.h"
#include "marker_item.h"

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

    /*
    SgLineSetPtr lines = new SgLineSet();
    SgVertexArrayPtr vtxs = new SgVertexArray();

    lines->setVertices(vtxs);
    
    vtxs->resize(6);
    (*vtxs)[0] = Vector3f(-1.0f,  0.0f,  0.0f);
    (*vtxs)[1] = Vector3f( 1.0f,  0.0f,  0.0f);
    (*vtxs)[2] = Vector3f( 0.0f, -1.0f,  0.0f);
    (*vtxs)[3] = Vector3f( 0.0f,  1.0f,  0.0f);
    (*vtxs)[4] = Vector3f( 0.0f,  0.0f, -1.0f);
    (*vtxs)[5] = Vector3f( 0.0f,  0.0f,  1.0f);
    
    lines->lineVertexIndices() =
        { 0, 1,
          2, 3,
          4, 5 };

    node->addChild(lines);
    */

    ready = true;
}

void MarkerVisualizerItem::Sync(Visualizer::Data*  data, int iframe){
    Visualizer::Frame* fr = data->GetFrame(iframe);

    while(sgLines.size() < fr->numLines){
        sgLines        .push_back(new SgLineSet());
        sgVtxs         .push_back(new SgVertexArray());
        sgLineMaterials.push_back(new SgMaterial());
        sgLines.back()->setVertices(sgVtxs.back());
        sgLines.back()->setMaterial(sgLineMaterials.back());
        sgNode->addChild(sgLines.back());
    }

    for(int i = 0; i < fr->numLines; i++){
        Visualizer::Lines* lines = data->GetLines(iframe, i);
        Vector3f* pvtx = data->GetLineVertices(iframe, i);
        int*      pidx = data->GetLineIndices (iframe, i);

        sgVtxs[i]->resize(lines->numVertices);
        std::copy(pvtx, pvtx + lines->numVertices, sgVtxs[i]->begin());
        sgLines[i]->lineVertexIndices() = std::vector<int>(pidx, pidx + lines->numIndices);

        sgLineMaterials[i]->setDiffuseColor(lines->color);
        sgLineMaterials[i]->setTransparency(lines->alpha);

        sgLines[i]->notifyUpdate();
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
    
    //printf("%d %d\n", data->numFrames, iframe);

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
