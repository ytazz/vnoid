#include "visualizer.h"

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

const char* VisualizerSharedMemoryName = "vnoid_marker_visualizer_shmem";

Visualizer::Header::Header(){
	numMaxFrames       = 1000;
    numMaxLines        = 10;
    numMaxSpheres      = 10;
    numMaxLineVertices = 2000;

    numFrames = 0;

	CalcSize();
}

void Visualizer::Header::CalcSize(){
    szLines  = sizeof(LinesHeader) + (sizeof(Vector3f) + sizeof(int))*numMaxLineVertices;
    szSphere = sizeof(Sphere);
    szFrame  = sizeof(FrameHeader) + numMaxLines*szLines + numMaxSpheres*szSphere;
	szTotal  = sizeof(Header) + numMaxFrames*szFrame;
}

Visualizer::Frame*  Visualizer::Data::GetFrame(int iframe){
	if(iframe >= numMaxFrames)
		return 0;

	if(iframe >= numFrames)
		numFrames = iframe + 1;

	return (Frame*)(((byte*)this) + sizeof(Header) + szFrame*iframe);
}

Visualizer::Lines*  Visualizer::Data::GetLines(int iframe, int i){
	Frame* fr = GetFrame(iframe);
	if(!fr)
		return 0;

	if(i >= numMaxLines)
		return 0;

	if(i >= fr->numLines)
		fr->numLines = i + 1;

	return (Lines*)(((byte*)fr) + sizeof(FrameHeader) + szLines*i);
}

Visualizer::Sphere* Visualizer::Data::GetSphere(int iframe, int i){
	Frame* fr = GetFrame(iframe);
	if(!fr)
		return 0;

	if(i >= numMaxSpheres)
		return 0;

	if(i >= fr->numSpheres)
		fr->numSpheres = i + 1;

	return (Sphere*)(((byte*)fr) + sizeof(FrameHeader) + szLines*numMaxLines + szSphere*i);
}

Vector3f* Visualizer::Data::GetLineVertices(int iframe, int i){
	Lines* lines = GetLines(iframe, i);
	if(!lines)
		return 0;

	return (Vector3f*)((byte*)lines + sizeof(LinesHeader));
}

int* Visualizer::Data::GetLineIndices(int iframe, int i){
	Lines* lines = GetLines(iframe, i);
	if(!lines)
		return 0;

	return (int*)((byte*)lines + sizeof(LinesHeader) + sizeof(Vector3f)*numMaxLineVertices);
}

Visualizer::Visualizer(){
    file = 0;
	data = 0;
}

Visualizer::~Visualizer(){
	Close();
}

bool Visualizer::Open(){
	if(file)
		Close();

	header.CalcSize();

	const char* name = VisualizerSharedMemoryName;
    const size_t sz = header.szTotal;
#ifdef _WIN32
	file = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, (DWORD)sz, name);
    data = (Data*)MapViewOfFile(file, FILE_MAP_WRITE, 0, (DWORD)0, (DWORD)sz);
#else
	// append '/' in front of shared memory name
	char n[256];
	sprintf(n, "/%s", name);
	file = shm_open(n, O_RDWR | O_CREAT, 0644); 
	ftruncate(file, sz); 
    data = (Data*) mmap(NULL, sz, PROT_READ|PROT_WRITE, MAP_SHARED, file, 0); 
#endif

    memset(data, 0, header.szTotal);
	memcpy(data, &header, sizeof(Header));

	return true;
}

void Visualizer::Close(){
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

}
}
