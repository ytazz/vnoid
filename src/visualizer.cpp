#include "visualizer.h"

#ifdef _WIN32
# include <windows.h>
#else
# include <sys/mman.h>
# include <sys/stat.h>
# include <fcntl.h>
# include <unistd.h>
#endif

#include <stdint.h>

namespace cnoid{
namespace vnoid{

const char* VisualizerSharedMemoryName = "vnoid_marker_visualizer_shmem";

Visualizer::Header::Header(){
	numMaxFrames       = 1000;
    numMaxLines        = 10;
    numMaxSpheres      = 10;
	numMaxBoxes        = 10;
	numMaxCylinders    = 10;
    numMaxLineVertices = 2000;

    numFrames = 0;

	CalcSize();
}

void Visualizer::Header::CalcSize(){
    szLines    = sizeof(LinesHeader) + (sizeof(Vector3f) + sizeof(int))*numMaxLineVertices;
    szSphere   = sizeof(Sphere);
	szBox      = sizeof(Box);
	szCylinder = sizeof(Cylinder);
    szFrame    = sizeof(FrameHeader) + numMaxLines*szLines + numMaxSpheres*szSphere + numMaxBoxes*szBox + numMaxCylinders*szCylinder;
	szTotal    = sizeof(Header) + numMaxFrames*szFrame;
}

Visualizer::Frame*  Visualizer::Data::GetFrame(int iframe){
	if(iframe >= numMaxFrames)
		return 0;

	if(iframe >= numFrames)
		numFrames = iframe + 1;

	return (Frame*)(((uint8_t*)this) + sizeof(Header) + szFrame*iframe);
}

Visualizer::Lines*  Visualizer::Data::GetLines(int iframe, int i){
	Frame* fr = GetFrame(iframe);
	if(!fr)
		return 0;

	if(i >= numMaxLines)
		return 0;

	if(i >= fr->numLines)
		fr->numLines = i + 1;

	Lines* lines = (Lines*)(((uint8_t*)fr) + sizeof(FrameHeader) + szLines*i);
	//new(lines) Lines();
	
	return lines;
}

Visualizer::Sphere* Visualizer::Data::GetSphere(int iframe, int i){
	Frame* fr = GetFrame(iframe);
	if(!fr)
		return 0;

	if(i >= numMaxSpheres)
		return 0;

	if(i >= fr->numSpheres)
		fr->numSpheres = i + 1;

	Sphere* sphere = (Sphere*)(((uint8_t*)fr) + sizeof(FrameHeader) + szLines*numMaxLines + szSphere*i);
	//new(sphere) Sphere();
	return sphere;
}

Visualizer::Box* Visualizer::Data::GetBox(int iframe, int i){
	Frame* fr = GetFrame(iframe);
	if(!fr)
		return 0;

	if(i >= numMaxBoxes)
		return 0;

	if(i >= fr->numBoxes)
		fr->numBoxes = i + 1;

	Box* box = (Box*)(((uint8_t*)fr) + sizeof(FrameHeader) + szLines*numMaxLines + szSphere*numMaxSpheres + szBox*i);
	//new(box) Box();
	return box;
}

Visualizer::Cylinder* Visualizer::Data::GetCylinder(int iframe, int i){
	Frame* fr = GetFrame(iframe);
	if(!fr)
		return 0;

	if(i >= numMaxCylinders)
		return 0;

	if(i >= fr->numCylinders)
		fr->numCylinders = i + 1;

	Cylinder* cylinder = (Cylinder*)(((uint8_t*)fr) + sizeof(FrameHeader) + szLines*numMaxLines + szSphere*numMaxSpheres + szBox*numMaxBoxes + szCylinder*i);
	
	return cylinder;
}

Vector3f* Visualizer::Data::GetLineVertices(int iframe, int i){
	Lines* lines = GetLines(iframe, i);
	if(!lines)
		return 0;

	return (Vector3f*)((uint8_t*)lines + sizeof(LinesHeader));
}

int* Visualizer::Data::GetLineIndices(int iframe, int i){
	Lines* lines = GetLines(iframe, i);
	if(!lines)
		return 0;

	return (int*)((uint8_t*)lines + sizeof(LinesHeader) + sizeof(Vector3f)*numMaxLineVertices);
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
	file = shm_open(n, O_RDWR | O_CREAT, (S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH)); 
	if(file == -1){
	    printf("vnoid visualizer: failed to open shared memory file\n");
	    file = 0;
	    return false;
	}
	printf("vnoid visualizer: mapping to memory. size: %d\n", (int)sz);
	ftruncate(file, sz); 
	data = (Data*) mmap(NULL, sz, PROT_READ|PROT_WRITE, MAP_SHARED, file, 0); 
	if(data == (void*)-1){
	    printf("vnoid visualizer: failed to map shared memory\n");
	    file = 0;
	    data = 0;
	    return false;	
	}
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
    if(data)
        munmap(data, header.szTotal);
       
    if(file)
    	close(file);
#endif

	file = 0;
	data = 0;
}

}
}
