#pragma once

#include "types.h"

namespace cnoid{
namespace vnoid{

/** @brief Simple Visualizer
 *  
 *  Visualizer provides an interface to communicate with MarkerVisualizerPlugin via shared memory.
 *  It enables visualization of user-defined data as simple shapes such as lines, spheres, and boxes.
 * 
 **/

/// Unique name of shared memory. No need to edit.
extern const char* VisualizerSharedMemoryName;

/// struct to keep track of indices of drawn items
struct VizInfo{
	int iframe;
	int ibox;
	int isphere;
	int icylinder;
	int ilines;
};

class Visualizer{
public:	
    struct FrameHeader{
        float  time;
        int    numLines;
        int    numSpheres;
        int    numBoxes;
        int    numCylinders;
    };
    struct Frame : FrameHeader{
    };
    struct Shape{
        Eigen::Matrix<float,3,1,Eigen::DontAlign>  color;
        float       alpha;
    };
    struct ShapeWithPose : Shape{
        Eigen::Matrix<double,3,1,Eigen::DontAlign>  pos;
        Eigen::Quaternion<double,Eigen::DontAlign>  ori;
    };
    struct LinesHeader : Shape{
        int        numVertices;
        int        numIndices;
        float      width;
    };
    struct Lines : LinesHeader{
    };
    struct Sphere : ShapeWithPose{
        float     radius;
    };
    struct Box : ShapeWithPose{
        Eigen::Matrix<double,3,1,Eigen::DontAlign>  size;
    };
    struct Cylinder : ShapeWithPose{
        float     radius;
        float     length;
    };
    struct Header{
        int    numMaxFrames;
        int    numMaxLines;
        int    numMaxSpheres;
        int    numMaxBoxes;
        int    numMaxCylinders;
        int    numMaxLineVertices;
        int    numFrames;
        int    szTotal;
        int    szFrame;
        int    szLines;
        int    szSphere;
        int    szBox;
        int    szCylinder;

        void CalcSize();

        Header();
    };
    struct Data : Header{
        Frame*     GetFrame   (int i);
        Lines*     GetLines   (int iframe, int i);
        Sphere*    GetSphere  (int iframe, int i);
        Box*       GetBox     (int iframe, int i);
        Cylinder*  GetCylinder(int iframe, int i);
        Vector3f*  GetLineVertices(int iframe, int i);
        int*       GetLineIndices (int iframe, int i);
    };

    Header  header;
#ifdef _WIN32
    void*   file;
#else
    int     file;
#endif    
	Data*   data;
	
public:
    /* @brief Open shared memory.
     * 
     * The size of the allocated shared memory is calcuated from the setting of the header.
     * So you need to set appropriate values to numXXX of header before calling Open.
     */
    bool Open();

    /* @brief Close shared memory
     * 
     */ 
    void Close();

     Visualizer();
    ~Visualizer();
};

}
}
