#pragma once

#include <cnoid/EigenTypes>

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

class Visualizer{
public:	
    struct FrameHeader{
        float  time;
        int    numLines;
        int    numSpheres;
    };
    struct Frame : FrameHeader{
    };
    struct Shape{
        Vector3f  color;
        float     alpha;
    };
    struct LinesHeader : Shape{
        int        numVertices;
        int        numIndices;
    };
    struct Lines : LinesHeader{
    };
    struct Sphere : Shape{
        Vector3f  pos;
        float     radius;
    };
    struct Header{
        int    numMaxFrames;
        int    numMaxLines;
        int    numMaxSpheres;
        int    numMaxLineVertices;
        int    numFrames;
        int    szTotal;
        int    szFrame;
        int    szLines;
        int    szSphere;

        void CalcSize();

        Header();
    };
    struct Data : Header{
        Frame*     GetFrame (int i);
        Lines*     GetLines (int iframe, int i);
        Sphere*    GetSphere(int iframe, int i);
        Vector3f*  GetLineVertices(int iframe, int i);
        int*       GetLineIndices (int iframe, int i);
    };

    Header  header;
    void*   file;
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
