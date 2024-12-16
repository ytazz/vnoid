#pragma once

#include "types.h"
#include "filter.h"
#include "robot_base.h"
#include "visualizer.h"

#include <iostream>
#include <vector>

/*  in bvh:
	y up
	x left
	z front
	cm
	deg

	to

	z up
	y left
	x front
	m
	rad

	up    y -> z
	left  x -> y
	front z -> x

	0 0 1
	1 0 0
	0 1 0

	bvh rotation order: ZXY
	after ordering: XYZ
*/
namespace cnoid{
namespace vnoid{

class Tokenizer{
public:
	std::string::const_iterator  str_begin, str_end;
	std::string::const_iterator  token_begin, token_end;
	std::string  delim;
	
	bool IsDelim(char c){
		for(int i = 0; i < (int)delim.size(); i++)
			if(delim[i] == c)
				return true;
		return false;
	}

public:
	void Set(const std::string& s, const std::string& d){
		str_begin = s.begin();
		str_end   = s.end();
		delim     = d;
		token_begin = token_end = str_begin;
		while(token_end != str_end && !IsDelim(*token_end))
			token_end++;
	}

	std::string GetToken(){
		return std::string(token_begin, token_end);
	}

	void Next(){
		token_begin = token_end;
		if(IsEnd())
			return;

		do{
			token_begin++;
		}
		while((token_begin != str_end) && IsDelim(*token_begin));
	
		token_end = token_begin;
		while(token_end != str_end && !IsDelim(*token_end))
			token_end++;

	}

	bool IsEnd(){
		return (token_begin == str_end);
	}

	Tokenizer(){
		delim.resize(1);
		delim[0] = ' ';
	}

	Tokenizer(const std::string& s, const std::string& d){
		Set(s, d);
	}
};

class Bvh{
public:
	struct Node{
		Bvh*            owner;
		std::string		name;
		int				numChannels;
		int				channelOffset;
		int				channels[6];
		Vector3         posOffset;
		double          mass;

		Filter          filter[6];

		Vector6         theta;
		
		Vector3         posRel;
		Quaternion      oriRel;

		Vector3         angle;
		Vector3         angled;
		Vector3         angledd;

		Vector3         pos;
		Quaternion      ori;
		Vector3         vel;
		Vector3         angvel;
		Vector3         acc;
		Vector3         angacc;

		Node*			parent;
		std::vector<Node*>	children;

		void AddChild(Node* n){
			n->parent = this;
			children.push_back(n);
		}
		void PrintRecurs(std::ostream& os, int depth);
		void CalcPoseRecurs(int idx0, int idx1, double tau, double dt);
	
		Node();
	};
	/*
	struct Centroid{
		Vector3  pos;    //< com position
		Vector3  vel;    //< com velocity
		Vector3  acc;
		Vector3  momentum;  //< momentum

		Vector3  force;    //< force acting on com
		Vector3  moment;   //< moment around com
	};
	*/
	std::string  name;
	std::string  filename;

	Tokenizer  tokenizer;

	Node*	root;				///< root node
	int		numFrames;			///< number of frames
	int		numChannels;		///< total number of channels
	double  sampleRate;			///< sample rate [sec]

	struct Channel{
		enum{
			Xposition,
			Yposition,
			Zposition,
			Xrotation,
			Yrotation,
			Zrotation,
		};
	};
	
	std::string  contents;			///< contents string

	Node*	curNode;
	
	typedef std::vector<Node*>	Nodes;
	Nodes	nodes;

	std::vector<double>	data;		///< motion data: numChannels * numFrames elements

public:
	void CreateNode(std::string name);
	void ParseData ();
	void Parse     ();
	
	void  Clear       ();
	bool  Load        (std::string filename);
	//void  WriteCsv    (std::string filename);
	//void  Print       (std::ostream& os);
	Node* FindNode    (std::string name);
	void  CalcPose    (double time);
	void  CalcCentroid(Centroid& centroid);
	void  Visualize   (Visualizer* viz, VizInfo& info);
	//void  CalcCentroid();

	Bvh();
};

}
}