#include "bvh.h"
#include "rollpitchyaw.h"

#include <fstream>
using namespace std;

const double pi = 3.1415926535;

namespace cnoid{
namespace vnoid{

Bvh::Node::Node(){
	numChannels   = 0;
	channelOffset = 0;
	parent        = 0;
	mass          = 0.0;

	for(int i = 0; i < 6; i++){
		filter[i].SetCutoff(0.5 * 1.0/0.025);
	}
}
/*
void Bvh::Node::PrintRecurs(ostream& os, int depth){
	for(int i = 0; i < depth; i++)
		os << ' ';
	
	os << name << ' '; 
	
	for(Node* child : children){
		child->PrintRecurs(os, depth+2);
	}
}
*/
void Bvh::Node::CalcPoseRecurs(int idx0, int idx1, double tau, double dt){
	double dtinv = 1.0/dt;
	double s     = tau*dtinv;

	theta = Vector6::Zero();
	
	for(int i = 0; i < numChannels; i++){
		double val0 = owner->data[idx0 + channelOffset + i];
		double val1 = owner->data[idx1 + channelOffset + i];
		double diff = val1 - val0;
		
		int ch = channels[i];
		if(ch < 3){
			// cm to m
			val0 *= 0.01;
	
			     if(ch == Channel::Xposition){ theta[1] = val0; }
			else if(ch == Channel::Yposition){ theta[2] = val0; }
			else if(ch == Channel::Zposition){ theta[0] = val0; }
		}
		else{
			// deg to rad
			val0 = (pi/180.0)*val0;
			
			     if(ch == Channel::Xrotation){ theta[4] = val0; }
			else if(ch == Channel::Yrotation){ theta[5] = val0; }
			else if(ch == Channel::Zrotation){ theta[3] = val0; }
		 }
	}

	posRel = posOffset + Vector3(theta[0], theta[1], theta[2]);
	oriRel = FromRollPitchYaw(Vector3(theta[3], theta[4], theta[5]));

	if(!parent){
		pos = posRel;
		ori = oriRel;
	}
	else{
		pos = parent->pos + parent->ori*posRel;
		ori = parent->ori * oriRel;
	}

	angle = ToRollPitchYaw(ori);

	// reset filter at the first frame
	if(idx0 == 0){
		for(int i = 0; i < 3; i++){
			filter[0+i].y = pos  [i];
			filter[3+i].y = angle[i];
		}
	}

	for(int i = 0; i < 3; i++){
		filter[0+i](pos[i], 0.001);
		vel[i] = filter[0+i].yd;
		acc[i] = filter[0+i].ydd;

		filter[3+i](angle[i], 0.001);
		angled [i] = filter[3+i].yd;
		angledd[i] = filter[3+i].ydd;
	}

	//angvel = VelocityFromRollPitchYaw(angle, angled);
	//angacc = AccelerationFromRollPitchYaw(angle, angled, angledd);

	for(Node* n : children)
		n->CalcPoseRecurs(idx0, idx1, tau, dt);

}

Bvh::Bvh(){
	Clear();
}

void Bvh::Clear(){
	nodes.clear();
	root    = 0;
	curNode = 0;

	numFrames   = 0;
	numChannels = 0;
	sampleRate  = 0.0f;
}

bool Bvh::Load(string filename){
	ifstream ifs;
	ifs.open(filename, ifstream::in);

	if(!ifs.is_open())
		return false;

	char c;
	contents.clear();
	while(c = ifs.get(), ifs.good())
		contents += c;

	ifs.close();
	
	// tokenize
	tokenizer = Tokenizer(contents, " \t\n");
	
	// parse
	try{
		Parse();
	}
	catch(...){
		return false;
	}
	return true;
}
/*
void Bvh::WriteCsv(string filename){
	FILE* file = fopen(filename.c_str(), "w");

	for(Node* n : nodes){
		fprintf(file, "%s_x, %s_y, %s_z, ", n->name.c_str(), n->name.c_str(), n->name.c_str());
	}
	fprintf(file, 
		    "cpx, cpy, cpz, "
		    "cvx, cvy, cvz, "
		    "cax, cay, caz, "
		    "Lx, Ly, Lz, "
		    "fx, fy, fz, "
		    "mx, my, mz, \n");
		
	Centroid centroid;
	for(int k = 0; k < numFrames; k++){
		double  t = k*sampleRate;

		CalcPose(t);

		for(Node* n : nodes){
			fprintf(file, "%f, %f, %f, ", n->pos.x(), n->pos.y(), n->pos.z());
		}

		fprintf(file, "%f, %f, %f, ", centroid.com_pos_ref.x(), centroid.com_pos_ref.y(), centroid.com_pos_ref.z());
		fprintf(file, "%f, %f, %f, ", centroid.com_vel_ref.x(), centroid.com_vel_ref.y(), centroid.com_vel_ref.z());
		fprintf(file, "%f, %f, %f, ", centroid.com_acc_ref.x(), centroid.com_acc_ref.y(), centroid.com_acc_ref.z());

		//fprintf(file, "%f, %f, %f, ", centroid.momentum.x(), centroid.momentum.y(), centroid.momentum.z());

		fprintf(file, "%f, %f, %f, ", centroid.force_ref .x(), centroid.force_ref .y(), centroid.force_ref .z());
		fprintf(file, "%f, %f, %f, ", centroid.moment_ref.x(), centroid.moment_ref.y(), centroid.moment_ref.z());

		fprintf(file, "\n");
	}

	fclose(file);
}
*/
void Bvh::CreateNode(string name){
	Node* child = new Node();
	child->owner = this;
	child->name  = name;
	child->channelOffset = numChannels;

	if(!curNode){
		root = curNode = child;
	}
	else{
		curNode->AddChild(child);
		curNode = child;
	}
	nodes.push_back(child);
}

void Bvh::Parse(){
	Clear();

	while(!tokenizer.IsEnd()){
		auto tok = tokenizer.GetToken();
	
		// beginning of header section
		if(tok == "HIERARCHY"){
				
		}
		// root identifier
		else if(tok == "ROOT"){
			tokenizer.Next();
			CreateNode(tokenizer.GetToken());
			tokenizer.Next();
			assert(tokenizer.GetToken() == "{");
		}
		else if(tok == "MASS"){
			// mass: custom extension. not in original bvh
			assert(curNode);
			tokenizer.Next(); curNode->mass = stod(tokenizer.GetToken());
		}
		else if(tok == "OFFSET"){
			assert(curNode);
			tokenizer.Next(); curNode->posOffset.y() = stod(tokenizer.GetToken());
			tokenizer.Next(); curNode->posOffset.z() = stod(tokenizer.GetToken());
			tokenizer.Next(); curNode->posOffset.x() = stod(tokenizer.GetToken());

			// scaling and rotation
			curNode->posOffset = 0.01*curNode->posOffset;
		}
		else if(tok == "CHANNELS"){
			assert(curNode);
			tokenizer.Next();
			int num = stoi(tokenizer.GetToken());
			curNode->numChannels = num;
			numChannels += num;
			for(int i = 0; i < num; i++){
				tokenizer.Next();
				auto tokChannel = tokenizer.GetToken();

				     if(tokChannel == "Xposition") curNode->channels[i] = Channel::Xposition;
				else if(tokChannel == "Yposition") curNode->channels[i] = Channel::Yposition;
				else if(tokChannel == "Zposition") curNode->channels[i] = Channel::Zposition;
				else if(tokChannel == "Xrotation") curNode->channels[i] = Channel::Xrotation;
				else if(tokChannel == "Yrotation") curNode->channels[i] = Channel::Yrotation;
				else if(tokChannel == "Zrotation") curNode->channels[i] = Channel::Zrotation;
			}
		}
		else if(tok == "JOINT"){
			assert(curNode);
			tokenizer.Next();
			CreateNode(tokenizer.GetToken());
			tokenizer.Next();
			assert(tokenizer.GetToken() == "{");
		}
		else if(tok == "End"){
			tokenizer.Next();
			assert(tokenizer.GetToken() == "Site");
			tokenizer.Next();
			assert(tokenizer.GetToken() == "{");
			CreateNode("");
		}
		else if(tok == "}"){
			assert(curNode);
			if(curNode->parent)
				curNode = curNode->parent;
		}	
		else if(tok == "MOTION"){

		}
		else if(tok == "Frames:"){
			tokenizer.Next();
			numFrames = stoi(tokenizer.GetToken());
		}
		else if(tok == "Frame"){
			tokenizer.Next();
			assert(tokenizer.GetToken() == "Time:");
			tokenizer.Next();
			sampleRate = stod(tokenizer.GetToken());

			// motion data comes next
			ParseData();
		}

		tokenizer.Next();
	}
}

void Bvh::ParseData(){
	data.resize(numChannels * numFrames);
	fill(data.begin(), data.end(), 0.0);

	for(vector<double>::iterator it = data.begin(); it != data.end(); it++){
		if(tokenizer.IsEnd())
			break;

		tokenizer.Next();
		*it = stod(tokenizer.GetToken());
	}
}
/*
void Bvh::Print(ostream& os){
	if(!root)
		return;

	root->PrintRecurs(os, 0);
}
*/
Bvh::Node* Bvh::FindNode(string name){
	for(Node* n : nodes){
		if(n->name == name)
			return n;
	}
	return 0;
}

void Bvh::CalcPose(double time){
	double dt = sampleRate;
	int    N  = numFrames;
	
	int fr0 = (int)(time/dt);
	int fr1 = fr0 + 1;

	fr0 = std::min(std::max(0, fr0), N-1);
	fr1 = std::min(std::max(0, fr1), N-1);

	double tau = time - fr0*dt;
	
	root->CalcPoseRecurs(fr0*numChannels, fr1*numChannels, tau, dt);
}

void Bvh::CalcCentroid(Centroid& centroid){
	// calc com
	double total_mass = 0.0;
	centroid.com_pos_ref = Vector3::Zero();
	centroid.com_vel_ref = Vector3::Zero();
	centroid.com_acc_ref = Vector3::Zero();

	for(Node* n : nodes){
		total_mass += n->mass;
		centroid.com_pos_ref += n->mass*n->pos;
		centroid.com_vel_ref += n->mass*n->vel;
		centroid.com_acc_ref += n->mass*n->acc;
	}
	centroid.com_pos_ref /= total_mass;
	centroid.com_vel_ref /= total_mass;
	centroid.com_acc_ref /= total_mass;

	/*
	// calc angular momentum and its derivative (which is moment)
	centroid.momentum = Vector3::Zero();
	centroid.moment   = Vector3::Zero();
	for(Node* n : nodes){
		centroid.momentum += (n->pos - centroid.pos) % (n->mass * n->vel);
		centroid.moment   += (n->vel - centroid.vel) % (n->mass * n->vel) + (n->pos - centroid.pos) % (n->mass * n->acc);
	}
	*/

	centroid.force_ref = total_mass*centroid.com_acc_ref;
}

void Bvh::Visualize(Visualizer* viz, VizInfo& info){
	// sphere indicating node position
	for(auto n : nodes){
		Visualizer::Sphere* sphereCom = viz->data->GetSphere(info.iframe, info.isphere++);
		sphereCom->color  = Eigen::Vector3f(0.5f, 0.5f, 0.5f);
		sphereCom->alpha  = 0.5f;
		sphereCom->pos    = n->pos;
		sphereCom->radius = 0.005f;
	}

	// lines connecting nodes
	Visualizer::Lines* lines = viz->data->GetLines(info.iframe, info.ilines);
    lines->color = Eigen::Vector3f(0.5f, 0.5f, 0.5f);
    lines->alpha = 0.5f;
    lines->width = 1.0f;
    int iv = 0;
    int ii = 0;
    for(auto n : nodes){
		Bvh::Node* np = n->parent;
		if(!np)
			continue;
	    viz->data->GetLineVertices(info.iframe, info.ilines)[iv+0] = n ->pos.cast<float>();
        viz->data->GetLineVertices(info.iframe, info.ilines)[iv+1] = np->pos.cast<float>();
        viz->data->GetLineIndices (info.iframe, info.ilines)[ii+0] = iv+0;
        viz->data->GetLineIndices (info.iframe, info.ilines)[ii+1] = iv+1;
        iv += 2;
        ii += 2;
    }
    lines->numVertices = iv;
    lines->numIndices  = ii;
    info.ilines++;

	const double scale = 0.05;

	// local x axis of each node
	lines = viz->data->GetLines(info.iframe, info.ilines);
    lines->color = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
    lines->alpha = 0.5f;
    lines->width = 1.0f;
    iv = 0;
    ii = 0;
    for(auto n : nodes){
	    viz->data->GetLineVertices(info.iframe, info.ilines)[iv+0] = n->pos.cast<float>();
        viz->data->GetLineVertices(info.iframe, info.ilines)[iv+1] = (n->pos + scale*(n->ori*Vector3d::UnitX())).cast<float>();
        viz->data->GetLineIndices (info.iframe, info.ilines)[ii+0] = iv+0;
        viz->data->GetLineIndices (info.iframe, info.ilines)[ii+1] = iv+1;
        iv += 2;
        ii += 2;
    }
    lines->numVertices = iv;
    lines->numIndices  = ii;
    info.ilines++;

	// local y axis of each node
	lines = viz->data->GetLines(info.iframe, info.ilines);
    lines->color = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    lines->alpha = 0.5f;
    lines->width = 1.0f;
    iv = 0;
    ii = 0;
    for(auto n : nodes){
	    viz->data->GetLineVertices(info.iframe, info.ilines)[iv+0] = n->pos.cast<float>();
        viz->data->GetLineVertices(info.iframe, info.ilines)[iv+1] = (n->pos + scale*(n->ori*Vector3d::UnitY())).cast<float>();
        viz->data->GetLineIndices (info.iframe, info.ilines)[ii+0] = iv+0;
        viz->data->GetLineIndices (info.iframe, info.ilines)[ii+1] = iv+1;
        iv += 2;
        ii += 2;
    }
    lines->numVertices = iv;
    lines->numIndices  = ii;
    info.ilines++;

	// local z axis of each node
	lines = viz->data->GetLines(info.iframe, info.ilines);
    lines->color = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    lines->alpha = 0.5f;
    lines->width = 1.0f;
    iv = 0;
    ii = 0;
    for(auto n : nodes){
	    viz->data->GetLineVertices(info.iframe, info.ilines)[iv+0] = n->pos.cast<float>();
        viz->data->GetLineVertices(info.iframe, info.ilines)[iv+1] = (n->pos + scale*(n->ori*Vector3d::UnitZ())).cast<float>();
        viz->data->GetLineIndices (info.iframe, info.ilines)[ii+0] = iv+0;
        viz->data->GetLineIndices (info.iframe, info.ilines)[ii+1] = iv+1;
        iv += 2;
        ii += 2;
    }
    lines->numVertices = iv;
    lines->numIndices  = ii;
    info.ilines++;
}

}
}
