#pragma once

#define _SCL_SECURE_NO_WARNINGS

#include <string>
#include <vector>
using namespace std;

#include <boost/algorithm/string/split.hpp>
using namespace boost;
using namespace boost::algorithm;

#include <Springhead.h>
using namespace Spr;

class BVH{
public:
	struct SyntaxError{};

	struct Node : UTRefCount{
		string			name;
		Vec3f			offset;
		int				numChannels;
		int				channelOffset;
		int				channels[6];
		Node*			parent;
		vector<Node*>	children;

		void AddChild(Node* n){
			n->parent = this;
			children.push_back(n);
		}
		void PrintRecurs(ostream& os, int depth);

		Node();
	};

	Node*	root;				///< root node
	int		numFrames;			///< number of frames
	int		numChannels;		///< total number of channels
	float	sampleRate;			///< sample rate [sec]

protected:
	enum{
		Xposition,
		Yposition,
		Zposition,
		Xrotation,
		Yrotation,
		Zrotation,
	};
	
	string contents;			///< contents string

	typedef split_iterator<string::iterator>	string_split_iterator;
	typedef iterator_range<string::iterator>	string_iterator_range;
	string_split_iterator	splitIt;

	Node*	curNode;
	
	typedef vector< UTRef<Node> >	Nodes;
	Nodes	nodes;

	vector<float>	data;		///< motion data: numChannels * numFrames elements

protected:
	string_iterator_range Next(){
		if(End())
			throw SyntaxError();
		return *splitIt++;
	}

	bool End(){
		return splitIt == string_split_iterator();
	}

	bool Compare(string_iterator_range str, const char* comp){
		return (str.end() - str.begin() == strlen(comp)) && equal(str.begin(), str.end(), comp);
	}

	template<class T>
	void Assert(T state){
		if(!state)
			throw SyntaxError();
	}

	void CreateNode(string name = string());
	void CreateNode(string_iterator_range str){
		CreateNode(string(str.begin(), str.end()));
	}
	void ParseData();
	void Parse();
	
public:
	void Clear();
	bool Load(const char* filename);
	void Print(ostream& os);

	/**	@brief	calculates local transformation of a node at specified time instant
		@param	aff		local transformation w.r.t. parent of the node
		@param	node	node
		@param	time	time
	 */
	void GetPose(Affinef& aff, Node* node, float time);

	void DrawNode(GRRenderIf* render, Node* node, float time);
	void Draw(GRRenderIf* render, float time);

	BVH();
};