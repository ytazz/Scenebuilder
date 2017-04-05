#ifndef SB_MODEL_SPR_H
#define SB_MODEL_SPR_H

#include <sbmesh.h>

namespace Scenebuilder{;

class ModelSPR{
public:
		
public:
	/// load from file
	void Load(string filename);

	/// save to file
	void Save(string filename);

	void Convert(vector<Mesh>& mesh, vector<int>& ml, bool solid, const Affinef& aff);

	ModelSPR();

};

}
#endif
