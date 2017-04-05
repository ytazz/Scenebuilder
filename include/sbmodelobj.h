#pragma once

#include <sbmodel.h>
#include <sbmesh.h>
#include <sbtokenizer.h>

namespace Scenebuilder{;

/*
 *	Wavefront OBJ format parser
 */

class LoaderOBJ : public ModelLoader{
public:
	struct Face{
		uint v[3];	///< vertex index
		uint t[3];	///< tex.coord index
		uint n[3];  ///< normal index
	};
	
	vector<Vec3f>	vertices;
	vector<Vec2f>	texCoords;
	vector<Vec3f>	normals;
	vector<Face>	faces;

	Tokenizer		rowTok;
	Tokenizer		colTok;

	void Parse();

public:
	void Clear();

	virtual void Load   (const string& filename);
	virtual void Convert(Model* model, const Affinef& aff);

	LoaderOBJ();

};

}
