#pragma once

#include <sbmodel.h>
#include <sbmesh.h>
#include <sbtokenizer.h>

/**
 pointcloud library PCD file format
 */

namespace Scenebuilder{;

class LoaderPCD : public ModelLoader{
public:
	Tokenizer	    rowTok;
	Tokenizer	    colTok;

	struct Point{
		Vec3f  pos;
		Vec3f  normal;
	};

	vector<Point>  points;
				    
protected:
	bool ParseBinary();

public:
	void Clear();

	virtual void Load   (const string& filename);
	virtual void Convert(Model* model, const Affinef& aff);

	LoaderPCD();

};

}
