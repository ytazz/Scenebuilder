#pragma once

#include <sbmodel.h>
#include <sbmesh.h>
#include <sbtokenizer.h>

namespace Scenebuilder{;

class LoaderSTL : public ModelLoader{
public:
	struct Facet{
		Vec3f	 normal;
		Vec3f	 vertices[3];
		uint16_t attr;    ///< binary STL 付加属性値
		Vec4f    color;   ///< binary STL 付加属性値に含まれる色情報
	};

	bool            useColor;
	bool            binary;
	vector<Facet>	facets;

	string		    name;
				    
	Tokenizer	    rowTok;
	Tokenizer	    colTok;
				    
	bool		    facetOpen;
	int			    vertexIdx;

protected:
	void Assert     (bool eval);
	void Parse      ();
	void ParseBinary();

public:
	void Clear();

	virtual void Load   (const string& filename);
	virtual void Save   (const string& filename);
	virtual void Convert(Model* model, const Affinef& aff);

	LoaderSTL(bool _useColor, bool _binary);

};

}
