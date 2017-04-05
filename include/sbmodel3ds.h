#pragma once

#include <sbmodel.h>
#include <sbmesh.h>

#include <stack>
#include <vector>
using namespace std;

namespace Scenebuilder{;

class Loader3DS : public ModelLoader{
public:
	struct ChunkInfo{
		enum{
			Unknown = -1,
			Main				= 0x4d4d,
			Version				= 0x0002,
			Editor				= 0x3d3d,
			MeshVersion			= 0x3d3e,
			MaterialBlock		= 0xafff,
			MaterialName		= 0xa000,
			MaterialAmbient		= 0xa010,
			MaterialDiffuse		= 0xa020,
			MaterialSpecular	= 0xa030,
			MaterialShininess	= 0xa040,
			MaterialStrength	= 0xa041,
			RGBByte				= 0x0011,
			ObjectBlock			= 0x4000,
			TriangleMesh		= 0x4100,
			VertexList			= 0x4110,
			FaceDescription		= 0x4120,
			FaceMaterialList	= 0x4130,
		};

		int		id;
		string	name;
		int		parent;
	};
	
	struct Material{
		string	name;
		Vec3f	ambient;
		Vec3f	diffuse;
		Vec3f	specular;
		float	shininess;
		float	strength;
	};
	struct Face{
		unsigned short	vertices[3];
		unsigned short	flag;
	};
	struct FaceMaterial{
		string					name;		///< material name
		int						index;		///< material index
		vector<unsigned short>	faces;		///< face indices to apply this material
	};
	struct Object : UTRefCount{
		string					name;		///< object name
		vector<Vec3f>			vertices;
		vector<Face>			faces;
		vector<FaceMaterial>	materialList;
	};

	int				version;		///< 3DS format version
	int				meshVersion;	///< mesh version
	
	vector<Material>			materials;
	vector< UTRef<Object> >		objects;

protected:
	vector<ChunkInfo>	chunkInfo;

	stack<ChunkInfo*>	chunkStack;		///< stack of chunks
	Material*			curMat;			///< currently read material
	Object*				curObj;
	Vec3f*				curColor;		///< currently read color
	float*				curFloat;

protected:
	ChunkInfo*	GetChunkInfo(int id);
	void DefChunk(int id, string name, int par = -1);

	void ReadChunks();
	void ParseBinary();

public:
	/// clear
	void Clear();

	/// load from file
	virtual void Load   (const string& filename);
	virtual void Convert(Model* model, const Affinef& aff);

	Loader3DS();

};

}
