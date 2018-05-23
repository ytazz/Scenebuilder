#pragma once

#include <sbtypes.h>
#include <sbtokenizer.h>

namespace Scenebuilder{;

/** Mesh
   - 3Dモデルの描画用形式．特定の処理系には依存しない
 */
class Mesh{
public:
	enum{
		Triangles,
		TriangleStrip,
		TriangleFan,
	};

	struct Vtx{
		Vec3f	pos;
		Vec3f	normal;
		Vec4f	color;
		Vec4f   color2;
		Vec2f	texcoord;
		Vec2i	boneIndex;
		float	boneWeight;

		Vtx(){
			boneIndex  = Vec2i(-1, -1);
			boneWeight = 1.0f;
		}
	};

public:
	uint    type;			///< current primitive type
	Vec3f	curNormal;		///< current normal
	Vec2f	curTexCoord;	///< current texCoord
	Vec4f	curColor;		///< current color
	Vec4f   curColor2;
	Vec2i	curBoneIndex;
	float	curBoneWeight;
	
	bool	front;			///< create front face
	bool	back;			///< create back face
	bool	solid;			///< create solid mesh or wire frame
	bool	tile;			///< tile color mode
	Vec2f   texture_scale;
	Vec2f	texture_offset;
	Affinef	aff;			///< affine transformation applied to position and normal

	vector<Vtx>	vtx;	///< temporary vertices before creating primitives

public:
	/* 頂点配列
	   - そのままVAOを作るのに使える
	   - 3つずつで三角形を張る
	   - front face is CCW
	 */
	vector<Vec3f>	positions;
	vector<Vec3f>	normals;
	vector<Vec4f>   colors;
	vector<Vec2f>	texcoords;
	vector<Vec2i>	boneIndices;
	vector<float>	boneWeights;

	Vec3f			bbmin;
	Vec3f			bbmax;

	void  CreateCircle   (vector<Vec2f>& points, uint slice);
	void  CreateTriangle (Vtx& v0, Vtx& v1, Vtx& v2, bool _2nd);
	void  CreatePrimitive();
	Vec2f MapTexCoord    (const Vec3f& p, const Vec3f& n, int style);
	
public:
	void Clear();

	void Begin(int type);
	void End();

	void Normal    (float x, float y, float z);
	void Normal    (const Vec3f& n);
	void TexCoord  (float x, float y);
	void TexCoord  (const Vec2f& t);
	void Color     (float r, float g, float b, float a);
	void Color     (const Vec4f& c);
	void Color2    (float r, float g, float b, float a);
	void Color2    (const Vec4f& c);
	void BoneIndex (int i0, int i1);
	void BoneWeight(float w);

	void Vertex  (float x, float y, float z);
	void Vertex  (const Vec3f& v);

	void GenerateNormals  ();
	void GenerateTexCoords(int style);
	void CalcBBox         ();

	void Box     (float sx, float sy, float sz, uint divx = 1, uint divy = 1, uint divz = 1);
	void Sphere  (float radius, uint slice, uint stack, bool upper = true, bool lower = true, float offset = 0.0f);
	void Cylinder(float radius, float length, uint slice, bool cap = true);
	void Capsule (float radius, float length, uint slice, uint stack);

	Mesh();
};

/* OpenGL Vertex Array Object (VAO)
	- Meshから変換して描画に用いる
 */
class VertexArray{
public:
	/// シェーダ上の頂点属性インデックス（nVidia / Springhead仕様に準拠）
	struct Attribute{
		enum{
			/* 組み込み属性のPosition ～ MultiTexCoordは参考のみ */
			Position       = 0,
			Normal         = 2,
			Color          = 3,
			SecondaryColor = 4,
			FogCoord       = 5,
			MultiTexCoord0 = 8,
			/// 以下Springhead拡張属性
			BlendIndex     = 11,
			BlendWeight    = 12,
			Count,
		};
	};

	struct Primitive{
		enum{
			Points,
			Lines,
			Triangles,
		};
	};

	Mesh* mesh;

	bool usePos;
	bool useNormal;
	bool useColor;
	bool useTexcoord;
	bool useBone;
	bool useVao;

	uint vboId[6];	///< VBO for position, normal, color, texcoord, blend index, blend weight
	uint vaoId;		///< vertex array object id (OpenGL 3.0+ required)

	uint  count;		///< number of vertices in array
	float pointSize;
	float lineWidth;

public:
	/// VBOをポインタにマップ（直接編集用）
	Vec3f*	MapPositionArray  ();
	Vec3f*  MapNormalArray    ();
	Vec4f*  MapColorArray     ();
	Vec2f*  MapTexCoordArray  ();
	Vec2i*  MapBoneIndexArray ();
	float*  MapBoneWeightArray();

	/// アンマップ
	void    UnmapPositionArray  ();
	void    UnmapNormalArray    ();
	void    UnmapColorArray     ();
	void    UnmapTexCoordArray  ();
	void    UnmapBoneIndexArray ();
	void    UnmapBoneWeightArray();

	/// 手動で作成
	void Create(uint count, bool p = true, bool n = false, bool c = false, bool t = false, bool b = false, bool vao = false, bool dynamic = false);

	/// メッシュから作成
	void Create(Mesh* mesh, bool vao, bool dynamic);
	
	void Delete();
	
	void Draw  (int type);
		
	 VertexArray();
	~VertexArray();
};

}
