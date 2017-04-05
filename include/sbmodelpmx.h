#pragma once

/** MMDのPMX形式のローダ

PMX構造（仕様書より抜粋）
-先頭-

PMXヘッダ
-
モデル情報
-
頂点数 [int]
頂点 * 頂点数
-
面数 [int]
参照頂点Index * 面数
-
テクスチャ数 [int]
テクスチャパス * テクスチャ数
-
材質数 [int]
材質 * 材質数
-
ボーン数 [int]
ボーン * ボーン数
-
モーフ数 [int]
モーフ * モーフ数
-
表示枠数 [int]
表示枠 * 表示枠数
-
剛体数 [int]
剛体 * 剛体数
-
Joint数 [int]
Joint * Joint数

-終了-

BDEF  -> ブレンドなし
BDEF2 -> 線形ブレンド．アフィン行列を線形補間する．つぶれやすい
SDEF  -> 球面ブレンド．quaternionを線形補間あるいはSLERPする．Cはおそらく回転中心．R0,R1は不明

回転は　Yaw(Y) * Pitch(X) * Roll(Z)

グローバル座標系
X:画面右  Y:画面上  Z:画面手前

ローカル座標系（ボーンなど）
大体下記のようになっている
X:ボーン向き  Y:XZに直交  Z:画面手前

 **/

#include <sbmodel.h>
#include <sbmesh.h>

#include <vector>
using namespace std;

namespace Scenebuilder{;

class LoaderPMX : public ModelLoader{
public:
#pragma pack(push, 1)
	struct Header{
		char	signature[4];	///< "PMX "
		float	version;
		byte	nBytes;			///< 8固定
		byte	encoding;
		byte	additionalUV;
		byte	vertexIndex;
		byte	textureIndex;
		byte	materialIndex;
		byte	boneIndex;
		byte	morphIndex;
		byte	bodyIndex;
	};
#pragma pack(pop)

	struct ModelInfo{
		wstring	modelName;
		wstring	modelNameEn;
		wstring comment;
		wstring	commentEn;
	};
	struct Blending{
		enum{
			BDEF1,
			BDEF2,
			BDEF4,
			SDEF
		};
	};
	struct Vertex{
		Vec3f	pos;
		Vec3f	normal;
		Vec2f	uv;
		// 追加UVは未対応
		// ボーン変形はBDEF2として認識
		int		bone[2];
		float	weight;
		float	edge;
	};
	/// 面　適用する材質ごとに並ぶ
	struct Face{
		int		vertices[3];
	};
	struct Texture{
		string	filename;
	};
	struct Material{
		wstring	name;
		wstring	nameEn;
		Vec4f	diffuse;
		Vec3f	specular;
		float	specularCoef;
		Vec3f	ambient;
		byte	flag;
		Vec4f	edgeColor;
		float	edgeSize;
		int		texture;	///< テクスチャインデックス
		int		textureSphere;
		byte	sphereMode;
		byte	toonFlag;
		int		textureToon;
		wstring	note;
		int		nFaces;		///< 対応する面数	
	};
	struct Bone{
		struct IKLink{
			int		index;
			byte	angleLimit;
			Vec3f	angleMin;
			Vec3f	angleMax;
		};
		enum{
			Connection       = 0x0001,
			EnableRotate     = 0x0002,
			EnableTranslate  = 0x0004,
			Display          = 0x0008,
			EnableManipulate = 0x0010,
			IK               = 0x0020,
			AddRotate        = 0x0100,
			AddTranslate     = 0x0200,
			FixAxis          = 0x0400,
			LocalAxis        = 0x0800,
			Physical         = 0x1000,
			External         = 0x2000,
		};
		wstring	name;
		wstring	nameEn;
		Vec3f	pos;
		int		parent;
		int		hierarchy;
		ushort	flag;
		/// ローカル軸
		Vec3f	localX;
		Vec3f	localZ;
		/// IKボーンの属性
		int		ikTarget;			///< ターゲットボーン
		int		ikIter;				///< IK反復回数
		float	ikAngleDiffLimit;	///< １反復あたりの角度変化量制限
		vector<IKLink>	ikLinks;
	};
	struct Morph{
		enum{
			Group,
			Vertex,
			Bone,
			UV,
			UV1,
			UV2,
			UV3,
			UV4,
			Material,
		};
		wstring  name;
		wstring  nameEn;
		byte     type;
	};
	struct Frame{
		wstring  name;
		wstring  nameEn;
	};
	struct Body{
		enum{
			Sphere,
			Box,
			Capsule,
		};
		enum{
			FollowBone,			///< 対応するボーンに合わせて剛体が動く．衝突回避のために使用
			Physical,			///< 剛体の物理運動に合わせてボーンが動く
			PhysicalBoneMatch,	///< 物理計算の後にボーンの接続関係に合わせて補正
		};
		wstring  name;
		wstring  nameEn;
		int		 bone;			///< 対応ボーン
		byte	 group;			///< 属する衝突グループ
		ushort	 noCollision;	///< 非衝突グループビットマスク（ビットが立っているグループの剛体とは衝突しない）
		byte	 shape;
		Vec3f	 size;			///< 球:size[0] = 半径  箱:size=各方向長さ  カプセル:size[0]=半径,size[1]=長さ
		Vec3f	 pos;
		Vec3f	 rot;
		float	 mass;
		float    trnDecay;
		float    rotDecay;
		float    cor;
		float    friction;
		byte     type;
	};
	struct Joint{
		wstring  name;
		wstring  nameEn;
		int		body[2];
		Vec3f	pos;
		Vec3f	rot;
		Vec3f	posMin;
		Vec3f	posMax;
		Vec3f	rotMin;
		Vec3f	rotMax;
		Vec3f	posSpring;
		Vec3f	rotSpring;
	};

	Header		header;
	ModelInfo	info;

	vector<Vertex>		vertices;
	vector<Face>		faces;
	vector<Texture>		textures;
	vector<Material>	materials;
	vector<Bone>		bones;
	vector<Morph>		morphs;
	vector<Frame>		frames;
	vector<Body>		bodies;
	vector<Joint>		joints;

	void GetIndex       (int sz, int* idx);
	void GetVertexIndex (int* idx);
	void GetTextureIndex(int* idx);
	void GetBoneIndex   (int* idx);
	void GetBodyIndex   (int* idx);
	void ParseBinary    ();

	int  GetFaceMaterial(int f);	///< 面に対応するマテリアルを取得

public:
	void Clear();

	virtual void Load   (const string& filename);
	virtual void Convert(Model* model, const Affinef& aff);

	LoaderPMX();

};

}
