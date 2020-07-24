#pragma once

#include <sbtypes.h>
#include <sbmesh.h>
#include <sbimage.h>
#include <sbscene.h>

#include <vector>
using namespace std;

namespace Scenebuilder{;

class SceneBase;

/* マテリアル
   - マテリアルの共通形式
 */
struct Material{
	Vec4f	ambient;
	Vec4f	diffuse;
	Vec4f	specular;
	Vec4f	emissive;
	float	shininess;
	string	texture;

	Material();
};

/* ボーン
 */
struct Bone{
	string			name;		///< ボーン名
	Bone*			parent;		///< 親ボーン
	vector<Bone*>	children;	///< 子ボーン
	Affinef			affIni;		///< 初期グローバル変換
	Affinef			affLocal;	///< 親ボーンとのローカル変換
	Affinef			aff;		///< 現在のグローバル変換
	Affinef			affRel;		///< 初期からの相対変換
	bool			terminal;	///< 

	void SetParent          (Bone* par);
	void SetTransform       (const Affinef& a);
	void CalcTransformRecurs();

	Bone();
};

/// ボーン用変換行列のパレット
/// - 先頭要素は単位変換とする
struct BonePalette : vector<int>{
	void Clear();
	int  Find (int bone);			///< ボーン番号のパレット上の位置．未登録で空きがあれば追加，なければ-1を返す
	
	BonePalette(size_t n):vector<int>(n, -1){}
};

/* モデル
   - 3Dモデルの共通形式
   - 描画と衝突判定の両方に利用される
 */
class Model : public UTRefCount{
public:
	int					id;				///< ロード元MeshのオブジェクトID
	string				filename;		///< モデルのパス

	vector<Mesh>		meshSolid;		///< メッシュ
	vector<Mesh>		meshWire;
	vector<Mesh>        meshPoints;
	vector<Material>	materials;		///< マテリアル
	vector<int>         materialList;	///< マテリアルリスト  メッシュ毎のマテリアルインデックス
	vector<Bone>		bones;			///< ボーン
	vector<BonePalette>	bonePaletteList;
	size_t				bonePaletteSize;

	/// BBox
	Vec3f	bbmin;
	Vec3f	bbmax;
	Vec3f	bbcenter;
	Vec3f	bbsize;
	float	bbradius;

public:
	void  CalcBound        ();
	void  InitBone         ();
	Bone* GetBone          (int i);
	int   GetBoneIndex     (const string& bonename);
	void  CalcBoneTransform();

	Model();
};

/* モデルローダ

 */
class ModelLoader : public UTRefCount{
public:		
	/// ロード用
	string				contents;		///< contents
	byte*				ptr;			///< pointer to current position
	byte*				ptr_end;		///< pointer to contents end

public:
	/// 型を指定して数値を取得
	template<class T>
	T Get(){
		T val = *(T*)ptr;
		ptr += sizeof(T);
		return val;
	}
	template<class T>
	void Get(T* p){
		*p = *(T*)ptr;
		ptr += sizeof(T);
	}

	/// null終端文字列を取得
	void GetCString(string& str){
		size_t len = strlen((char*)ptr);
		str.assign(ptr, ptr+len);
		ptr += len+1;
	}

	/// [文字列長] + [文字列]形式
	void GetNString(int n, string& str){
		str.assign(ptr, ptr+n);
		ptr += n;
	}
	void GetNString(int n, wstring& str){
		str.assign((wchar_t*)ptr, ((wchar_t*)ptr)+n);
		ptr += sizeof(wchar_t)*n;
	}

	/// スキップ
	void Skip(size_t n){
		ptr += n;
	}

	bool IsEnd(){
		return ptr == ptr_end;
	}

public:
	/// ロード
	virtual void Load(const string& filename) = 0;

	/// セーブ
	virtual void Save(const string& filename){}

	/** 独自形式から共通形式へ変換
		@param aff          アフィン変換（スケーリング用）
	 */
	virtual void Convert(Model* model, const Affinef& aff) = 0;
};

/* モデルコンテナ
   - 各種アダプタが利用するためにモデルを格納する
 */
class ModelContainer{
public:
	SceneBase*	scene;

	vector< UTRef<Model> >	models;
	
public:
	void   Set              (SceneBase* s);
	Model* GetModel         (int id);
	bool   LoadModel        (int id, const string& filename, MeshProp* meshProp);
	void   UnloadModel      (int id);
	void   UnloadModelRecurs(int id);
	
	ModelContainer();
};

}
