#pragma once

#include <sbadaptor.h>
#include <sbmesh.h>
#include <sbmodel.h>

#include <Springhead.h>
using namespace Spr;

/** Springhead Graphics adaptor **/

namespace Scenebuilder{;

class Mesh;

class AdaptorSprGR : public Adaptor{
public:
	// 描画オプション
	struct DrawOption{
		bool            useVao;         ///< VAOを使用(OpenGL 3.3+)

		bool			drawSolid;		///< ソリッド描画するか
		bool			drawWire;		///< ワイヤフレーム描画するか
		bool            drawPoints;     ///< 点群描画するか
	
		bool			drawAxis;		///< BodyとConnectorの座標軸を描画するか
		Vec4f			axisColorX;		///< x軸の色
		Vec4f			axisColorY;		///< y軸の色
		Vec4f			axisColorZ;		///< z軸の色
	
		bool			drawCoM;		///< Bodyの重心位置を描画するか
		Vec4f			comColor;		///< 重心の色
	
		bool			drawBBox;		///< bboxを描画するか
		Vec4f			bboxColor;		///< bboxの色

		DrawOption();
	};

	struct ConnectorAux;
	struct AttachAux;
	struct ShapeAux;

	struct BodyAux : Aux{
		Affinef	aff;
		Affinef	affIni;
		bool	initial;
		float	bbradius;
		
		vector<ConnectorAux*>	cons;
		
		void CalcBound();
		BodyAux();
		virtual ~BodyAux();
	};
	struct MaterialAux : Aux{
		vector<AttachAux*>	attaches;
		GRMaterialDesc	mat;			///< Springheadマテリアル
		string	        colorname;		///< 色名
		Vec4f		    color;			///< 色RGB
		float		    alpha;			///< 透過度
		
		void Set(GRRenderIf* render);	///< マテリアルの適用
		
		MaterialAux();
		virtual ~MaterialAux();
	};
	
	struct ShapeAux : Aux{
		vector<AttachAux*>	attaches;	///< reference to attaches
		Vec3f	bbcenter;				///< center of bbox
		Vec3f	bbsize;					///< size of bounding box
		float	bbradius;				///< radius of bounding sphere
		int     auto_texcoord;
		Vec2f   texture_scale;
		Vec2f	texture_offset;
		
		virtual void CalcBound() = 0;
		virtual void Draw     (GRRenderIf* render, const DrawOption& opt) = 0;
		        void DrawBBox (GRRenderIf* render, const DrawOption& opt);
		virtual int  GetBoneIndex(const string& bonename){ return -1; }

		ShapeAux();
		virtual ~ShapeAux();
	};
	struct PrimitiveShapeAux : ShapeAux{
		Mesh		meshSolid;
		Mesh		meshWire;
		Mesh        meshPoints;
		VertexArray	arraySolid;
		VertexArray	arrayWire;
		VertexArray	arrayPoints;

		virtual void Create   (const DrawOption& opt);
		virtual void Draw     (GRRenderIf* render, const DrawOption& opt);
	};
	struct BoxAux : PrimitiveShapeAux{
		Vec3f	size;
		
		virtual void Create   (const DrawOption& opt);
		virtual void CalcBound();
		BoxAux();
	};
	struct SphereAux : PrimitiveShapeAux{
		float	radius;
		uint    slice;
		uint    stack;
		
		virtual void Create   (const DrawOption& opt);
		virtual void CalcBound();
		SphereAux();
	};
	struct CylinderAux : PrimitiveShapeAux{
		float	radius;
		float	height;
		uint    slice;

		virtual void Create   (const DrawOption& opt);
		virtual void CalcBound();
		CylinderAux();
	};
	struct CapsuleAux : PrimitiveShapeAux{
		float	radius;
		float	height;
		uint    slice;
		uint    stack;

		virtual void Create   (const DrawOption& opt);
		virtual void CalcBound();
		CapsuleAux();
	};
	struct MeshAux : ShapeAux{
		string  filename;
		Model*	model;
		vector<GRMaterialDesc>	materials;		///< Materialから変換したGRMaterialDesc
		vector<VertexArray>		arraySolid;
		vector<VertexArray>		arrayWire;
		vector<VertexArray>		arrayPoints;
		
		virtual void Create   (const DrawOption& opt);
		virtual void CalcBound();
		virtual void Draw     (GRRenderIf* render, const DrawOption& opt);
		        void DrawBoned(GRRenderIf* render, const DrawOption& opt);
		virtual int  GetBoneIndex(const string& bonename);
		MeshAux();
	};
	struct LightAux : Aux{
		vector<AttachAux*>	attaches;
		LightAux();
		~LightAux();
	};
	struct CameraAux : Aux{
		vector<AttachAux*>	attaches;
		CameraAux();
		~CameraAux();		
	};
	struct ConnectorAux : Aux{
		BodyAux*	body;
		vector<AttachAux*>	attaches;

		Affinef		aff;
		Affinef		affIni;
		bool		initial;
	
		ConnectorAux(BodyAux* b);
		virtual ~ConnectorAux();
	};
	struct AttachAux : Aux{
		ShapeAux*		shape;
		LightAux*		light;
		CameraAux*		camera;
		ConnectorAux*	con;
		MaterialAux*	mat;
		string			bone;			///< ボーン名（MeshへのAttachの場合）
		int				boneIndex;		///< ボーンインデックス

		void Draw(GRRenderIf* render, const DrawOption& opt);

		AttachAux(ShapeAux* sh, LightAux* li, CameraAux* cam, ConnectorAux* con, MaterialAux* mat);
		virtual ~AttachAux();
	};
		
	vector<BodyAux*>	bodies;
	vector<AttachAux*>	attaches;
	vector<MeshAux*>	meshes;
	float				sceneBound;

	DrawOption			opt;

	void ConvertMaterial(GRMaterialDesc& grMat, const Material& mat, int id);
	
public:

	virtual int		CreateObject(int id);
	virtual void	DeleteObject(int id);
	virtual void	SyncObjectProperty(int id, bool download, int cat);
	
	/** select camera used for rendering
		- id		CameraへリンクしているAttachのID
		視点変換を外側で設定する場合はid = -1とする
	 */
	void SelectCamera(int id);

	/// draw all
	void Draw(GRRenderIf* render);

	/// draw coordinate axis
	void DrawAxis(GRRenderIf* render, float length);

	void EnableVao    (bool on = true);

	void ShowSolid     (bool on = true);
	void ShowWireframe (bool on = true);
	void ShowPointcloud(bool on = true);

	void ShowAxis    (bool on = true);
	void SetAxisColor(const char* x, const char* y, const char* z);
	
	void ShowCenterOfMass    (bool on = true);
	void SetCenterOfMassColor(const char* c);
	
	void ShowBBox    (bool on = true);
	void SetBBoxColor(const char* c);

	/** get global pose
		@param id	id of Body, Attach or Connector
	 */
	Affinef	GetBodyPose(int id);

	/// 大きさを計算・取得
	void	CalcBound();
	float	GetSceneBound();
	float	GetBodyBound(int id);
	
	AdaptorSprGR();
};

}
