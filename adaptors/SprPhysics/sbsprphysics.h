#pragma once

#include <sbadaptor.h>
#include <sbmodel.h>

#include <SprPhysics.h>
using namespace Spr;

/** Springhead Physics adaptor
	[functions]
	- physics simulation
	- renders collision detection geometries and physical states

	- 制限
		- 同じShapeに異なる複数のPhysicalMaterialを関連付けてAttachすることはできない
		  ＊Springheadでは形状（CDShape）自体が物性を持つため
 **/
extern "C"{
typedef void (*SetParamFunc              )(void* jnt, const char* name, double value);
typedef bool (*IsCyclicFunc              )(void* jnt);
typedef void (*GetMovableAxesFunc        )(void* jnt, int* n, int* indices);
typedef void (*CompBiasFunc              )(void* jnt, double* dbv , double* dbw , const double* prel, const double* qrel, const double* vrel, const double* wrel);
typedef void (*CompErrorFunc             )(void* jnt, double* Bv  , double* Bw  , const double* prel, const double* qrel                                        );
typedef void (*UpdateJointStateFunc      )(void* jnt, double* pos , double* vel , const double* prel, const double* qrel, const double* vrel, const double* wrel);
typedef void (*CompJointJacobianFunc     )(void* jnt, double* Jv  , double* Jw  , double pos            );
typedef void (*CompJointCoriolisAccelFunc)(void* jnt, double* cv  , double* cw  , double pos, double vel);
typedef void (*CompRelativePositionFunc  )(void* jnt, double* prel, double* qrel, double pos            );
typedef void (*CompRelativeVelocityFunc  )(void* jnt, double* vrel, double* wrel, double pos, double vel);
}

namespace Scenebuilder{;

class ModelSTL;

class AdaptorSprPH : public Adaptor{
protected:
	struct GenericJointCallback : public PHGenericJointCallback{
		void*			           hDll;
		SetParamFunc               hSetParam;
		IsCyclicFunc               hIsCyclic;
		GetMovableAxesFunc         hGetMovableAxes;
		CompBiasFunc               hCompBias;
		CompErrorFunc              hCompError;
		UpdateJointStateFunc       hUpdateJointState;
		CompJointJacobianFunc      hCompJointJacobian;
		CompJointCoriolisAccelFunc hCompJointCoriolisAccel;
		CompRelativePositionFunc   hCompRelativePosition;
		CompRelativeVelocityFunc   hCompRelativeVelocity;

		void Init();

		virtual void SetParam              (PHGenericJointIf* jnt, const char* name, double value);
		virtual bool IsCyclic              (PHGenericJointIf* jnt);
		virtual void GetMovableAxes        (PHGenericJointIf* jnt, int& n, int* indices);
		virtual void CompBias              (PHGenericJointIf* jnt, Vec3d&  dbv, Vec3d&  dbw, const Vec3d& prel, const Quaterniond& qrel, const Vec3d& vrel, const Vec3d& wrel);
		virtual void CompError             (PHGenericJointIf* jnt, Vec3d&  Bv , Vec3d&  Bw , const Vec3d& prel, const Quaterniond& qrel                                      );
		virtual void UpdateJointState      (PHGenericJointIf* jnt, double& pos, double& vel, const Vec3d& prel, const Quaterniond& qrel, const Vec3d& vrel, const Vec3d& wrel);
		virtual void CompJointJacobian     (PHGenericJointIf* jnt, Vec3d& Jv  , Vec3d&       Jw  , double pos            );
		virtual void CompJointCoriolisAccel(PHGenericJointIf* jnt, Vec3d& cv  , Vec3d&       cw  , double pos, double vel);
		virtual void CompRelativePosition  (PHGenericJointIf* jnt, Vec3d& prel, Quaterniond& qrel, double pos            );
		virtual void CompRelativeVelocity  (PHGenericJointIf* jnt, Vec3d& vrel, Vec3d&       wrel, double pos, double vel);

		 GenericJointCallback();
		~GenericJointCallback();
	};

	struct ConnectorAux;
	struct AttachAux;
	struct JointAux;
	struct ContactGroupAux;

	struct BodyAux : Aux{
		PHSolidIf*					solid;
		PHTreeNodeIf*				treeNode;
		bool						auto_mass;
		bool						auto_tree;
		vector<ConnectorAux*>		cons;		///< back reference to Attach
		vector<ContactGroupAux*>	groups;		///< back reference to ContactGroup

		BodyAux(PHSolidIf* s);
		virtual ~BodyAux();
	};
	struct ConnectorAux : Aux{
		BodyAux*	body;
		Posed		pose;		///< Bodyとの相対的な位置と向き．Connectorの直上の親がBodyではない場合も考慮
		vector<JointAux*>		joints;		///< back reference to Joint
		vector<AttachAux*>		attaches;	///< back reference to Attach

		ConnectorAux(BodyAux* b);
		virtual ~ConnectorAux();
	};
	struct MaterialAux : Aux{
		vector<AttachAux*>	attaches;
		PHMaterial	mat;				///< Springhead 物性
		
		MaterialAux& operator=(PhysicalMaterialProp& prop);
		virtual ~MaterialAux();
	};
	struct ShapeAux : Aux{
		vector<AttachAux*>	attaches;		///< back reference to Attach
		vector<CDShapeIf*>	shapes;

		ShapeAux();
		~ShapeAux();
	};
	struct AttachAux : Aux{
		ShapeAux*		shape;		///< reference to Shape
		ConnectorAux*	con;		///< reference to Connector
		MaterialAux*	mat;
		int				idxBegin;
		int				idxEnd;
		
		void OnChange(Aux* caller);
		AttachAux(ShapeAux* s, ConnectorAux* c, MaterialAux* m, int ib, int ie);
		virtual ~AttachAux();
	};
	struct JointAux : Aux{
		PHJointIf*		joint;
		ConnectorAux*	sock;		///< reference to socket Connector
		ConnectorAux*	plug;		///< reference to plug Connector
		
		GenericJointCallback	callback;

		void OnChange(Aux* caller);
		JointAux(PHJointIf* j, ConnectorAux* s, ConnectorAux* p);
		virtual ~JointAux();
	};
	struct GearAux : Aux{
		PHGearIf*		gear;
		GearAux(PHGearIf* g);
	};
	struct ContactGroupAux : Aux{
		vector<BodyAux*>	bodies;		///< reference to Body
		virtual ~ContactGroupAux();
	};

protected:
	PHSdkIf*		phSdk;
	PHSceneIf*		phScene;

	vector<BodyAux *>	bodies;
	vector<JointAux*>	joints;
	vector<GearAux *>   gears;
	bool				treeReady;

	vector<string>		dllSearchPaths;

protected:
	void ConvertMesh(vector<CDShapeIf*>& sh, Model* model, MeshProp* meshProp);
	
public:
	virtual int		CreateObject(int id);
	virtual void	DeleteObject(int id);
	virtual void	SyncObjectProperty(int id, bool download, int cat);

	/// generic joint用DLL検索パスを追加
	void AddDllSearchPath(const string& str);

	/// auto_treeの剛体にPHTreeNodeを作成する
	void CreateTreeNodes();

	using Adaptor::Set;
	void Set(PHSdkIf* _phSdk, PHSceneIf* _phScene);
	PHSolidIf*	GetSolid(int    id  );
	PHSolidIf*	GetSolid(string name);
	PHJointIf*	GetJoint(int    id  );
	PHJointIf*	GetJoint(string name);
	CDShapeIf**	GetShape(int    id  );
	CDShapeIf**	GetShape(string name);

	void Step();

	AdaptorSprPH();
};

}
