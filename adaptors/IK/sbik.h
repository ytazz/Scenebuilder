#pragma once

#include <sbadaptor.h>
#include <sbiksolver.h>
#include <sbikbody.h>
#include <sbikjoint.h>
#include <sbikbodyhandle.h>
#include <sbikjointhandle.h>
#include <sbikjointsync.h>
#include <sbikcomhandle.h>

namespace Scenebuilder{;

/**
    Scenebuilder::IKのアダプタ
 **/
class AdaptorIK : public Adaptor{
public:
	struct ConnectorAux;
	struct AttachAux;
	struct JointAux;
	struct BodyHandleAux;

	struct BodyAux : Aux{
		IKBody*                ikBody;
		vector<ConnectorAux*>  cons;		///< back reference to Attach
		
		BodyAux(IKBody* b);
		virtual ~BodyAux();
	};
	struct ConnectorAux : Aux{
		BodyAux*	body;
		Posed		pose;             ///< Bodyとの相対的な位置と向き．Connectorの直上の親がBodyではない場合も考慮
		vector<JointAux *>      joints;   ///< back reference to JointAux
		vector<BodyHandleAux*>  handles;  ///< back reference to HandleAux
	
		ConnectorAux(BodyAux* b);
		virtual ~ConnectorAux();
	};
	struct JointAux : Aux{
		ConnectorAux*	sock;		///< reference to socket Connector
		ConnectorAux*	plug;		///< reference to plug Connector
		IKJointBase *   ikJoint;
		
		void OnChange(Aux* caller);
		JointAux(ConnectorAux* _sock, ConnectorAux* _plug, IKJointBase* _joint);
		virtual ~JointAux();
	};
	struct BodyHandleAux : Aux{
		IKBodyHandle*  ikBodyHandle;
		ConnectorAux*  sock;
		
		void OnChange(Aux* caller);
		BodyHandleAux(IKBodyHandle* _handle, ConnectorAux* _sock);
		virtual ~BodyHandleAux();
	};
	struct JointHandleAux : Aux{
		IKJointHandle*      ikJointHandle;
		
		JointHandleAux(IKJointHandle* _handle);
		virtual ~JointHandleAux();
	};
	struct JointSyncAux : Aux{
		IKJointSync*      ikJointSync;
		
		JointSyncAux(IKJointSync* _sync);
		virtual ~JointSyncAux();
	};
	struct ComHandleAux : Aux{
		IKComHandle*      ikComHandle;
		vector<BodyAux*>  bodies;
		
		void OnChange(Aux* caller);
		ComHandleAux(IKComHandle* _handle);
		virtual ~ComHandleAux();
	};

public:
	vector<BodyAux       *>	bodies      ;
	vector<JointAux      *>	joints      ;
	vector<BodyHandleAux *>	bodyHandles ;
	vector<JointHandleAux*>	jointHandles;
	vector<JointSyncAux  *>	jointSyncs  ;
	vector<ComHandleAux  *> comHandles  ;

	IKSolver            ikSolver;

public:

	/// 取得
	IKBody       *  GetBody       (int    id  );
	IKBody       *  GetBody       (string name);
	IKJointBase  *  GetJoint      (int    id  );
	IKJointBase  *  GetJoint      (string name);
	IKBodyHandle *  GetBodyHandle (int    id  );
	IKBodyHandle *  GetBodyHandle (string name);
	IKJointHandle*  GetJointHandle(int    id  );
	IKJointHandle*  GetJointHandle(string name);
	IKJointSync  *  GetJointSync  (int    id  );
	IKJointSync  *  GetJointSync  (string name);
	IKComHandle  *  GetComHandle  (int    id  );
	IKComHandle  *  GetComHandle  (string name);
	
	virtual int    CreateObject(int id);
	virtual void   DeleteObject(int id);
	virtual void   SyncObjectProperty(int id, bool download, int cat);

	AdaptorIK();
};

}
