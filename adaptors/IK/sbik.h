﻿#pragma once

#include <sbadaptor.h>
#include <sbiksolver.h>
#include <sbikbody.h>
#include <sbikjoint.h>
#include <sbikhandle.h>

namespace Scenebuilder{;

/**
    Scenebuilder::IKのアダプタ
 **/
class AdaptorIK : public Adaptor{
public:
	struct ConnectorAux;
	struct AttachAux;
	struct JointAux;
	struct HandleAux;

	struct BodyAux : Aux{
		IKBody*                ikBody;
		vector<ConnectorAux*>  cons;		///< back reference to Attach
		
		BodyAux(IKBody* b);
		virtual ~BodyAux();
	};
	struct ConnectorAux : Aux{
		BodyAux*	body;
		Posed		pose;             ///< Bodyとの相対的な位置と向き．Connectorの直上の親がBodyではない場合も考慮
		vector<JointAux *>  joints;   ///< back reference to JointAux
		vector<HandleAux*>  handles;  ///< back reference to HandleAux
	
		ConnectorAux(BodyAux* b);
		virtual ~ConnectorAux();
	};
	struct JointAux : Aux{
		ConnectorAux*	sock;		///< reference to socket Connector
		ConnectorAux*	plug;		///< reference to plug Connector
		IKJoint*        ikJoint;
		
		void OnChange(Aux* caller);
		JointAux(ConnectorAux* _sock, ConnectorAux* _plug, IKJoint* _joint);
		virtual ~JointAux();
	};
	struct HandleAux : Aux{
		IKHandle*      ikHandle;
		ConnectorAux*  sock;
		
		void OnChange(Aux* caller);
		HandleAux(IKHandle* _handle, ConnectorAux* _sock);
		virtual ~HandleAux();
	};
	struct ComHandleAux : Aux{
		IKComHandle*      ikComHandle;
		vector<BodyAux*>  bodies;
		
		void OnChange(Aux* caller);
		ComHandleAux(IKComHandle* _handle);
		virtual ~ComHandleAux();
	};

public:
	vector<BodyAux     *>	bodies    ;
	vector<JointAux    *>	joints    ;
	vector<HandleAux   *>	handles   ;
	vector<ComHandleAux*>   comHandles;

	IKSolver            ikSolver;

public:

	/// 取得
	IKBody     *  GetBody     (int    id  );
	IKBody     *  GetBody     (string name);
	IKJoint    *  GetJoint    (int    id  );
	IKJoint    *  GetJoint    (string name);
	IKHandle   *  GetHandle   (int    id  );
	IKHandle   *  GetHandle   (string name);
	IKComHandle*  GetComHandle(int    id  );
	IKComHandle*  GetComHandle(string name);
	
	virtual int    CreateObject(int id);
	virtual void   DeleteObject(int id);
	virtual void   SyncObjectProperty(int id, bool download, int cat);

	AdaptorIK();
};

}