#pragma once

#include <sbadaptor.h>
#include <sbiksolver.h>
#include <sbikbody.h>
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
		IKBody*                body;
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
		
		void OnChange(Aux* caller);
		JointAux(ConnectorAux* s, ConnectorAux* p);
		virtual ~JointAux();
	};
	struct HandleAux : Aux{
		IKHandle*  handle;
		ConnectorAux*  end;
		ConnectorAux*  ref;

		void OnChange(Aux* caller);
		HandleAux(IKHandle* h, ConnectorAux* e, ConnectorAux* r);
		virtual ~HandleAux();
	};

public:
	vector<BodyAux  *>	bodies ;
	vector<JointAux *>	joints ;
	vector<HandleAux*>	handles;

	IKSolver            ikSolver;

public:

	/// 取得
	IKBody  *  GetBody  (int    id  );
	IKBody  *  GetBody  (string name);
	IKHandle*  GetHandle(int    id  );
	IKHandle*  GetHandle(string name);
	
	virtual int    CreateObject(int id);
	virtual void   DeleteObject(int id);
	virtual void   SyncObjectProperty(int id, bool download, int cat);

	AdaptorIK();
};

}
