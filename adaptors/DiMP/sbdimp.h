#ifndef SB_DIMP_H
#define SB_DIMP_H

#include <sbadaptor.h>

#include <DiMP2/DiMP.h>

#include <sstream>
#include <map>
using namespace std;

namespace Scenebuilder{;

/** AdaptorDiMP
	- 動作計画ライブライDiMP2用のアダプタ
 **/

class AdaptorDiMP : public Adaptor{
protected:
	struct BodyAux : Aux{
		DiMP2::Object*	obj;
		BodyAux(DiMP2::Object* o):obj(o){}
	};
	struct ConnectorAux : Aux{
		BodyAux*	      body;
		DiMP2::Connector* con;
		ConnectorAux(BodyAux* b, DiMP2::Connector* c):body(b), con(c){}
	};
	struct ShapeAux : Aux{
		DiMP2::Geometry*  geo;
		ShapeAux(DiMP2::Geometry* g):geo(g){}
	};
	struct AttachAux : Aux{
		ShapeAux*		shape;
		ConnectorAux*	con;
		AttachAux(ShapeAux* s, ConnectorAux* c):shape(s), con(c){}
	};
	struct JointAux : Aux{
		DiMP2::Joint*	joint;
		ConnectorAux*	sock;
		ConnectorAux*	plug;
		JointAux(DiMP2::Joint* j, ConnectorAux* s, ConnectorAux* p):joint(j), sock(s), plug(p){}
	};
	
	DiMP2::Graph*	graph;
	real_t			syncTime;
	
public:
	void             Set        (DiMP2::Graph*  g);
	void             SetSyncTime(real_t time);
	DiMP2::Object*   GetObject  (int    id  );
	DiMP2::Object*   GetObject  (string name);
	DiMP2::Joint*    GetJoint   (int    id  );
	DiMP2::Joint*    GetJoint   (string name);
	DiMP2::Geometry* GetGeometry(int    id  );
	DiMP2::Geometry* GetGeometry(string name);
	
	virtual int		CreateObject(int id);
	virtual void	DeleteObject(int id);
	virtual void	SyncObjectProperty(int id, bool download, int cat);

	
	AdaptorDiMP();
};

}
#endif
