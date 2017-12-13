#ifndef SB_DIMP_H
#define SB_DIMP_H

#include <sbadaptor.h>

#include <DiMP/DiMP.h>

#include <sstream>
#include <map>
using namespace std;

namespace Scenebuilder{;

/** AdaptorDiMP
	- 動作計画ライブライDiMP用のアダプタ
 **/

class AdaptorDiMP : public Adaptor{
protected:
	struct BodyAux : Aux{
		DiMP::Object*	obj;
		BodyAux(DiMP::Object* o):obj(o){}
	};
	struct ConnectorAux : Aux{
		BodyAux*	      body;
		DiMP::Connector* con;
		ConnectorAux(BodyAux* b, DiMP::Connector* c):body(b), con(c){}
	};
	struct ShapeAux : Aux{
		vector<DiMP::Geometry*>  geos;
		string  filename;

		ShapeAux(){}
		ShapeAux(DiMP::Geometry* g){ geos.push_back(g); }
	};
	struct AttachAux : Aux{
		ShapeAux*		shape;
		ConnectorAux*	con;
		AttachAux(ShapeAux* s, ConnectorAux* c):shape(s), con(c){}
	};
	struct JointAux : Aux{
		DiMP::Joint*	joint;
		ConnectorAux*	sock;
		ConnectorAux*	plug;
		JointAux(DiMP::Joint* j, ConnectorAux* s, ConnectorAux* p):joint(j), sock(s), plug(p){}
	};
	
	DiMP::Graph*	graph;
	//real_t			syncTime;
	
public:
	void            Set        (DiMP::Graph*  g);
	//void            SetSyncTime(real_t time);
	DiMP::Object*   GetObject  (int    id  );
	DiMP::Object*   GetObject  (string name);
	DiMP::Joint*    GetJoint   (int    id  );
	DiMP::Joint*    GetJoint   (string name);
	
	std::pair<DiMP::Geometry*, size_t> GetGeometry(int    id  );
	std::pair<DiMP::Geometry*, size_t> GetGeometry(string name);
	
	virtual int		CreateObject(int id);
	virtual void	DeleteObject(int id);
	virtual void	SyncObjectProperty(int id, bool download, int cat);

	
	AdaptorDiMP();
};

}
#endif
