#ifndef SB_POVRAY_H
#define SB_POVRAY_H

#include <sbadaptor.h>

/** POVRAY adaptor **/

namespace Scenebuilder{;

class AdaptorPovray : public Adaptor{
public:
	struct ConnectorAux;
	struct AttachAux;

	struct BodyAux : Aux{
		vector<ConnectorAux*>	cons;
		virtual ~BodyAux();
	};
	struct MaterialAux : Aux{
		vector<AttachAux*>	attaches;
		virtual ~MaterialAux();
	};
	struct ShapeAux : Aux{
		vector<AttachAux*>	attaches;
		virtual ~ShapeAux();
	};
	struct BoxAux : ShapeAux{
		Vec3f	size;
	};
	struct SphereAux : ShapeAux{
		float	radius;
	};
	struct CylinderAux : ShapeAux{
		float	radius;
		float	height;
	};
	struct MeshAux : ShapeAux{
	};
	struct LightAux : Aux{
		vector<AttachAux*>	attaches;
		virtual ~LightAux();
	};
	struct CameraAux : Aux{
		vector<AttachAux*>	attaches;
		virtual ~CameraAux();		
	};
	struct ConnectorAux : Aux{
		BodyAux*	body;
		vector<AttachAux*>	attaches;

		ConnectorAux(BodyAux* b);
		virtual ~ConnectorAux();
	};
	struct AttachAux : Aux{
		ShapeAux*		shape;
		LightAux*		light;
		CameraAux*		camera;
		ConnectorAux*	con;
		MaterialAux*	mat;

		virtual void OnChange(Aux* caller);
		AttachAux(ShapeAux* sh, LightAux* li, CameraAux* cam, ConnectorAux* con, MaterialAux* mat);
		virtual ~AttachAux();
	};
		
	vector<BodyAux*>		bodies;
	vector<AttachAux*>		attaches;

public:

	virtual int		CreateObject(int id);
	virtual void	DeleteObject(int id);
	virtual void	SyncObjectProperty(int id, bool download, int cat);

	/// ファイルに出力
	void WriteHeader(ostream& os);
	void Write(ostream& os);
	
	AdaptorPovray();
};

}
#endif
