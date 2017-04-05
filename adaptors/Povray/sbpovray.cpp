#include "sbpovray.h"
#include <sbpath.h>
#include <sbmessage.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
using namespace boost;

#define AUTO(T, X, EXP) T X = static_cast<T>(EXP)

namespace Scenebuilder{;

AdaptorPovray::BodyAux::~BodyAux(){
	foreach(ConnectorAux* con, cons)
		con->body = 0;
}

AdaptorPovray::MaterialAux::~MaterialAux(){
	foreach(AttachAux* att, attaches)
		att->mat = 0;
}

AdaptorPovray::ConnectorAux::ConnectorAux(BodyAux* b):body(b){
	body->cons.push_back(this);
}
AdaptorPovray::ConnectorAux::~ConnectorAux(){
	if(body)
		RemoveFromArray(body->cons, this);

	foreach(AttachAux* att, attaches)
		att->con = 0;
}

AdaptorPovray::ShapeAux::~ShapeAux(){
	foreach(AttachAux* att, attaches)
		att->shape = 0;
}

AdaptorPovray::LightAux::~LightAux(){
	foreach(AttachAux* att, attaches)
		att->light = 0;
}

AdaptorPovray::CameraAux::~CameraAux(){
	foreach(AttachAux* att, attaches)
		att->camera = 0;
}

AdaptorPovray::AttachAux::AttachAux(ShapeAux* sh, LightAux* li, CameraAux* cam, ConnectorAux* c, MaterialAux* m):
	shape(sh), light(li), camera(cam), con(c), mat(m){
	if(shape)
		shape->attaches.push_back(this);
	if(light)
		light->attaches.push_back(this);
	if(camera)
		camera->attaches.push_back(this);
	con  ->attaches.push_back(this);
	if(mat)
		mat ->attaches.push_back(this);
}
AdaptorPovray::AttachAux::~AttachAux(){
	if(shape)
		RemoveFromArray(shape->attaches, this);
	if(light)
		RemoveFromArray(light->attaches, this);
	if(camera)
		RemoveFromArray(camera->attaches, this);
	if(con)
		RemoveFromArray(con->attaches, this);
	if(mat)
		RemoveFromArray(mat->attaches, this);
}

//-------------------------------------------------------------------------------------------------

AdaptorPovray::AdaptorPovray(){
}

int AdaptorPovray::CreateObject(int id){
	int type = scene->GetObjectType(id);
	Property* prop = scene->GetProperty(id);
	string name = scene->GetName(id);
	
	if(type == BodyProp::id){
		BodyAux* bodyAux = new BodyAux();
		bodies.push_back(bodyAux);
		RegAux(id, bodyAux);
		Message::Out("created BodyAux for %s", name.c_str());
		return SupportState::Supported;
	}
	else if(type == ConnectorProp::id){
		int bodyId = scene->FindAncestor(id, BodyProp::id, typedb);
		if(!IsValidID(bodyId) || IsIgnored(bodyId))
			return SupportState::Ignored;
		if(IsUndefined(bodyId))
			return SupportState::Undefined;

		AUTO(BodyAux*, bodyAux, GetAux(bodyId));
		
		RegAux(id, new ConnectorAux(bodyAux));
		Message::Out("created ConnectorAux for %s", name.c_str());
		return SupportState::Supported;
	}
	else if(type == VisualMaterialProp::id){
		RegAux(id, new MaterialAux());
		Message::Out("created MaterialAux for %s", name.c_str());
		return SupportState::Supported;
	}
	else if(typedb->KindOf(type, ShapeProp::id)){
		AUTO(ShapeProp*, shapeProp, prop);

		// 視覚化用形状じゃなければ無視
		if(!shapeProp->visual)
			return SupportState::Ignored;

		ShapeAux* shapeAux = 0;

		if(type == BoxProp::id){
			shapeAux = new BoxAux();
			Message::Out("created BoxAux for %s", name.c_str());
		}
		else if(type == SphereProp::id){
			shapeAux = new SphereAux();
			Message::Out("created SphereAux for %s", name.c_str());
		}
		else if(type == CylinderProp::id){
			shapeAux = new CylinderAux();
			Message::Out("created CylinderAux for %s", name.c_str());
		}
		else if(type == MeshProp::id){
			AUTO(MeshProp*, meshProp, prop);
			Path path = Path(scene->GetLocation(id)) + Path(string(meshProp->filename));
		}
		if(shapeAux){
			RegAux(id, shapeAux);
			return SupportState::Supported;
		}
	}
	if(type == AttachProp::id){
		int shapeId = scene->GetLink(id, "shape");
		int conId   = scene->GetLink(id, "connector");
		int matId   = scene->GetLink(id, "vmat");
		
		if(!IsValidID(shapeId) || IsIgnored(shapeId)){
			Message::Error("%s: invalid link to shape", name.c_str());
			return SupportState::Ignored;
		}
		if(!typedb->KindOf(scene->GetObjectType(shapeId), ShapeProp::id)){
			Message::Error("%s: shape must link to Shape", name.c_str());
			return SupportState::Ignored;
		}
		if(!IsValidID(conId) || IsIgnored(conId)){
			Message::Error("%s: invalid link to connector", name.c_str());
			return SupportState::Ignored;
		}
		if(scene->GetObjectType(conId) != ConnectorProp::id){
			Message::Error("%s: connector must link to Connector", name.c_str());
			return SupportState::Ignored;
		}
		if(IsUndefined(shapeId) || IsUndefined(conId))
			return SupportState::Undefined;
		
		// visual material is optional
		if(IsValidID(matId) && IsUndefined(matId))
			return SupportState::Undefined;
		
		AUTO(ShapeAux*,     shapeAux, GetAux(shapeId));
		AUTO(ConnectorAux*, conAux,   GetAux(conId));
		AUTO(MaterialAux*,  matAux,   GetAux(matId));

		AttachAux* attachAux = new AttachAux(shapeAux, 0, 0, conAux, matAux);
		Message::Out("created AttachAux for %s", name.c_str());
				
		attaches.push_back(attachAux);
		RegAux(id, attachAux);
		return SupportState::Supported;
	}
	return SupportState::Ignored;
}

void AdaptorPovray::DeleteObject(int id){
	int type = scene->GetObjectType(id);
	Aux* aux = GetAux(id);
	
	if(type == BodyProp::id){
		AUTO(BodyAux*, bodyAux, aux);
		RemoveFromArray(bodies, bodyAux);
	}
	else if(type == ConnectorProp::id){

	}
	else if(type == VisualMaterialProp::id){

	}
	else if(typedb->KindOf(type, ShapeProp::id)){

	}
	if(type == AttachProp::id){
		AUTO(AttachAux*, attachAux, aux);
		RemoveFromArray(attaches, attachAux);
	}

	RemoveAux(id);
}

void AdaptorPovray::SyncObjectProperty(int id, bool download, int cat){
	if(!IsSupported(id))
		return;
	
	int type = scene->GetObjectType(id);
	Property* prop = scene->GetProperty(id);

	if(type == BodyProp::id){
		// Bodyの位置をフレームに反映
		AUTO(BodyProp*, bodyProp, prop);
		AUTO(BodyAux*, bodyAux, GetAux(id));
		if(cat & AttrCategory::State){
			Transform tr;
			int par = scene->FindAncestor(id, SpatialObjectProp::id, typedb);
			if(par != -1)
				scene->CalcRelativeTransform(-1, par, typedb, tr);
		}
	}
	else if(type == ConnectorProp::id){
		AUTO(ConnectorProp*, conProp, prop);
		AUTO(ConnectorAux*, conAux, GetAux(id));
		if(cat & AttrCategory::Param && download){
			// calculate relative pose between owner body
			int bodyId = scene->FindAncestor(id, BodyProp::id, typedb);
		}
	}
	else if(typedb->KindOf(type, ShapeProp::id)){
		if(type == BoxProp::id){
			AUTO(BoxProp*, boxProp, prop);
			AUTO(BoxAux*, boxAux, GetAux(id));
			if(cat & AttrCategory::Param){
				if(download){
					boxAux->size = boxProp->size;
				}
				else{
					boxProp->size = boxAux->size;
				}
			}
		}
		else if(type == SphereProp::id){
			AUTO(SphereProp*, sphereProp, prop);
			AUTO(SphereAux*, sphereAux, GetAux(id));
			if(cat & AttrCategory::Param){
				if(download){
					sphereAux->radius = (float)sphereProp->radius;
				}
				else{
					sphereProp->radius = (real_t)sphereAux->radius;
				}
			}
		}
		else if(type == CylinderProp::id){
			AUTO(CylinderProp*, cylinderProp, prop);
			AUTO(CylinderAux*, cylinderAux, GetAux(id));
			if(cat & AttrCategory::Param){
				if(download){
					cylinderAux->radius = (float)cylinderProp->radius;
					cylinderAux->height = (float)cylinderProp->height;
				}
				else{
					cylinderProp->radius = (real_t)cylinderAux->radius;
					cylinderProp->height = (real_t)cylinderAux->height;
				}
			}
		}
		else if(type == MeshProp::id){
			AUTO(MeshProp*, meshProp, prop);
			AUTO(MeshAux*, meshAux, GetAux(id));
			if(cat & AttrCategory::Param && download){
				// ファイル名変更時のリロードは未対応
			}
		}
	}
	else if(type == VisualMaterialProp::id){
		AUTO(VisualMaterialProp*, matProp, prop);
		AUTO(MaterialAux*, matAux, GetAux(id));
		if(cat & AttrCategory::Param){
			if(download){
			}
			else{
			}
		}
	}
}

void AdaptorPovray::WriteHeader(ostream& os){

}

void AdaptorPovray::Write(ostream& os){
	/*
	// ヘッダ
	WriteHeader(os);
	
	// 設定
	option.Write(os);
	
	// カメラ
	camera.Write(os);

	// 光源
	foreach(Light& light, lights)
		light.Write(os);

	*/
}

}
