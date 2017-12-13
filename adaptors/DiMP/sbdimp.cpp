#include "sbdimp.h"
#include <sbmessage.h>
#include <sbmodel.h>

#define AUTO(T, X, EXP) T X = static_cast<T>(EXP)

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorDiMP::AdaptorDiMP(){
	graph    = 0;
	//syncTime = 0.0;
}

void AdaptorDiMP::Set(DiMP::Graph* g){
	graph = g;
}

//void AdaptorDiMP::SetSyncTime(real_t time){
//	syncTime = time;
//}

int AdaptorDiMP::CreateObject(int id){
	int type       = scene->GetObjectType(id);
	Property* prop = scene->GetProperty(id);

	// 重複を避けるためフルパスをDiMP側での名前とする
	//string name    = scene->GetName(id);
	Address addr;
	scene->GetAddress(addr, scene->GetRoot(), id);
	string name = addr.ToString();
	
	if(type == BodyProp::id){
		DiMP::Object* obj = new DiMP::Object(graph, name);
		RegAux(id, new BodyAux(obj));
		return SupportState::Supported;
	}
	else if(type == ConnectorProp::id){
		int bodyId = scene->FindAncestor(id, BodyProp::id, typedb);
		if(!IsValidID(bodyId) || IsIgnored(bodyId))
			return SupportState::Ignored;
		if(IsUndefined(bodyId))
			return SupportState::Undefined;
	
		AUTO(BodyAux*, bodyAux, GetAux(bodyId));
		
		// コネクタを作成
		DiMP::Connector* con = new DiMP::Connector(bodyAux->obj);
		
		RegAux(id, new ConnectorAux(bodyAux, con));
		return SupportState::Supported;
	}
	else if(typedb->KindOf(type, ShapeProp::id)){
		AUTO(ShapeProp*, shapeProp, prop);

		// 衝突判定形状
		if(shapeProp->collision){
			ShapeAux* shape = 0;
			if(type == BoxProp::id){
				shape = new ShapeAux(new DiMP::Box(graph, ((BoxProp*)prop)->size, name));
			}
			else if(type == SphereProp::id){
				shape = new ShapeAux(new DiMP::Sphere(graph, ((SphereProp*)prop)->radius, name));
			}
			else if(type == CylinderProp::id){
				shape = new ShapeAux(new DiMP::Cylinder(graph, ((CylinderProp*)prop)->radius, ((CylinderProp*)prop)->height, name));
			}
			else if(type == MeshProp::id){
				AUTO(MeshProp*, meshProp, prop);
			
				// 衝突判定用でなければ無視
				if(!meshProp->collision)
					return SupportState::Ignored;

				// モデルコンテナから取得
				Model* model = models->GetModel(id);
				if(!model){
					Message::Error("model %s not found in the container", (const char*)meshProp->filename);
					return SupportState::Ignored;
				}
				
				shape = new ShapeAux();
				if(meshProp->convex){
					DiMP::Mesh* geo = new DiMP::Mesh(graph, name);
					for(uint i = 0; i < model->meshSolid.size(); i++){
						Mesh& mesh = model->meshSolid[i];
						int ntri = ((int)mesh.positions.size())/3;
						geo->tris.resize(ntri);
						for(int j = 0; j < ntri; j++){
							geo->tris[j].vertices[0] = mesh.positions[3*j+0];
							geo->tris[j].vertices[1] = mesh.positions[3*j+1];
							geo->tris[j].vertices[2] = mesh.positions[3*j+2];
							geo->tris[j].normal      = mesh.normals  [3*j+0];
						}
					}
					shape->geos.push_back(geo);
				}
				else{

				}
			}
			if(shape){
				RegAux(id, shape);
				return SupportState::Supported;
			}
		}
	}
	else if(type == AttachProp::id){
		int shapeId = scene->GetLink(id, "shape");
		int conId   = scene->GetLink(id, "connector");
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
		if(IsUndefined(shapeId) || IsUndefined(conId)){
			return SupportState::Undefined;
		}

		AUTO(ShapeAux*, shapeAux, GetAux(shapeId));
		AUTO(ConnectorAux*, conAux, GetAux(conId));

		conAux->con->geos.Add(&shapeAux->geos[0], shapeAux->geos.size());
		RegAux(id, new AttachAux(shapeAux, conAux));
		return SupportState::Supported;
	}
	else if(typedb->KindOf(type, JointProp::id)){
		int sockId = scene->GetLink(id, "sock");
		int plugId = scene->GetLink(id, "plug");

		if(!IsValidID(sockId) || IsIgnored(sockId)){
			Message::Error("%s: invalid link to sock", name.c_str());
			return SupportState::Ignored;
		}
		if(scene->GetObjectType(sockId) != ConnectorProp::id){
			Message::Error("%s: sock must link to Connector", name.c_str());
			return SupportState::Ignored;
		}
		if(!IsValidID(plugId) || IsIgnored(plugId)){
			Message::Error("%s: invalid link to plug", name.c_str());
			return SupportState::Ignored;
		}
		if(scene->GetObjectType(plugId) != ConnectorProp::id){
			Message::Error("%s: plug must link to Connector", name.c_str());
			return SupportState::Ignored;
		}
		if(IsUndefined(sockId) || IsUndefined(plugId))
			return SupportState::Undefined;

		AUTO(ConnectorAux*, sockAux, GetAux(sockId));
		AUTO(ConnectorAux*, plugAux, GetAux(plugId));
		
		// create joint
		DiMP::Joint* jnt;
		if(type == HingeProp::id)
			jnt = new DiMP::Hinge(sockAux->con, plugAux->con, 0, name);
		else if(type == SliderProp::id)
			jnt = new DiMP::Slider(sockAux->con, plugAux->con, 0, name);
		else if(type == BalljointProp::id)
			jnt = new DiMP::Balljoint(sockAux->con, plugAux->con, 0, name);
		else return false;
		
		JointAux* jointAux = new JointAux(jnt, sockAux, plugAux);
		RegAux(id, jointAux);
		return SupportState::Supported;
	}
	else if(type == GravityProp::id){
		return SupportState::Supported;
	}
	return false;
}

void AdaptorDiMP::DeleteObject(int id){

}

void AdaptorDiMP::SyncObjectProperty(int id, bool download, int cat){
	if(!IsSupported(id))
		return;
	int type = scene->GetObjectType(id);	
	Property* prop = scene->GetProperty(id);

	if(type == BodyProp::id){
		AUTO(BodyAux*, bodyAux, GetAux(id));
		AUTO(BodyProp*, bodyProp, prop);
		DiMP::Object* obj = bodyAux->obj;
		
		if(cat & AttrCategory::Param){
			if(download){
				obj->param.mass      = bodyProp->mass;
				obj->param.inertia   = bodyProp->inertia[0][0];
				obj->param.dynamical = bodyProp->dynamical;
			}
			else{

			}
		}
		if(cat & AttrCategory::State){
			// Bodyが属するNamespaceの位置と向きを取得
			Transform tr;
			int par = scene->FindAncestor(id, SpatialObjectProp::id, typedb);
			if(par != -1)
				scene->CalcRelativeTransform(-1, par, typedb, tr);
			
			if(download){
				// 初期状態に反映
				obj->param.iniPos    = tr.rot * bodyProp->trn + tr.trn;
				obj->param.iniOri    = tr.rot * bodyProp->rot;
				obj->param.iniVel    = bodyProp->vel;
				obj->param.iniAngvel = bodyProp->angvel;
			}
			else{
				// syncTimeに設定された時刻の状態
				quat_t qinv = tr.rot.Conjugated();
				bodyProp->trn    = qinv * (obj->snapshot.pos - tr.trn);
				bodyProp->rot    = qinv *  obj->snapshot.ori;
				bodyProp->vel    = obj->snapshot.vel;
				bodyProp->angvel = obj->snapshot.angvel;
				//bodyProp->trn    = qinv * (obj->Pos(syncTime) - tr.trn);
				//bodyProp->rot    = qinv * obj->Ori(syncTime);
				//bodyProp->vel    = obj->Vel(syncTime);
				//bodyProp->angvel = obj->Angvel(syncTime);
			}
		}
	}
	else if(type == ConnectorProp::id){
		AUTO(ConnectorAux*, conAux, GetAux(id));
		if(cat & AttrCategory::State && download){
			// calculate relative pose between owner body
			int bodyId = scene->FindAncestor(id, BodyProp::id, typedb);
			Transform trans;
			scene->CalcRelativeTransform(bodyId, id, typedb, trans);
			conAux->con->pose.Pos() = trans.trn;
			conAux->con->pose.Ori() = trans.rot;
		}

	}
	else if(typedb->KindOf(type, ShapeProp::id)){
		AUTO(ShapeAux*, shapeAux, GetAux(id));
		if(type == BoxProp::id){
			AUTO(BoxProp*, boxProp, prop);
			AUTO(DiMP::Box*, box, shapeAux->geos[0]);
			if(cat & AttrCategory::Param){
				if(download){
					box->size = boxProp->size;
				}
				else{
					boxProp->size = box->size;
				}
			}
		}
		else if(type == SphereProp::id){
			AUTO(SphereProp*, sphereProp, prop);
			AUTO(DiMP::Sphere*, sphere, shapeAux->geos[0]);
			if(cat & AttrCategory::Param){
				if(download){
					sphere->radius = sphereProp->radius;
				}
				else{
					sphereProp->radius = sphere->radius;
				}
			}
		}
		else if(type == CylinderProp::id){
			AUTO(CylinderProp*, cylinderProp, prop);
			AUTO(DiMP::Cylinder*, cylinder, shapeAux->geos[0]);
			if(cat & AttrCategory::Param){
				if(download){
					cylinder->radius = cylinderProp->radius;
					cylinder->length = cylinderProp->height;
				}
				else{
					cylinderProp->radius = cylinder->radius;
					cylinderProp->height = cylinder->length;
				}
			}
		}
	}
	else if(typedb->KindOf(type, JointProp::id)){
		AUTO(JointAux*, jntAux, GetAux(id));

		if(typedb->KindOf(type, HingeProp::id)){
			AUTO(HingeProp*, hingeProp, prop);
			if(download){
				jntAux->joint->param.ini_p [0] = hingeProp->pos;
				jntAux->joint->param.ini_v [0] = hingeProp->vel;
				jntAux->joint->param.rmin_p[0] = hingeProp->range [0];
				jntAux->joint->param.rmax_p[0] = hingeProp->range [1];
				jntAux->joint->param.rmin_v[0] = hingeProp->vrange[0];
				jntAux->joint->param.rmax_v[0] = hingeProp->vrange[1];
			}
		}
		if(typedb->KindOf(type, SliderProp::id)){
			AUTO(SliderProp*, sliderProp, prop);
			if(download){
				jntAux->joint->param.ini_p [0] = sliderProp->pos;
				jntAux->joint->param.ini_v [0] = sliderProp->vel;
				jntAux->joint->param.rmin_p[0] = sliderProp->range [0];
				jntAux->joint->param.rmax_p[0] = sliderProp->range [1];
				jntAux->joint->param.rmin_v[0] = sliderProp->vrange[0];
				jntAux->joint->param.rmax_v[0] = sliderProp->vrange[1];
			}
		}
	}
	else if(type == GravityProp::id){
		if(download)
			graph->param.gravity = ((GravityProp*)prop)->accel;
	}
}

DiMP::Object* AdaptorDiMP::GetObject(int id){
	AUTO(BodyAux*, bodyAux, HandleByID(id, BodyProp::id));
	return bodyAux->obj;
}

DiMP::Object* AdaptorDiMP::GetObject(string name){
	AUTO(BodyAux*, bodyAux, HandleByName(name, BodyProp::id));
	return bodyAux->obj;
}

DiMP::Joint* AdaptorDiMP::GetJoint(int id){
	AUTO(JointAux*, jntAux, HandleByID(id, JointProp::id));
	return jntAux->joint;
}

DiMP::Joint* AdaptorDiMP::GetJoint(string name){
	AUTO(JointAux*, jntAux, HandleByName(name, JointProp::id));
	return jntAux->joint;
}

std::pair<DiMP::Geometry*, size_t> AdaptorDiMP::GetGeometry(int id){
	AUTO(ShapeAux*, shapeAux, HandleByID(id, ShapeProp::id));
	return std::make_pair(shapeAux->geos[0], shapeAux->geos.size());
}

std::pair<DiMP::Geometry*, size_t> AdaptorDiMP::GetGeometry(string name){
	AUTO(ShapeAux*, shapeAux, HandleByName(name, ShapeProp::id));
	return std::make_pair(shapeAux->geos[0], shapeAux->geos.size());
}

}
