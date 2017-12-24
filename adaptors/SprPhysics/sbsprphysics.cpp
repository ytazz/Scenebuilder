#include "sbsprphysics.h"
#include <sbmessage.h>
#include <sbpath.h>
#include <sbmodelstl.h>

#include <functional>
using namespace std::placeholders;

#include <windows.h>
#undef min
#undef max

#define AUTO(T, X, EXP) T X = static_cast<T>(EXP)

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorSprPH::BodyAux::BodyAux(PHSolidIf* s){
	solid     = s;
	treeNode  = 0;
	auto_mass = false;
	auto_tree = false;
}

AdaptorSprPH::BodyAux::~BodyAux(){
	for(uint i = 0; i < cons.size(); i++)
		cons[i]->body = 0;
	for(uint i = 0; i < groups.size(); i++)
		RemoveFromArray(groups[i]->bodies, this);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorSprPH::ConnectorAux::ConnectorAux(BodyAux* b):body(b){
	body->cons.push_back(this);
}

AdaptorSprPH::ConnectorAux::~ConnectorAux(){
	if(body)
		RemoveFromArray(body->cons, this);

	for(uint i = 0; i < attaches.size(); i++){
		attaches[i]->con = 0;
		attaches[i]->OnChange(this);
	}
	for(uint i = 0; i < joints.size(); i++){
		JointAux* jnt = joints[i];
		if(jnt->sock == this)
			jnt->sock = 0;
		if(jnt->plug == this)
			jnt->plug = 0;
		jnt->OnChange(this);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorSprPH::MaterialAux& AdaptorSprPH::MaterialAux::operator=(PhysicalMaterialProp& prop){
	mat.density = (float)prop.density;
	mat.e       = (float)prop.cor;
	mat.mu0     = (float)prop.static_friction;
	mat.mu      = (float)prop.dynamic_friction;
	mat.spring  = (float)prop.spring;
	mat.damper  = (float)prop.damper;

	if(prop.velocity_field_mode == "none"    ) mat.velocityFieldMode = PHMaterial::VelocityField::NONE;
	if(prop.velocity_field_mode == "linear"  ) mat.velocityFieldMode = PHMaterial::VelocityField::LINEAR;
	if(prop.velocity_field_mode == "cylinder") mat.velocityFieldMode = PHMaterial::VelocityField::CYLINDER;
	mat.velocityFieldAxis      = prop.velocity_field_axis;
	mat.velocityFieldMagnitude = prop.velocity_field_magnitude;

	return *this;
}

AdaptorSprPH::MaterialAux::~MaterialAux(){
	for(uint i = 0; i < attaches.size(); i++)
		attaches[i]->mat = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorSprPH::ShapeAux::ShapeAux(){

}

AdaptorSprPH::ShapeAux::~ShapeAux(){
	for(uint i = 0; i < attaches.size(); i++)
		attaches[i]->shape = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorSprPH::AttachAux::AttachAux(ShapeAux* s, ConnectorAux* c, MaterialAux* m, int ib, int ie):shape(s), con(c), mat(m), idxBegin(ib), idxEnd(ie){
	con  ->attaches.push_back(this);
	shape->attaches.push_back(this);
	if(mat)
		mat->attaches.push_back(this);
}

AdaptorSprPH::AttachAux::~AttachAux(){
	if(con)
		RemoveFromArray(con->attaches, this);
	if(shape)
		RemoveFromArray(shape->attaches, this);
	if(mat)
		RemoveFromArray(shape->attaches, this);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorSprPH::GenericJointCallback::GenericJointCallback(){
	hDll                    = 0;
	hSetParam               = 0;
	hIsCyclic               = 0;
	hGetMovableAxes         = 0;
	hCompBias               = 0;
	hCompError              = 0;
	hUpdateJointState       = 0;
	hCompJointJacobian      = 0;
	hCompJointCoriolisAccel = 0;
	hCompRelativePosition   = 0;
	hCompRelativeVelocity   = 0;
}

AdaptorSprPH::GenericJointCallback::~GenericJointCallback(){
	if(hDll)
		FreeLibrary((HMODULE)hDll);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorSprPH::JointAux::JointAux(PHJointIf* j, ConnectorAux* s, ConnectorAux* p):joint(j), sock(s), plug(p){
	sock->joints.push_back(this);
	plug->joints.push_back(this);		
}

AdaptorSprPH::JointAux::~JointAux(){
	if(sock)
		RemoveFromArray(sock->joints, this);
	if(plug)
		RemoveFromArray(plug->joints, this);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorSprPH::GearAux::GearAux(PHGearIf* g){
	gear = g;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorSprPH::ContactGroupAux::~ContactGroupAux(){
	for(uint i = 0; i < bodies.size(); i++)
		RemoveFromArray(bodies[i]->groups, this);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void ReportInertia(PHSolidIf* s){
	DSTR << s->GetName() << " : " << "m = " << s->GetMass() << ", com = " << s->GetCenterOfMass() << endl;
}

void AdaptorSprPH::AttachAux::OnChange(Aux* caller){
	if(caller == con){
		// 形状配置の変更を反映
		if(con->body){
			for(int i = idxBegin; i < idxEnd; i++)
				con->body->solid->SetShapePose(i, con->pose);
			// 質量の再計算
			if(con->body->auto_mass){
				con->body->solid->CompInertia();
				ReportInertia(con->body->solid);
			}
		}
	}
	if(caller == mat){
		if(shape){
			// 物性の変化を反映
			for(uint i = 0; i < shape->shapes.size(); i++){
				CDShapeIf* sh = shape->shapes[i];
				//sh->SetDensity        (mat->mat.density);
				//sh->SetElasticity     (mat->mat.e      );
				//sh->SetStaticFriction (mat->mat.mu0    );
				//sh->SetDynamicFriction(mat->mat.mu     );
				//sh->SetContactSpring  (mat->mat.spring );
				//sh->SetContactDamper  (mat->mat.damper );
				sh->SetMaterial(mat->mat);
			}
		}
		// 質量の再計算
		if(con && con->body && con->body->auto_mass){
			con->body->solid->CompInertia();
			ReportInertia(con->body->solid);
		}
	}
}

void AdaptorSprPH::JointAux::OnChange(Aux* caller){
	if(caller == sock)
		joint->SetSocketPose(sock->pose);
	if(caller == plug)
		joint->SetPlugPose(plug->pose);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void AdaptorSprPH::GenericJointCallback::Init(){
	hSetParam               = (SetParamFunc              )GetProcAddress((HMODULE)hDll, "SetParamFunc"              );
	hIsCyclic               = (IsCyclicFunc              )GetProcAddress((HMODULE)hDll, "IsCyclicFunc"              );
	hGetMovableAxes         = (GetMovableAxesFunc        )GetProcAddress((HMODULE)hDll, "GetMovableAxesFunc"        );
	hCompBias               = (CompBiasFunc              )GetProcAddress((HMODULE)hDll, "CompBiasFunc"              );
	hCompError              = (CompErrorFunc             )GetProcAddress((HMODULE)hDll, "CompErrorFunc"             );
	hUpdateJointState       = (UpdateJointStateFunc      )GetProcAddress((HMODULE)hDll, "UpdateJointStateFunc"      );
	hCompJointJacobian      = (CompJointJacobianFunc     )GetProcAddress((HMODULE)hDll, "CompJointJacobianFunc"     );
	hCompJointCoriolisAccel = (CompJointCoriolisAccelFunc)GetProcAddress((HMODULE)hDll, "CompJointCoriolisAccelFunc");
	hCompRelativePosition   = (CompRelativePositionFunc  )GetProcAddress((HMODULE)hDll, "CompRelativePositionFunc"  );
	hCompRelativeVelocity   = (CompRelativeVelocityFunc  )GetProcAddress((HMODULE)hDll, "CompRelativeVelocityFunc"  );
}

void AdaptorSprPH::GenericJointCallback::SetParam(PHGenericJointIf* jnt, const string& name, double value){
	if(hSetParam)
		return hSetParam(jnt, name.c_str(), value);
}
bool AdaptorSprPH::GenericJointCallback::IsCyclic(PHGenericJointIf* jnt){
	if(hIsCyclic)
		return hIsCyclic(jnt);
	return PHGenericJointCallback::IsCyclic(jnt);
}
void AdaptorSprPH::GenericJointCallback::GetMovableAxes(PHGenericJointIf* jnt, int& n, int* indices){
	if(hGetMovableAxes)
		hGetMovableAxes(jnt, &n, indices);
}
void AdaptorSprPH::GenericJointCallback::CompBias(PHGenericJointIf* jnt, Vec3d& dbv, Vec3d& dbw, const Vec3d& prel, const Quaterniond& qrel, const Vec3d& vrel, const Vec3d& wrel){
	if(hCompBias)
		hCompBias(jnt, (double*)&dbv, (double*)&dbw, (const double*)&prel, (const double*)&qrel, (const double*)&vrel, (const double*)&wrel);
}
void AdaptorSprPH::GenericJointCallback::CompError(PHGenericJointIf* jnt, Vec3d& Bv, Vec3d& Bw, const Vec3d& prel, const Quaterniond& qrel){
	if(hCompError)
		hCompError(jnt, (double*)&Bv, (double*)&Bw, (const double*)&prel, (const double*)&qrel);
}
void AdaptorSprPH::GenericJointCallback::UpdateJointState(PHGenericJointIf* jnt, double& pos, double& vel, const Vec3d& prel, const Quaterniond& qrel, const Vec3d& vrel, const Vec3d& wrel){
	if(hUpdateJointState)
		hUpdateJointState(jnt, &pos, &vel, (const double*)&prel, (const double*)&qrel, (const double*)&vel, (const double*)&wrel);
}
void AdaptorSprPH::GenericJointCallback::CompJointJacobian(PHGenericJointIf* jnt, Vec3d& Jv, Vec3d& Jw, double pos){
	if(hCompJointJacobian)
		hCompJointJacobian(jnt, (double*)&Jv, (double*)&Jw, pos);
}
void AdaptorSprPH::GenericJointCallback::CompJointCoriolisAccel(PHGenericJointIf* jnt, Vec3d& cv, Vec3d& cw, double pos, double vel){
	if(hCompJointCoriolisAccel)
		hCompJointCoriolisAccel(jnt, (double*)&cv, (double*)&cw, pos, vel);
}
void AdaptorSprPH::GenericJointCallback::CompRelativePosition(PHGenericJointIf* jnt, Vec3d& prel, Quaterniond& qrel, double pos){
	if(hCompRelativePosition)
		hCompRelativePosition(jnt, (double*)prel, (double*)&qrel, pos);
}
void AdaptorSprPH::GenericJointCallback::CompRelativeVelocity(PHGenericJointIf* jnt, Vec3d& vrel, Vec3d& wrel, double pos, double vel){
	if(hCompRelativeVelocity)
		hCompRelativeVelocity(jnt, (double*)&vrel, (double*)&wrel, pos, vel);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorSprPH::AdaptorSprPH(){
	phSdk     = 0;
	phScene   = 0;
	treeReady = false;
}

void AdaptorSprPH::Set(PHSdkIf* _phSdk, PHSceneIf* _phScene){
	phSdk	= _phSdk;
	phScene	= _phScene;
}

PHSolidIf*	AdaptorSprPH::GetSolid(int id){
	AUTO(BodyAux*, bodyAux, HandleByID(id, BodyProp::id));
	return bodyAux->solid;
}

PHSolidIf*	AdaptorSprPH::GetSolid(string name){
	AUTO(BodyAux*, bodyAux, HandleByName(name, BodyProp::id));
	return bodyAux->solid;
}

PHJointIf*	AdaptorSprPH::GetJoint(int id){
	AUTO(JointAux*, jointAux, HandleByID(id, JointProp::id));
	return jointAux->joint;
}

PHJointIf*	AdaptorSprPH::GetJoint(string name){
	AUTO(JointAux*, jointAux, HandleByName(name, JointProp::id));
	return jointAux->joint;
}

CDShapeIf**	AdaptorSprPH::GetShape(int id){
	AUTO(ShapeAux*, shapeAux, HandleByID(id, ShapeProp::id));
	return &shapeAux->shapes[0];
}

CDShapeIf**	AdaptorSprPH::GetShape(string name){
	AUTO(ShapeAux*, shapeAux, HandleByName(name, ShapeProp::id));
	return &shapeAux->shapes[0];
}

void AdaptorSprPH::AddDllSearchPath(const string& str){
	dllSearchPaths.push_back(str);
}

void AdaptorSprPH::ConvertMesh(vector<CDShapeIf*>& sh, Model* model, MeshProp* meshProp){
	Message::Out("converting model to CDConvexMesh");
	
	for(uint i = 0; i < model->meshSolid.size(); i++){
		Mesh& mesh = model->meshSolid[i];

		if(meshProp->prism){
			Message::Out("converting to prism: %d vertices", mesh.positions.size());
			// 角柱化する場合，各面から三角柱をつくる
			Vec3f dir = meshProp->prismdir;

			uint n = (uint)mesh.positions.size();
			for(uint j = 0; j < n; j += 3){
				// プリズム化の向きに対して反対向きの面はスキップ
				if(mesh.normals[j+0] * dir <= 0.0) continue;
				if(mesh.normals[j+1] * dir <= 0.0) continue;
				if(mesh.normals[j+2] * dir <= 0.0) continue;
				
				CDConvexMeshDesc cd;
				Vec3f v, vp;
				for(int k = 0; k < 3; k++){
					v = mesh.positions[j+k];
					// 角柱の底面に射影した点
					float s = - (v * dir) / (dir * dir);
					vp = v + s * dir;
					cd.vertices.push_back(v);
					cd.vertices.push_back(vp);
				}
				sh.push_back(phSdk->CreateShape(cd));
			}
			Message::Out("converting to prism finished");
		}
		else{
			// 凸包を作る
			CDConvexMeshDesc cd;
			for(uint j = 0; j < mesh.positions.size(); j++)
				cd.vertices.push_back(mesh.positions[j]);
			sh.push_back(phSdk->CreateShape(cd));
		}
	}
}

int AdaptorSprPH::CreateObject(int id){
	int type = scene->GetObjectType(id);
	Property* prop = scene->GetProperty(id);
	string name = scene->GetName(id);
	
	if(type == BodyProp::id){
		BodyAux* body = new BodyAux(phScene->CreateSolid());
		RegAux(id, body);
		bodies.push_back(body);
		treeReady = false;

		Message::Extra("created PHSolid for %s", name.c_str());
		return SupportState::Supported;
	}
	else if(type == PhysicalMaterialProp::id){
		RegAux(id, new MaterialAux());
		Message::Extra("created MaterialAux for %s", name.c_str());
		return SupportState::Supported;
	}
	else if(typedb->KindOf(type, ShapeProp::id)){
		AUTO(ShapeProp*, shapeProp, prop);

		// collision shape以外は無視
		if(!shapeProp->collision)
			return SupportState::Ignored;

		ShapeAux* shapeAux = 0;
		if(type == BoxProp::id){
			shapeAux = new ShapeAux();
			shapeAux->shapes.push_back(phSdk->CreateShape(CDBoxDesc()));
			Message::Extra("created CDBox for %s", name.c_str());
		}
		else if(type == SphereProp::id){
			shapeAux = new ShapeAux();
			shapeAux->shapes.push_back(phSdk->CreateShape(CDSphereDesc()));
			Message::Extra("created CDSphere for %s", name.c_str());
		}
		else if(type == CapsuleProp::id){
			shapeAux = new ShapeAux();
			shapeAux->shapes.push_back(phSdk->CreateShape(CDCapsuleDesc()));
			Message::Extra("created CDCapsule for %s", name.c_str());
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

			// CDConvexMeshへ変換
			vector<CDShapeIf*> sh;
			ConvertMesh(sh, model, meshProp);

			if(!sh.empty()){
				shapeAux = new ShapeAux();
				shapeAux->shapes.assign(sh.begin(), sh.end());
				Message::Extra("created %d CDConvexMesh for %s", sh.size(), name.c_str());
			}
		}

		if(shapeAux){
			RegAux(id, shapeAux);
			return SupportState::Supported;
		}
		else{
			Message::Out("shape type of %s not supported", name.c_str());
		}
	}
	else if(type == ConnectorProp::id){
		int bodyId = scene->FindAncestor(id, BodyProp::id, typedb);
		if(!IsValidID(bodyId) || IsIgnored(bodyId)){
			Message::Error("%s: body invalid", name.c_str());
			return SupportState::Ignored;
		}
		if(IsUndefined(bodyId))
			return SupportState::Undefined;

		AUTO(BodyAux*, bodyAux, GetAux(bodyId));
		ConnectorAux* conAux = new ConnectorAux(bodyAux);
		
		RegAux(id, conAux);
		Message::Extra("created ConnectorAux for %s", name.c_str());
		
		return SupportState::Supported;
	}
	else if(type == AttachProp::id){
		AUTO(AttachProp*, attProp, prop);
			
		// get PHSolid and CDShape
		int shapeId = scene->GetLink(id, "shape");
		int conId   = scene->GetLink(id, "connector");
		int matId   = scene->GetLink(id, "pmat");
		
		if(!IsValidID(shapeId) || IsIgnored(shapeId)){
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
		// physical material is optional
		if(IsValidID(matId) && IsUndefined(matId))
			return SupportState::Undefined;

		AUTO(ShapeAux*    , shapeAux, GetAux(shapeId));
		AUTO(ConnectorAux*, conAux  , GetAux(conId  ));
		AUTO(MaterialAux* , matAux  , GetAux(matId  ));

		// assign shape (pose will be set later)
		int idxBegin = conAux->body->solid->NShape();
		for(uint i = 0; i < shapeAux->shapes.size(); i++)
			conAux->body->solid->AddShape(shapeAux->shapes[i]);
		//conAux->body->solid->AddShapes(&shapeAux->shapes[0], &shapeAux->shapes[0] + shapeAux->shapes.size());
		int idxEnd = conAux->body->solid->NShape();

		AttachAux* attAux = new AttachAux(shapeAux, conAux, matAux, idxBegin, idxEnd);

		RegAux(id, attAux);
		Message::Extra("created AttachAux for %s", name.c_str());
		
		return SupportState::Supported;
	}
	else if(typedb->KindOf(type, JointProp::id)){
		// get connectors
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
		PHJointIf* phJoint = 0;
		// Joint1Dでfilenameが指定されていればGenericJointを作成
		bool  genericJoint = false;
		void* hDll;
		string          filename;
		vector<string>	paramnames;
		vector<real_t>	paramvalues;

		if(typedb->KindOf(type, Joint1DProp::id)){
			AUTO(Joint1DProp*, prop1d, prop);
			if(!(prop1d->filename == "")){
				genericJoint = true;
				// filenameをファイルパスとパラメータに分解
				string tmp(prop1d->filename);
				Tokenizer tok(tmp, ":", true);
				filename = to_string(tok.GetToken());
				tok.Next();
				if(!tok.IsEnd()){
					Tokenizer subtok(tok.GetToken(), ";", true);
					while(!subtok.IsEnd()){
						Tokenizer subsubtok(subtok.GetToken(), "=", true);
						string_iterator_pair tokname, tokvalue;
						if(!subsubtok.IsEnd())
							tokname  = subsubtok.GetToken();
						subsubtok.Next();
						if(!subsubtok.IsEnd())
							tokvalue = subsubtok.GetToken();

						real_t value;
						Converter::FromString(tokvalue, value);

						paramnames .push_back(to_string(tokname));
						paramvalues.push_back(value);
					
						subtok.Next();
					}
				}

				// DLLをロード
				Path path = Path(scene->GetLocation(id)) + Path(filename);
				(HMODULE&)hDll = LoadLibrary(path.c_str());
				if(!hDll){
					for(uint i = 0; i < dllSearchPaths.size(); i++){
						Path path = Path(dllSearchPaths[i]) + Path(filename);
						(HMODULE&)hDll = LoadLibrary(path.c_str());
						if(hDll)
							break;
					}
				}
				if(!hDll){
					Message::Error("failed to load dll for generic joint: %s", prop1d->filename);
					return SupportState::Ignored;
				}
			}
		}
		if(genericJoint){
			phJoint = phScene->CreateJoint(sockAux->body->solid, plugAux->body->solid, PHGenericJointDesc());
			Message::Extra("created PHGenericJoint for %s", name.c_str());
		}
		else if(type == HingeProp::id){
			phJoint = phScene->CreateJoint(sockAux->body->solid, plugAux->body->solid, PHHingeJointDesc());
			Message::Extra("created PHHingeJoint for %s", name.c_str());
		}
		else if(type == SliderProp::id){
			phJoint = phScene->CreateJoint(sockAux->body->solid, plugAux->body->solid, PHSliderJointDesc());
			Message::Extra("created PHSliderJoint for %s", name.c_str());
		}
		else if(type == BalljointProp::id){
			phJoint = phScene->CreateJoint(sockAux->body->solid, plugAux->body->solid, PHBallJointDesc());
			Message::Extra("created PHBallJoint for %s", name.c_str());
		}
		else if(type == SpringProp::id){
			phJoint = phScene->CreateJoint(sockAux->body->solid, plugAux->body->solid, PHSpringDesc());
			Message::Extra("created PHSpring for %s", name.c_str());
		}
		else if(type == FixjointProp::id){
			phJoint = phScene->CreateJoint(sockAux->body->solid, plugAux->body->solid, PHFixJointDesc());
			Message::Extra("created PHFixJoint for %s", name.c_str());
		}
		else if(type == PointToPointProp::id){
			phJoint = phScene->CreateJoint(sockAux->body->solid, plugAux->body->solid, PHPointToPointMateDesc());
			Message::Extra("created PHPointToPointMate for %s", name.c_str());
		}
		else if(type == PointToLineProp::id){
			phJoint = phScene->CreateJoint(sockAux->body->solid, plugAux->body->solid, PHPointToLineMateDesc());
			Message::Extra("created PHPointToLineMate for %s", name.c_str());
		}
		else if(type == PointToPlaneProp::id){
			phJoint = phScene->CreateJoint(sockAux->body->solid, plugAux->body->solid, PHPointToPlaneMateDesc());
			Message::Extra("created PHPointToPlaneMate for %s", name.c_str());
		}
		else if(type == LineToLineProp::id){
			phJoint = phScene->CreateJoint(sockAux->body->solid, plugAux->body->solid, PHLineToLineMateDesc());
			Message::Extra("created PHLineToLineMate for %s", name.c_str());
		}
		else if(type == PlaneToPlaneProp::id){
			phJoint = phScene->CreateJoint(sockAux->body->solid, plugAux->body->solid, PHPlaneToPlaneMateDesc());
			Message::Extra("created PHPlaneToPlaneMate for %s", name.c_str());
		}

		if(phJoint){
			// create joint limit
			if(typedb->KindOf(type, Joint1DProp::id)){
				DCAST(PH1DJointIf, phJoint)->CreateLimit();
			}
			/*ボールジョイントの可動範囲の仕様が安定しないので保留
			if(typedb->KindOf(type, BalljointProp::id)){
				DCAST(PHBallJointIf, phJoint)->CreateConeLimit();
			}*/
		
			JointAux* jnt = new JointAux(phJoint, sockAux, plugAux);

			if(genericJoint){
				PHGenericJointIf* phGenJoint = phJoint->Cast();
				jnt->callback.hDll = hDll;
				jnt->callback.Init();
				phGenJoint->SetCallback(&jnt->callback);
				uint n = (uint)paramnames.size();
				for(uint i = 0; i < n; i++)
					phGenJoint->SetParam(paramnames[i].c_str(), paramvalues[i]);
			}

			RegAux(id, jnt);
			joints.push_back(jnt);

			treeReady = false;

			return SupportState::Supported;
		}
		else{
			Message::Out("joint type of %s not supported", name.c_str());
		}
	}
	else if(type == GearProp::id){
		// get joints
		int upId   = scene->GetLink(id, "up");
		int downId = scene->GetLink(id, "down");

		if(!IsValidID(upId) || IsIgnored(upId)){
			Message::Error("%s: invalid link to up joint", name.c_str());
			return SupportState::Ignored;
		}
		if(!typedb->KindOf(scene->GetObjectType(upId), Joint1DProp::id)){
			Message::Error("%s: up joint must be kind of Joint1D", name.c_str());
			return SupportState::Ignored;
		}
		if(!IsValidID(downId) || IsIgnored(downId)){
			Message::Error("%s: invalid link to down joint", name.c_str());
			return SupportState::Ignored;
		}
		if(!typedb->KindOf(scene->GetObjectType(downId), Joint1DProp::id)){
			Message::Error("%s: down joint must be kind of Joint1D", name.c_str());
			return SupportState::Ignored;
		}
		if(IsUndefined(upId) || IsUndefined(downId))
			return SupportState::Undefined;

		AUTO(JointAux*, upAux  , GetAux(upId  ));
		AUTO(JointAux*, downAux, GetAux(downId));

		PHGearIf* phGear = phScene->CreateGear(upAux->joint->Cast(), downAux->joint->Cast());
		Message::Extra("created PHGear for %s", name.c_str());

		GearAux* gear = new GearAux(phGear);
		RegAux(id, gear);
		gears.push_back(gear);

		return SupportState::Supported;
	}
	else if(type == GravityProp::id){
		return SupportState::Supported;
	}
	else if(type == ContactGroupProp::id){
		AUTO(ContactGroupProp*, conProp, prop);
		
		vector<int>    bodies;
		vector<int>    links;
		vector<string> names;
			
		// この階層以下のすべてのBody
		if(conProp->all_bodies){
			int parId = scene->GetParent(id);
			scene->EnumObjects(parId, BodyProp::id, typedb, links);
		}
		// リンクで参照されているBody
		else{
			scene->GetLinks(id, links, names);
		}

		for(uint i = 0; i < links.size(); i++){
			int dest = links[i];
			if(IsUndefined(dest))
				return SupportState::Undefined;
			if(IsIgnored(dest))
				continue;
			if(scene->GetObjectType(dest) != BodyProp::id)
				continue;
			bodies.push_back(dest);
		}

		ContactGroupAux* grpAux = new ContactGroupAux();
		for(uint i = 0; i < bodies.size(); i++){
			AUTO(BodyAux*, bodyAux, GetAux(bodies[i]));
			grpAux->bodies.push_back(bodyAux);
			bodyAux->groups.push_back(grpAux);
		}
		RegAux(id, grpAux);
		return SupportState::Supported;
	}
	return SupportState::Ignored;
}

void AdaptorSprPH::DeleteObject(int id){
	int type = scene->GetObjectType(id);
	Aux* aux = GetAux(id);

	if(type == BodyProp::id){
		AUTO(BodyAux*, bodyAux, aux);
		phScene->DelChildObject(bodyAux->solid);
		// PHSolidと一緒にPHJointも消される
		for(uint i = 0; i < bodyAux->cons.size(); i++){
			for(uint j = 0; j < bodyAux->cons[i]->joints.size(); j++)
				bodyAux->cons[i]->joints[j]->joint = 0;
		}

		RemoveFromArray(bodies, bodyAux);
		treeReady = false;
	}
	else if(type == ConnectorProp::id){

	}
	else if(typedb->KindOf(type, ShapeProp::id)){
		AUTO(ShapeAux*, shapeAux, aux);
		for(uint i = 0; i < shapeAux->shapes.size(); i++)
			phSdk->DelChildObject(shapeAux->shapes[i]);
	}
	else if(type == AttachProp::id){
		// 形状割当て解除
		AUTO(AttachAux*, attachAux, aux);

		if(attachAux->con){
			// 同じBodyに割り当てられている他のAttachに関してインデックスを修正
			if(attachAux->con->body){
				int N = attachAux->idxEnd - attachAux->idxBegin;
		
				for(uint i = 0; i < attachAux->con->body->cons.size(); i++){
					ConnectorAux* con = attachAux->con->body->cons[i];

					for(uint j = 0; j < con->attaches.size(); j++){
						AttachAux* att = con->attaches[j];

						if(att->idxBegin >= attachAux->idxEnd){
							att->idxBegin -= N;
							att->idxEnd   -= N;
						}
					}
				}
				// 
				attachAux->con->body->solid->RemoveShapes(attachAux->idxBegin, attachAux->idxEnd);
			}
		}
	}
	else if(typedb->KindOf(type, JointProp::id)){
		AUTO(JointAux*, jointAux, aux);
		if(jointAux->joint)
			phScene->DelChildObject(jointAux->joint);

		RemoveFromArray(joints, jointAux);
		treeReady = false;
	}
	else if(type == GearProp::id){
		AUTO(GearAux*, gearAux, aux);
		phScene->DelChildObject(gearAux->gear);

		RemoveFromArray(gears, gearAux);
	}
	else if(type == GravityProp::id){
		// 重力解除
		phScene->SetGravity(Vec3d());
	}

	RemoveAux(id);
}

void AdaptorSprPH::SyncObjectProperty(int id, bool download, int cat){
	if(!IsSupported(id))
		return;
	int type = scene->GetObjectType(id);
	Property* prop = scene->GetProperty(id);

	if(type == BodyProp::id){
		AUTO(BodyProp*, bodyProp, prop);
		AUTO(BodyAux*, bodyAux, GetAux(id));
		
		if(cat & AttrCategory::Param){
			if(download){
				bodyAux->auto_mass = bodyProp->auto_mass;
				bodyAux->auto_tree = bodyProp->auto_tree;

				// auto_mass無向時はプロパティから取得
				if(!bodyProp->auto_mass){
					bodyAux->solid->SetMass			(bodyProp->mass);
					bodyAux->solid->SetInertia		(bodyProp->inertia);
					bodyAux->solid->SetCenterOfMass	(bodyProp->center);
				}
				bodyAux->solid->SetDynamical (bodyProp->dynamical );
				bodyAux->solid->SetStationary(bodyProp->stationary);
			}
			else{
				bodyProp->mass       = bodyAux->solid->GetMass        ();
				bodyProp->inertia    = bodyAux->solid->GetInertia     ();
				bodyProp->center     = bodyAux->solid->GetCenterOfMass();
				bodyProp->dynamical  = bodyAux->solid->IsDynamical    ();
				bodyProp->stationary = bodyAux->solid->IsStationary   ();
			}		
		}
		if(cat & AttrCategory::State){
			Transform tr;
			int par = scene->FindAncestor(id, SpatialObjectProp::id, typedb);
			if(par != -1)
				scene->CalcRelativeTransform(-1, par, typedb, tr);
				
			if(download){
				bodyAux->solid->SetFramePosition  (tr.rot * bodyProp->trn + tr.trn);
				bodyAux->solid->SetOrientation    (tr.rot * bodyProp->rot);
				bodyAux->solid->SetVelocity       (bodyProp->vel);
				bodyAux->solid->SetAngularVelocity(bodyProp->angvel);
			}
			else{
				quat_t qinv = tr.rot.Conjugated();
				bodyProp->trn    = qinv * (bodyAux->solid->GetFramePosition() - tr.trn);
				bodyProp->rot    = qinv * bodyAux->solid->GetOrientation();
				bodyProp->vel    = bodyAux->solid->GetVelocity();
				bodyProp->angvel = bodyAux->solid->GetAngularVelocity();
			}		
		}
	}
	else if(type == ConnectorProp::id){
		AUTO(ConnectorProp*, conProp, prop);
		AUTO(ConnectorAux*, conAux, GetAux(id));

		if(cat & AttrCategory::State && download){
			// calculate relative pose between owner body
			Transform trans;
			scene->CalcRelativeTransform(conAux->body->id, id, typedb, trans);
			conAux->pose.Pos() = trans.trn;
			conAux->pose.Ori() = trans.rot;

			// call OnChange of Attach and Joint registered to this Connector
			for(uint i = 0; i < conAux->joints.size(); i++)
				conAux->joints[i]->OnChange(conAux);
			for(uint i = 0; i < conAux->attaches.size(); i++)
				conAux->attaches[i]->OnChange(conAux);
		}
	}
	else if(type == PhysicalMaterialProp::id){
		AUTO(MaterialAux*, matAux, GetAux(id));
		AUTO(PhysicalMaterialProp*, matProp, prop);

		if(cat & AttrCategory::Param){
			if(download){
				*matAux = *matProp;
				
				// call OnChange of attaches
				for(uint i = 0; i < matAux->attaches.size(); i++)
					matAux->attaches[i]->OnChange(matAux);
			}
		}
	}
	else if(typedb->KindOf(type, ShapeProp::id)){
		AUTO(ShapeAux*, shapeAux, GetAux(id));

		if(type == BoxProp::id){
			AUTO(BoxProp*, boxProp, prop);
			AUTO(CDBoxIf*, cdBox, shapeAux->shapes[0]);
			if(cat & AttrCategory::Param){
				if(download)
					cdBox->SetBoxSize(boxProp->size);
				else
					boxProp->size = cdBox->GetBoxSize();
			}
		}
		else if(type == SphereProp::id){
			AUTO(SphereProp*, sphereProp, prop);
			AUTO(CDSphereIf*, cdSphere, shapeAux->shapes[0]);
			if(cat & AttrCategory::Param){
				if(download)
					cdSphere->SetRadius((float)sphereProp->radius);
				else
					sphereProp->radius = cdSphere->GetRadius();
			}
		}
		else if(type == CapsuleProp::id){
			AUTO(CapsuleProp*, capsuleProp, prop);
			AUTO(CDCapsuleIf*, cdCapsule, shapeAux->shapes[0]);
			if(cat & AttrCategory::Param){
				if(download){
					cdCapsule->SetRadius((float)capsuleProp->radius);
					cdCapsule->SetLength((float)capsuleProp->height);
				}
				else{
					capsuleProp->radius = cdCapsule->GetRadius();
					capsuleProp->height = cdCapsule->GetLength();
				}
			}
		}
	}
	else if(typedb->KindOf(type, JointProp::id)){
		AUTO(JointProp*, jntProp, prop);
		AUTO(JointAux*, jointAux, GetAux(id));
		PHJointIf* joint = jointAux->joint->Cast();

		if(cat & AttrCategory::Param){
			if(download){
				joint->Enable(jntProp->enabled);
			}
			else{
				jntProp->enabled = joint->IsEnabled();
			}
		}

		if(typedb->KindOf(type, Joint1DProp::id)){
			AUTO(Joint1DProp*, joint1DProp, prop);
			PH1DJointIf* joint1D = joint->Cast();

			if(cat & AttrCategory::Param){
				if(download){
					joint1D->GetLimit()->SetRange(joint1DProp->range);
					joint1D->SetDamper(joint1DProp->damper);
					joint1D->SetSpring(joint1DProp->spring);
				}
				else{
					Vec2d range;
					joint1D->GetLimit()->GetRange(range);
					joint1DProp->range  = range;
					joint1DProp->damper = joint1D->GetDamper();
					joint1DProp->spring = joint1D->GetSpring();
				}
			}
			if(cat & AttrCategory::State){
				if(download){
					// 目標値を制約範囲内に収める
					real_t pos = joint1DProp->targetpos;
					real_t vel = joint1DProp->targetvel;
					pos = std::min(std::max(joint1DProp-> range[0], pos), joint1DProp-> range[1]);
					vel = std::min(std::max(joint1DProp->vrange[0], vel), joint1DProp->vrange[1]);
					joint1D->SetTargetPosition(pos);
					joint1D->SetTargetVelocity(vel);
				}
				else{
					joint1DProp->pos       = joint1D->GetPosition();
					joint1DProp->vel       = joint1D->GetVelocity();
					joint1DProp->targetpos = joint1D->GetTargetPosition();
					joint1DProp->targetvel = joint1D->GetTargetVelocity();
				}
			}

			if(type == HingeProp::id){
				AUTO(HingeProp*, hingeProp, joint1DProp);
				PHHingeJointIf* hinge = joint1D->Cast();
				// PHGenericJointの場合もある
				if(hinge){
					if(cat & AttrCategory::Param){
						if(download){
							hinge->SetCyclic(hingeProp->cyclic);
						}
						else{
							hingeProp->cyclic = hinge->IsCyclic();
						}
					}
				}
			}
		}
		if(type == BalljointProp::id){
			AUTO(BalljointProp*, ballProp, jntProp);
			PHBallJointIf* ball = joint->Cast();
			if(ball){
				if(cat & AttrCategory::Param){
					if(download){
						ball->SetDamper(ballProp->damper);
						ball->SetSpring(ballProp->spring);
					}
					else{
						ballProp->damper = ball->GetDamper();
						ballProp->spring = ball->GetSpring();
					}
				}
				if(cat & AttrCategory::State){
					if(download){
						ball->SetTargetPosition(ballProp->targetpos);
						ball->SetTargetVelocity(ballProp->targetvel);
					}
					else{
						ballProp->pos       = ball->GetPosition      ();
						ballProp->vel       = ball->GetVelocity      ();
						ballProp->targetpos = ball->GetTargetPosition();
						ballProp->targetvel = ball->GetTargetVelocity();
					}
				}
			}
		}
		if(type == SpringProp::id){
			AUTO(SpringProp*, sprProp, jntProp);
			PHSpringIf* spr = joint->Cast();
			if(spr){
				if(cat & AttrCategory::Param){
					if(download){
						double d = sprProp->damper;
						double k = sprProp->spring;
						spr->SetDamper(Vec3d(d,d,d));
						spr->SetSpring(Vec3d(k,k,k));
					}
					else{
						sprProp->damper = spr->GetDamper().norm();
						sprProp->spring = spr->GetSpring().norm();
					}
				}
			}
		}
		if(type == PointToPlaneProp::id){
			AUTO(PointToPlaneProp*, ptpProp, jntProp);
			PHPointToPlaneMateIf* ptp = joint->Cast();
			if(ptp){
				if(cat & AttrCategory::Param){
					if(download){
						ptp->SetRange(ptpProp->range);
					}
					else{
						ptp->GetRange(ptpProp->range);
					}
				}
			}
		}
	}
	else if(type == GearProp::id){
		AUTO(GearProp*, gearProp, prop);
		AUTO(GearAux* , gearAux, GetAux(id));
		if(cat & AttrCategory::Param){
			if(download){
				gearAux->gear->SetRatio (gearProp->ratio );
				gearAux->gear->SetOffset(gearProp->offset);
				if(gearProp->type == "vel")
					gearAux->gear->SetMode(PHGearDesc::MODE_VEL);
				if(gearProp->type == "pos")
					gearAux->gear->SetMode(PHGearDesc::MODE_POS);
			}
			else{
				gearProp->ratio  = gearAux->gear->GetRatio ();
				gearProp->offset = gearAux->gear->GetOffset();
				if(gearAux->gear->GetMode() == PHGearDesc::MODE_VEL)
					gearProp->type = string("vel");
				if(gearAux->gear->GetMode() == PHGearDesc::MODE_POS)
					gearProp->type = string("pos");
			}
		}
	}
	else if(type == GravityProp::id){
		AUTO(GravityProp*, gravProp, prop);
		if(cat & AttrCategory::Param){
			if(download)
				phScene->SetGravity(gravProp->accel);
			else
				gravProp->accel = phScene->GetGravity();
		}
	}
	else if(type == ContactGroupProp::id){
		AUTO(ContactGroupProp*, grpProp, prop);
		AUTO(ContactGroupAux*, grpAux, GetAux(id));
		if(cat & AttrCategory::Param){
			if(download){
				vector<PHSolidIf*> solids;
				for(uint i = 0; i < grpAux->bodies.size(); i++)
					solids.push_back(grpAux->bodies[i]->solid);
				if(!solids.empty())
					phScene->SetContactMode(&solids[0], solids.size(),
						grpProp->enable ? PHSceneDesc::MODE_LCP : PHSceneDesc::MODE_NONE);
			}
		}
	}
}

void AdaptorSprPH::Step(){
	phScene->Step();
}

void AdaptorSprPH::CreateTreeNodes(){
	// BodyのTreeNodeを再作成
	if(!treeReady){
		for(uint i = 0; i < bodies.size(); i++){
			if(bodies[i]->auto_tree && bodies[i]->treeNode){
				phScene->DelChildObject(bodies[i]->treeNode);
				bodies[i]->treeNode = 0;
			}
		}
		for(uint i = 0; i < bodies.size(); i++){
			if(bodies[i]->auto_tree){
				bodies[i]->treeNode = phScene->CreateTreeNodes(bodies[i]->solid);
			}
		}
		treeReady = true;
	}
}

}
