﻿#include <IK/sbik.h>
#include <sbmessage.h>
#include <sbrollpitchyaw.h>

#define AUTO(T, X, EXP) T X = static_cast<T>(EXP)

namespace Scenebuilder{;

AdaptorIK::BodyAux::BodyAux(IKBody* b){
	ikBody = b;
}

AdaptorIK::BodyAux::~BodyAux(){
	for(uint i = 0; i < cons.size(); i++)
		cons[i]->body = 0;
}

AdaptorIK::ConnectorAux::ConnectorAux(AdaptorIK::BodyAux* b){
	body = b;
	body->cons.push_back(this);
}

AdaptorIK::ConnectorAux::~ConnectorAux(){
	if(body)
		RemoveFromArray(body->cons, this);

	for(uint i = 0; i < joints.size(); i++){
		JointAux* jnt = joints[i];
		if(jnt->sock == this)
			jnt->sock = 0;
		if(jnt->plug == this)
			jnt->plug = 0;
		jnt->OnChange(this);
	}
	for(uint i = 0; i < handles.size(); i++){
		BodyHandleAux* handle = handles[i];
		if(handle->sock == this)
			handle->sock = 0;
		handle->OnChange(this);
	}

}

AdaptorIK::JointAux::JointAux(AdaptorIK::ConnectorAux* _sock, AdaptorIK::ConnectorAux* _plug, IKJointBase* _joint){
	sock    = _sock;
	plug    = _plug;
	ikJoint = _joint;
	sock->joints.push_back(this);
	plug->joints.push_back(this);
}

AdaptorIK::JointAux::~JointAux(){
	if(sock)
		RemoveFromArray(sock->joints, this);
	if(plug)
		RemoveFromArray(plug->joints, this);
}

AdaptorIK::BodyHandleAux::BodyHandleAux(IKBodyHandle* _handle, AdaptorIK::ConnectorAux* _sock){
	ikBodyHandle = _handle;
	sock     = _sock;
	sock->handles.push_back(this);
}

AdaptorIK::BodyHandleAux::~BodyHandleAux(){
	if(sock)
		RemoveFromArray(sock->handles, this);
}

AdaptorIK::JointHandleAux::JointHandleAux(IKJointHandle* _handle){
	ikJointHandle = _handle;
}

AdaptorIK::JointHandleAux::~JointHandleAux(){
}

AdaptorIK::JointSyncAux::JointSyncAux(IKJointSync* _sync){
	ikJointSync = _sync;
}

AdaptorIK::JointSyncAux::~JointSyncAux(){
}

AdaptorIK::ComHandleAux::ComHandleAux(IKComHandle* _handle){
	ikComHandle = _handle;
}

AdaptorIK::ComHandleAux::~ComHandleAux(){
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void AdaptorIK::JointAux::OnChange(Aux* caller){
	if(caller == sock)
		ikJoint->SetSocketPose(sock->pose);
	if(caller == plug)
		ikJoint->SetPlugPose(plug->pose);
}

void AdaptorIK::BodyHandleAux::OnChange(Aux* caller){
	if(caller == sock)
		ikBodyHandle->SetSocketPose(sock->pose);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorIK::AdaptorIK(){
}

IKBody* AdaptorIK::GetBody(int id){
	AUTO(BodyAux*, bodyAux, HandleByID(id, BodyProp::id));
	return bodyAux->ikBody;
}

IKBody* AdaptorIK::GetBody(string name){
	AUTO(BodyAux*, bodyAux, HandleByName(name, BodyProp::id));
	return bodyAux->ikBody;
}

IKJointBase* AdaptorIK::GetJoint(int id){
	AUTO(JointAux*, jntAux, HandleByID(id, JointProp::id));
	return jntAux->ikJoint;
}

IKJointBase* AdaptorIK::GetJoint(string name){
	AUTO(JointAux*, jntAux, HandleByName(name, JointProp::id));
	return jntAux->ikJoint;
}

IKBodyHandle*	AdaptorIK::GetBodyHandle(int id){
	AUTO(BodyHandleAux*, handleAux, HandleByID(id, IKProp::id));
	return handleAux->ikBodyHandle;
}

IKBodyHandle*	AdaptorIK::GetBodyHandle(string name){
	AUTO(BodyHandleAux*, handleAux, HandleByName(name, IKProp::id));
	return handleAux->ikBodyHandle;
}

IKJointHandle*	AdaptorIK::GetJointHandle(int id){
	AUTO(JointHandleAux*, handleAux, HandleByID(id, IKJointProp::id));
	return handleAux->ikJointHandle;
}

IKJointHandle*	AdaptorIK::GetJointHandle(string name){
	AUTO(JointHandleAux*, handleAux, HandleByName(name, IKJointProp::id));
	return handleAux->ikJointHandle;
}

IKJointSync*	AdaptorIK::GetJointSync(int id){
	AUTO(JointSyncAux*, handleAux, HandleByID(id, IKSyncProp::id));
	return handleAux->ikJointSync;
}

IKJointSync*	AdaptorIK::GetJointSync(string name){
	AUTO(JointSyncAux*, handleAux, HandleByName(name, IKSyncProp::id));
	return handleAux->ikJointSync;
}

IKComHandle* AdaptorIK::GetComHandle(int id){
	AUTO(ComHandleAux*, handleAux, HandleByID(id, IKComProp::id));
	return handleAux->ikComHandle;
}

IKComHandle* AdaptorIK::GetComHandle(string name){
	AUTO(ComHandleAux*, handleAux, HandleByName(name, IKComProp::id));
	return handleAux->ikComHandle;
}

int AdaptorIK::CreateObject(int id){
	int       type = scene->GetObjectType(id);
	Property* prop = scene->GetProperty  (id);
	string    name = scene->GetName      (id);
	
	if(type == BodyProp::id){
		BodyAux* body = new BodyAux(ikSolver.AddBody(name));
		RegAux(id, body);
		bodies.push_back(body);
		
		Message::Extra("created IKBody for %s", name.c_str());
		return SupportState::Supported;
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

		int  jntType   = -1;
		int  mateType  = -1;
		int  limitType = -1;
		if(type == HingeProp::id){
			jntType = IKJoint::Type::Hinge;
		}
		else if(type == SliderProp::id){
			jntType = IKJoint::Type::Slider;
		}
		else if(type == UniversaljointProp::id){
			jntType = IKJoint::Type::Universaljoint;
		}
		else if(type == BalljointProp::id){
			jntType = IKJoint::Type::Balljoint;
		}
		else if(type == FixjointProp::id){
			jntType = IKJoint::Type::Fixjoint;
		}
		else if(type == FreejointProp::id){
			jntType = IKJoint::Type::Freejoint;
		}
		else if(type == PointToPointProp::id){
			mateType = IKMate::Type::PointToPoint;
		}
		else if(type == PointToLineProp::id){
			mateType = IKMate::Type::PointToLine;
		}
		else if(type == PointToPlaneProp::id){
			mateType = IKMate::Type::PointToPlane;
		}
		else if(type == DistanceProp::id){
			mateType = IKMate::Type::Distance;
		}
		else if(type == ConicLimitProp::id){
			limitType = IKLimit::Type::Conic;
		}
		else{
			Message::Error("%s: unsupported joint type", name.c_str());
			return SupportState::Ignored;
		}

		AUTO(ConnectorAux*, sockAux, GetAux(sockId));
		AUTO(ConnectorAux*, plugAux, GetAux(plugId));

		//
		IKJointBase* ikJoint;
		if(jntType != -1)
			ikJoint = ikSolver.AddJoint(jntType, name);
		if(mateType != -1)
			ikJoint = ikSolver.AddMate(mateType, name);
		if(limitType != -1)
			ikJoint = ikSolver.AddLimit(limitType, name);
		
		// 親子関係を設定
		IKBody* sockBody = sockAux->body->ikBody;
		IKBody* plugBody = plugAux->body->ikBody;

		ikJoint->SetSocketBody(sockBody);
		ikJoint->SetPlugBody  (plugBody);

		if(jntType != -1)
			plugBody->SetParent(sockBody, (IKJoint*)ikJoint);

		JointAux* jntAux = new JointAux(sockAux, plugAux, ikJoint);
		RegAux(id, jntAux);
		joints.push_back(jntAux);

		return SupportState::Supported;
	}
	else if(type == IKProp::id){
		// get connectors
		int sockId = scene->GetLink(id, "sock");
		
		if(!IsValidID(sockId) || IsIgnored(sockId)){
			Message::Error("%s: invalid link to sock", name.c_str());
			return SupportState::Ignored;
		}
		if(scene->GetObjectType(sockId) != ConnectorProp::id){
			Message::Error("%s: sock must link to Connector", name.c_str());
			return SupportState::Ignored;
		}
		if(IsUndefined(sockId))
			return SupportState::Undefined;

		AUTO(ConnectorAux*, sockAux, GetAux(sockId));
		
		IKBody* sockBody = sockAux->body->ikBody;
		
		BodyHandleAux* bodyHandleAux = new BodyHandleAux(ikSolver.AddBodyHandle(sockBody, name), sockAux);
		RegAux(id, bodyHandleAux);
		bodyHandles.push_back(bodyHandleAux);

		return SupportState::Supported;
	}
	else if(type == IKJointProp::id){
		// get connectors
		int jntId = scene->GetLink(id, "path");
		
		if(!IsValidID(jntId) || IsIgnored(jntId)){
			Message::Error("%s: invalid link to joint", name.c_str());
			return SupportState::Ignored;
		}
		if(!typedb->KindOf(scene->GetObjectType(jntId), JointProp::id)){
			Message::Error("%s: joint must link to Joint", name.c_str());
			return SupportState::Ignored;
		}
		if(IsUndefined(jntId))
			return SupportState::Undefined;

		AUTO(JointAux*, jntAux, GetAux(jntId));
		
		JointHandleAux* handleAux = new JointHandleAux(ikSolver.AddJointHandle((IKJoint*)jntAux->ikJoint, name));
		RegAux(id, handleAux);
		jointHandles.push_back(handleAux);

		return SupportState::Supported;
	}
	else if(type == IKSyncProp::id){
		int jntId0 = scene->GetLink(id, "path0");
		int jntId1 = scene->GetLink(id, "path1");
		
		if( !IsValidID(jntId0) || IsIgnored(jntId0) ||
			!IsValidID(jntId1) || IsIgnored(jntId1) ){
			Message::Error("%s: invalid link to joint", name.c_str());
			return SupportState::Ignored;
		}
		if( !typedb->KindOf(scene->GetObjectType(jntId0), JointProp::id) ||
			!typedb->KindOf(scene->GetObjectType(jntId1), JointProp::id) ){
			Message::Error("%s: joint must link to Joint", name.c_str());
			return SupportState::Ignored;
		}
		if( IsUndefined(jntId0) ||
			IsUndefined(jntId1) )
			return SupportState::Undefined;

		AUTO(JointAux*, jntAux0, GetAux(jntId0));
		AUTO(JointAux*, jntAux1, GetAux(jntId1));
		
		JointSyncAux* syncAux = new JointSyncAux(ikSolver.AddJointSync((IKJoint*)jntAux0->ikJoint, (IKJoint*)jntAux1->ikJoint, name));
		RegAux(id, syncAux);
		jointSyncs.push_back(syncAux);

		return SupportState::Supported;
	}
	else if(type == IKComProp::id){
		vector<int>    bodies;
		vector<int>    links;
		vector<string> names;
		
		scene->GetLinks(id, links, names);

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

		ComHandleAux* comHandleAux = new ComHandleAux(ikSolver.AddComHandle(name));
		for(uint i = 0; i < bodies.size(); i++){
			AUTO(BodyAux*, bodyAux, GetAux(bodies[i]));
			comHandleAux->bodies.push_back(bodyAux);
			comHandleAux->ikComHandle->AddBody(bodyAux->ikBody);
		}
		RegAux(id, comHandleAux);
		comHandles.push_back(comHandleAux);

		return SupportState::Supported;
	}
	return SupportState::Ignored;
}

void AdaptorIK::DeleteObject(int id){
	int type = scene->GetObjectType(id);
	Aux* aux = GetAux(id);

	if(type == BodyProp::id){
		AUTO(BodyAux*, bodyAux, aux);
		ikSolver.DeleteBody(bodyAux->ikBody);
		RemoveFromArray(bodies, bodyAux);
	}
	else if(type == ConnectorProp::id){

	}
	else if(typedb->KindOf(type, JointProp::id)){
		AUTO(JointAux*, jointAux, aux);
		RemoveFromArray(joints, jointAux);
	}
	else if(type == IKProp::id){
		AUTO(BodyHandleAux*, bodyHandleAux, aux);
		ikSolver.DeleteBodyHandle(bodyHandleAux->ikBodyHandle);
		RemoveFromArray(bodyHandles, bodyHandleAux);
	}
	else if(type == IKJointProp::id){
		AUTO(JointHandleAux*, jntHandleAux, aux);
		ikSolver.DeleteJointHandle(jntHandleAux->ikJointHandle);
		RemoveFromArray(jointHandles, jntHandleAux);
	}
	else if(type == IKSyncProp::id){
		AUTO(JointSyncAux*, jntSyncAux, aux);
		ikSolver.DeleteJointSync(jntSyncAux->ikJointSync);
		RemoveFromArray(jointSyncs, jntSyncAux);
	}
	else if(type == IKComProp::id){
		AUTO(ComHandleAux*, comHandleAux, aux);
		ikSolver.DeleteComHandle(comHandleAux->ikComHandle);
		RemoveFromArray(comHandles, comHandleAux);
	}
	RemoveAux(id);
}

void AdaptorIK::SyncObjectProperty(int id, bool download, int cat){
	if(!IsSupported(id))
		return;
	int type = scene->GetObjectType(id);
	Property* prop = scene->GetProperty(id);

	if(type == BodyProp::id){
		AUTO(BodyProp*, bodyProp, prop      );
		AUTO(BodyAux* , bodyAux , GetAux(id));
		
		if(cat & AttrCategory::Param){
			if(download){
				bodyAux->ikBody->mass    = bodyProp->mass;
				bodyAux->ikBody->inertia = bodyProp->inertia;
				bodyAux->ikBody->center  = bodyProp->center;
			}
			else{
				bodyProp->mass    = bodyAux->ikBody->mass;
				bodyProp->inertia = bodyAux->ikBody->inertia;
				bodyProp->center  = bodyAux->ikBody->center;
			}		
		}
		if(cat & AttrCategory::State){
			Transform tr;
			int par = scene->FindAncestor(id, SpatialObjectProp::id, typedb);
			if(par != -1)
				scene->CalcRelativeTransform(-1, par, typedb, tr);
				
			if(download){
				bodyAux->ikBody->SetInitialPose(pose_t(tr.rot * bodyProp->trn + tr.trn, tr.rot * bodyProp->rot));
			}
			else{
				quat_t qinv = tr.rot.Conjugated();
				pose_t pose;
				bodyAux->ikBody->GetPose(pose);
				bodyProp->trn = qinv * (pose.Pos() - tr.trn);
				bodyProp->rot = qinv *  pose.Ori();
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

			for(uint i = 0; i < conAux->joints.size(); i++)
				conAux->joints[i]->OnChange(conAux);
			for(uint i = 0; i < conAux->handles.size(); i++)
				conAux->handles[i]->OnChange(conAux);
		}
	}
	else if(typedb->KindOf(type, JointProp::id)){
		AUTO(JointProp*, jntProp, prop);
		AUTO(JointAux*, jointAux, GetAux(id));

		if(typedb->KindOf(type, Joint1DProp::id)){
			AUTO(Joint1DProp*, joint1DProp, prop);
			IKJoint* ikJoint = ((IKJoint*)jointAux->ikJoint);

			if(cat & AttrCategory::Param){
				if(download){
					ikJoint->SetPosLimit(0, joint1DProp-> range[0], joint1DProp-> range[1]);
					ikJoint->SetVelLimit(0, joint1DProp->vrange[0], joint1DProp->vrange[1]);
				}
			}
			if(cat & AttrCategory::State){
				if(download){
					// IK計算の初期値に設定
					ikJoint->SetInitialPos(0, joint1DProp->pos);
					ikJoint->SetInitialVel(0, joint1DProp->vel);
				}
				else{
					// IKの計算結果をtargetposに代入
					//joint1DProp->targetpos = ikJoint->GetPos(0);
					joint1DProp->pos = ikJoint->GetPos(0);
				}
			}
		}
		if(typedb->KindOf(type, UniversaljointProp::id)){
			AUTO(UniversaljointProp*, uniProp, prop);
			IKJoint* ikJoint = ((IKJoint*)jointAux->ikJoint);
				
			if(cat & AttrCategory::State){
				if(download){
					// IK計算の初期値に設定
					ikJoint->SetInitialPos(0, uniProp->pos[0]);
					ikJoint->SetInitialPos(1, uniProp->pos[1]);
				}
			}
		}
		if(typedb->KindOf(type, BalljointProp::id)){
			AUTO(BalljointProp*, ballProp, prop);
			IKJoint* ikJoint = ((IKJoint*)jointAux->ikJoint);
				
			vec3_t angle;
			if(cat & AttrCategory::State){
				if(download){
					// IK計算の初期値に設定
					angle = ToRollPitchYaw(ballProp->pos);
					ikJoint->SetInitialPos(0, angle[0]);
					ikJoint->SetInitialPos(1, angle[1]);
					ikJoint->SetInitialPos(2, angle[2]);
				}
				else{
					// IKの計算結果をtargetposに代入
					angle[0] = ikJoint->GetPos(0);
					angle[1] = ikJoint->GetPos(1);
					angle[2] = ikJoint->GetPos(2);
					ballProp->targetpos = FromRollPitchYaw(angle);
				}
			}
		}
		if(typedb->KindOf(type, FreejointProp::id)){
			AUTO(FreejointProp*, freeProp, prop);
			IKJoint* ikJoint = ((IKJoint*)jointAux->ikJoint);
				
			vec3_t angle;
			if(cat & AttrCategory::State){
				if(download){
					// IK計算の初期値に設定
					angle = ToRollPitchYaw(freeProp->ori);
					ikJoint->SetInitialPos(0, freeProp->pos[0]);
					ikJoint->SetInitialPos(1, freeProp->pos[1]);
					ikJoint->SetInitialPos(2, freeProp->pos[2]);
					ikJoint->SetInitialPos(3, angle[0]);
					ikJoint->SetInitialPos(4, angle[1]);
					ikJoint->SetInitialPos(5, angle[2]);
				}
			}
		}
		if(typedb->KindOf(type, PointToPlaneProp::id)){
			AUTO(PointToPlaneProp*, ptpProp, prop);
			IKMate* ikMate = ((IKMate*)jointAux->ikJoint);
		
			if(cat & AttrCategory::Param){
				if(download){
					ikMate->rangeMin.z = ptpProp->range[0];
					ikMate->rangeMax.z = ptpProp->range[1];
				}
			}
		}
		if(typedb->KindOf(type, DistanceProp::id)){
			AUTO(DistanceProp*, distProp, prop);
			IKMate* ikMate = ((IKMate*)jointAux->ikJoint);
		
			if(cat & AttrCategory::Param){
				if(download){
					ikMate->distance = distProp->distance;
				}
			}
		}
		if(typedb->KindOf(type, ConicLimitProp::id)){
			AUTO(ConicLimitProp*, coneProp, prop);
			IKLimit* ikLimit = ((IKLimit*)jointAux->ikJoint);
		
			if(cat & AttrCategory::Param){
				if(download){
					ikLimit->angle = coneProp->angle;
				}
			}
		}
	}
	else if(type == IKProp::id){
		AUTO(IKProp*, ikProp, prop);
		AUTO(BodyHandleAux*, bodyHandleAux, GetAux(id));
		if(cat & AttrCategory::Param){
			if(download){
				bodyHandleAux->ikBodyHandle->EnablePos(ikProp->enable_trn);
				bodyHandleAux->ikBodyHandle->EnableOri(ikProp->enable_rot);
			}
		}
		if(cat & AttrCategory::State){
			if(download){
				bodyHandleAux->ikBodyHandle->SetDesiredPos(ikProp->trn);
				bodyHandleAux->ikBodyHandle->SetDesiredOri(ikProp->rot);
			}
			else{
			}
		}
	}
	else if(type == IKSyncProp::id){
		AUTO(IKSyncProp*, ikSyncProp, prop);
		AUTO(JointSyncAux*, jointSyncAux, GetAux(id));
		if(cat & AttrCategory::Param){
			if(download){
				jointSyncAux->ikJointSync->SetRatio(0, ikSyncProp->ratio);
			}
		}
	}
	else if(type == IKComProp::id){
		AUTO(IKComProp*, ikComProp, prop);
		AUTO(ComHandleAux*, comHandleAux, GetAux(id));
		if(cat & AttrCategory::Param){
			if(download){
				comHandleAux->ikComHandle->EnablePos(ikComProp->enable);
			}
		}
	}
}

}
