#include <IK/sbik.h>
#include <sbmessage.h>

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
		HandleAux* handle = handles[i];
		if(handle->sock == this)
			handle->sock = 0;
		handle->OnChange(this);
	}

}

AdaptorIK::JointAux::JointAux(AdaptorIK::ConnectorAux* _sock, AdaptorIK::ConnectorAux* _plug, IKJoint* _joint){
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

AdaptorIK::HandleAux::HandleAux(IKHandle* _handle, AdaptorIK::ConnectorAux* _sock){
	ikHandle = _handle;
	sock     = _sock;
	sock->handles.push_back(this);
}

AdaptorIK::HandleAux::~HandleAux(){
	if(sock)
		RemoveFromArray(sock->handles, this);
}

AdaptorIK::JointHandleAux::JointHandleAux(IKJointHandle* _handle){
	ikJointHandle = _handle;
}

AdaptorIK::JointHandleAux::~JointHandleAux(){
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

void AdaptorIK::HandleAux::OnChange(Aux* caller){
	if(caller == sock)
		ikHandle->SetSocketPose(sock->pose);
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

IKJoint* AdaptorIK::GetJoint(int id){
	AUTO(JointAux*, jntAux, HandleByID(id, JointProp::id));
	return jntAux->ikJoint;
}

IKJoint* AdaptorIK::GetJoint(string name){
	AUTO(JointAux*, jntAux, HandleByName(name, JointProp::id));
	return jntAux->ikJoint;
}

IKHandle*	AdaptorIK::GetHandle(int id){
	AUTO(HandleAux*, handleAux, HandleByID(id, IKProp::id));
	return handleAux->ikHandle;
}

IKHandle*	AdaptorIK::GetHandle(string name){
	AUTO(HandleAux*, handleAux, HandleByName(name, IKProp::id));
	return handleAux->ikHandle;
}

IKJointHandle*	AdaptorIK::GetJointHandle(int id){
	AUTO(JointHandleAux*, handleAux, HandleByID(id, IKJointProp::id));
	return handleAux->ikJointHandle;
}

IKJointHandle*	AdaptorIK::GetJointHandle(string name){
	AUTO(JointHandleAux*, handleAux, HandleByName(name, IKJointProp::id));
	return handleAux->ikJointHandle;
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
		BodyAux* body = new BodyAux(ikSolver.AddBody());
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

		int  jntType;
		if(type == HingeProp::id){
			jntType = IKJoint::Type::Hinge;
		}
		else if(type == SliderProp::id){
			jntType = IKJoint::Type::Slider;
		}
		else if(type == BalljointProp::id){
			jntType = IKJoint::Type::Balljoint;
		}
		else if(type == FixjointProp::id){
			jntType = IKJoint::Type::Fixjoint;
		}
		else if(type == LineToLineProp::id){
			jntType = IKJoint::Type::LineToLine;
		}
		else{
			Message::Error("%s: unsupported joint type", name.c_str());
			return SupportState::Ignored;
		}

		AUTO(ConnectorAux*, sockAux, GetAux(sockId));
		AUTO(ConnectorAux*, plugAux, GetAux(plugId));

		//
		IKJoint* ikJoint = ikSolver.AddJoint(jntType);
		
		// 親子関係を設定
		IKBody* sockBody = sockAux->body->ikBody;
		IKBody* plugBody = plugAux->body->ikBody;

		ikJoint->SetSocketBody(sockBody);
		ikJoint->SetPlugBody  (plugBody);

		if( typedb->KindOf(type, Joint1DProp  ::id) ||
			typedb->KindOf(type, BalljointProp::id) ||
			typedb->KindOf(type, FixjointProp ::id) )
			plugBody->SetParent(sockBody, ikJoint);

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
		
		HandleAux* handleAux = new HandleAux(ikSolver.AddHandle(sockBody), sockAux);
		RegAux(id, handleAux);
		handles.push_back(handleAux);

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
		
		JointHandleAux* handleAux = new JointHandleAux(ikSolver.AddJointHandle(jntAux->ikJoint));
		RegAux(id, handleAux);
		jointHandles.push_back(handleAux);

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

		ComHandleAux* comHandleAux = new ComHandleAux(ikSolver.AddComHandle());
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
		AUTO(HandleAux*, handleAux, aux);
		ikSolver.DeleteHandle(handleAux->ikHandle);
		RemoveFromArray(handles, handleAux);
	}
	else if(type == IKJointProp::id){
		AUTO(JointHandleAux*, jntHandleAux, aux);
		ikSolver.DeleteJointHandle(jntHandleAux->ikJointHandle);
		RemoveFromArray(jointHandles, jntHandleAux);
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
				bodyAux->ikBody->SetPose(pose_t(tr.rot * bodyProp->trn + tr.trn, tr.rot * bodyProp->rot));
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
				
			if(cat & AttrCategory::Param){
				if(download){
					jointAux->ikJoint->SetPosLimit(0, joint1DProp-> range[0], joint1DProp-> range[1]);
					jointAux->ikJoint->SetVelLimit(0, joint1DProp->vrange[0], joint1DProp->vrange[1]);
				}
			}
			if(cat & AttrCategory::State){
				if(download){
					// IK計算の初期値に設定
					jointAux->ikJoint->SetInitialPos(0, joint1DProp->pos);
					jointAux->ikJoint->SetInitialVel(0, joint1DProp->vel);
				}
				else{
					// IKの計算結果をtargetposに代入
					joint1DProp->targetpos = jointAux->ikJoint->GetPos(0);
				}
			}
		}
	}
	else if(type == IKProp::id){
		AUTO(IKProp*, ikProp, prop);
		AUTO(HandleAux*, handleAux, GetAux(id));
		if(cat & AttrCategory::Param){
			if(download){
				handleAux->ikHandle->EnablePos(ikProp->enable_trn);
				handleAux->ikHandle->EnableOri(ikProp->enable_rot);
			}
		}
		if(cat & AttrCategory::State){
			if(download){
				handleAux->ikHandle->SetDesiredPos(ikProp->trn);
				handleAux->ikHandle->SetDesiredOri(ikProp->rot);
			}
			else{
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
