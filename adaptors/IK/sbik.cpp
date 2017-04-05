#include <IK/sbik.h>
#include <sbmessage.h>

#define AUTO(T, X, EXP) T X = static_cast<T>(EXP)

namespace Scenebuilder{;

AdaptorIK::BodyAux::BodyAux(IKBody* b){
	body = b;
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
		if(handle->end == this)
			handle->end = 0;
		if(handle->ref== this)
			handle->ref = 0;
		handle->OnChange(this);
	}

}
AdaptorIK::JointAux::JointAux(AdaptorIK::ConnectorAux* s, AdaptorIK::ConnectorAux* p){
	sock  = s;
	plug  = p;
	sock->joints.push_back(this);
	plug->joints.push_back(this);
}
AdaptorIK::JointAux::~JointAux(){
	if(sock)
		RemoveFromArray(sock->joints, this);
	if(plug)
		RemoveFromArray(plug->joints, this);
}
AdaptorIK::HandleAux::HandleAux(IKHandle* h, AdaptorIK::ConnectorAux* e, AdaptorIK::ConnectorAux* r){
	handle = h;
	end    = e;
	ref    = r;
	end->handles.push_back(this);
	ref->handles.push_back(this);
}
AdaptorIK::HandleAux::~HandleAux(){
	if(end)
		RemoveFromArray(end->handles, this);
	if(ref)
		RemoveFromArray(ref->handles, this);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void AdaptorIK::JointAux::OnChange(Aux* caller){
	if(!sock || !plug)
		return;
	IKBody* sockBody = sock->body->body;
	IKBody* plugBody = plug->body->body;
	if(plugBody->GetParent() == sockBody){
		if(caller == sock)
			plugBody->SetSocketPose(sock->pose);
		if(caller == plug)
			plugBody->SetPlugPose(plug->pose);
	}
}

void AdaptorIK::HandleAux::OnChange(Aux* caller){
	if(!end || !ref)
		return;
	if(caller == end)
		handle->SetEndPose(end->pose);
	if(caller == ref)
		handle->SetRefPose(ref->pose);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorIK::AdaptorIK(){
}

IKBody* AdaptorIK::GetBody(int id){
	AUTO(BodyAux*, bodyAux, HandleByID(id, BodyProp::id));
	return bodyAux->body;
}

IKBody* AdaptorIK::GetBody(string name){
	AUTO(BodyAux*, bodyAux, HandleByName(name, BodyProp::id));
	return bodyAux->body;
}

IKHandle*	AdaptorIK::GetHandle(int id){
	AUTO(HandleAux*, handleAux, HandleByID(id, IKProp::id));
	return handleAux->handle;
}

IKHandle*	AdaptorIK::GetHandle(string name){
	AUTO(HandleAux*, handleAux, HandleByName(name, IKProp::id));
	return handleAux->handle;
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

		int jntType;
		if(type == HingeProp::id)
			jntType = IKBody::JointType::Hinge;
		else if(type == SliderProp::id)
			jntType = IKBody::JointType::Slider;
		else if(type == BalljointProp::id)
			jntType = IKBody::JointType::Balljoint;
		else if(type == FixjointProp::id)
			jntType = IKBody::JointType::Fixjoint;
		else{
			Message::Error("%s: unsupported joint type", name.c_str());
			return SupportState::Ignored;
		}

		AUTO(ConnectorAux*, sockAux, GetAux(sockId));
		AUTO(ConnectorAux*, plugAux, GetAux(plugId));
		
		// 親子関係を設定
		IKBody* sockBody = sockAux->body->body;
		IKBody* plugBody = plugAux->body->body;
		if(!plugBody->parent){
			plugBody->SetParent(sockBody, jntType);
		}

		JointAux* jntAux = new JointAux(sockAux, plugAux);
		RegAux(id, jntAux);
		joints.push_back(jntAux);

		return SupportState::Supported;
	}
	else if(type == IKProp::id){
		// get connectors
		int endId = scene->GetLink(id, "end");
		int refId = scene->GetLink(id, "ref");

		if(!IsValidID(endId) || IsIgnored(endId)){
			Message::Error("%s: invalid link to end", name.c_str());
			return SupportState::Ignored;
		}
		if(scene->GetObjectType(endId) != ConnectorProp::id){
			Message::Error("%s: end must link to Connector", name.c_str());
			return SupportState::Ignored;
		}
		if(!IsValidID(refId) || IsIgnored(refId)){
			Message::Error("%s: invalid link to ref", name.c_str());
			return SupportState::Ignored;
		}
		if(scene->GetObjectType(refId) != ConnectorProp::id){
			Message::Error("%s: ref must link to Connector", name.c_str());
			return SupportState::Ignored;
		}
		if(IsUndefined(refId) || IsUndefined(refId))
			return SupportState::Undefined;

		AUTO(ConnectorAux*, endAux, GetAux(endId));
		AUTO(ConnectorAux*, refAux, GetAux(refId));
		
		IKBody* endBody = endAux->body->body;
		IKBody* refBody = refAux->body->body;
		
		HandleAux* handleAux = new HandleAux(ikSolver.AddHandle(endBody, refBody), endAux, refAux);
		RegAux(id, handleAux);
		handles.push_back(handleAux);

		return SupportState::Supported;
	}
	return SupportState::Ignored;
}

void AdaptorIK::DeleteObject(int id){
	int type = scene->GetObjectType(id);
	Aux* aux = GetAux(id);

	if(type == BodyProp::id){
		AUTO(BodyAux*, bodyAux, aux);
		ikSolver.DeleteBody(bodyAux->body);
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
		ikSolver.DeleteHandle(handleAux->handle);
		RemoveFromArray(handles, handleAux);
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
			}
			else{
			}		
		}
		if(cat & AttrCategory::State){
			Transform tr;
			int par = scene->FindAncestor(id, SpatialObjectProp::id, typedb);
			if(par != -1)
				scene->CalcRelativeTransform(-1, par, typedb, tr);
				
			if(download){
				bodyAux->body->SetPose(pose_t(tr.rot * bodyProp->trn + tr.trn, tr.rot * bodyProp->rot));
			}
			else{

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
			IKBody* plugBody = jointAux->plug->body->body;
				
			if(cat & AttrCategory::Param){
				if(download){
					//plugBody->SetPosLimit(0, joint1DProp->range[0], joint1DProp->range[1]);
				}
			}
			if(cat & AttrCategory::State){
				if(download){
					plugBody->SetJointPos(0, joint1DProp->pos);
				}
				else{
					// IKの計算結果をtargetposに代入
					if(plugBody->handled)
						joint1DProp->targetpos = plugBody->GetJointPos(0);
				}
			}
		}
	}
	else if(type == IKProp::id){
		AUTO(IKProp*, ikProp, prop);
		AUTO(HandleAux*, handleAux, GetAux(id));
		if(cat & AttrCategory::Param){
			if(download){
				handleAux->handle->EnablePos(ikProp->enable_trn);
				handleAux->handle->EnableOri(ikProp->enable_rot);
			}
		}
		if(cat & AttrCategory::State){
			if(download){
				handleAux->handle->SetDesiredPose(pose_t(ikProp->trn, ikProp->rot));
			}
			else{
				pose_t pose;
				handleAux->handle->GetCurrentPose(pose);
				ikProp->trn = pose.Pos();
				ikProp->rot = pose.Ori();
			}
		}
	}
}


}
