#include <sbadaptor.h>
#include <sbmessage.h>
#include <sbmodel.h>

namespace Scenebuilder{;

Adaptor::Adaptor(){
	scene = 0;
}

Adaptor::~Adaptor(){

}

bool Adaptor::IsValidID(int id){
	return 0 <= id && id < (int)alive.size();
}

bool Adaptor::IsSupported(int id){
	if(!IsValidID(id))
		return false;
	return state[id] == SupportState::Supported;
}

bool Adaptor::IsIgnored(int id){
	if(!IsValidID(id))
		return true;
	return state[id] == SupportState::Ignored;
}

bool Adaptor::IsUndefined(int id){
	if(!IsValidID(id))
		return true;
	return state[id] == SupportState::Undefined;
}

void Adaptor::RegAux(int id, Adaptor::Aux* aux){
	if(IsValidID(id)){
		handle[id] = aux;
		aux->adaptor = this;
		aux->id = id;
	}
}

void Adaptor::RemoveAux(int id){
	if(!IsValidID(id))
		return;

	// ハンドルを削除
	handle[id] = 0;
}

Adaptor::Aux* Adaptor::GetAux(int id){
	if(IsValidID(id))
		return handle[id];
	return 0;
}

void Adaptor::Set(SceneBase* s, TypeDB* db, ModelContainer* m, int id){
	scene  = s;
	typedb = db;
	models = m;
	rootId = (id == -1 ? scene->GetRoot() : id);
	alive .clear();
	handle.clear();
}

void Adaptor::Mirror(){
	if(!scene)
		return;
	scene->GetLivenessArray(addr, alive);

	// 配列長がオブジェクト総数より短いならresize
	size_t n = alive.size();
	if(handle.size() < n){
		handle  .resize(n);
		state   .resize(n, SupportState::Undefined);
		toCreate.resize(n);
		toDelete.resize(n);
	}

	for(uint i = 0; i < n; i++){
		// alive and support state is undefined -> attempt to create
		toCreate[i] = (alive[i] && state[i] == SupportState::Undefined);

		// not alive and supported -> attempt to delete
		toDelete[i] = (!alive[i] && state[i] == SupportState::Supported);
	}

	for(uint i = 0; i < n; i++){
		if(toDelete[i]){
			DeleteObject(i);
			handle[i] = 0;
		}
		if(!alive[i])
			state[i] = SupportState::Undefined;
	}

	// this loop iterates until no call to CreateObject returns UNDEFINED
	const uint maxLoop = 10;
	uint l;
	bool undef;
	for(l = 0; l < maxLoop; l++){
		undef = false;
		for(uint i = 0; i < n; i++){
			if(toCreate[i]){
				state[i] = CreateObject(i);
				// Undefined is returned, so loop again
				if(state[i] == SupportState::Undefined)
					undef = true;
				else toCreate[i] = false;
			}
		}
		if(!undef)
			break;
	}
	if(undef && l == maxLoop){
		Message::Error("maximum loop count reached in mirroring.");
	}
}

void Adaptor::SyncProperty(bool download, int cat, int type){
	int n = (int)alive.size();
	for(int i = 0; i < n; i++){
		if(alive[i] && state[i] == SupportState::Supported){
			if(type == -1 || typedb->KindOf(scene->GetObjectType(i), type)){
				SyncObjectProperty(i, download, cat);
			}
		}
	}
}

Adaptor::Aux* Adaptor::HandleByID(int id, int typeId){
	if(!typedb->KindOf(scene->GetObjectType(id), typeId))
		throw AdaptorFailure();
	if(!alive[id])
		throw AdaptorFailure();
	if(!IsSupported(id))
		throw AdaptorFailure();
	return handle[id];
}

Adaptor::Aux* Adaptor::HandleByName(string path, int typeId){
	return HandleByID(scene->Find(Address(path), rootId), typeId);
}

}
