#include <sbsync.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
using namespace boost;


namespace Scenebuilder{;

Synchronizer::Synchronizer(){
	scene[0] = 0;
	scene[1] = 0;
}

void Synchronizer::Set(SceneBase* s0, SceneBase* s1){
	scene[0] = s0;
	scene[1] = s1;
	alive[0].clear();
	alive[1].clear();
	idMap[0].clear();
	idMap[1].clear();
}

void Synchronizer::SyncTree(Address& addr0, Address& addr1){
	scene[0]->GetLivenessArray(addr0, alive[0]);
	scene[1]->GetLivenessArray(addr1, alive[1]);
	vector<int>	toCopy[2];
	vector<int>	toDelete[2];
	
	int n;
	int side;

	// identify objects to copy/delete
	for(side = 0; side < 2; side++){
		n = (int)alive[side].size();
		for(int i = 0; i < n; i++){
			// alive but no copy => create copy
			if(alive[side][i] && idMap[side][i] == -1){
				toCopy[side].push_back(i);
			}
			// not alive but copy exists => delete copy
			if(!alive[side][i] && idMap[side][i] != -1){
				toDelete[!side].push_back(idMap[side][i]);
				// unlink
				idMap[side][i] = -1;
			}
		}
	}

	// delete
	for(side = 0; side < 2; side++){
		foreach(int id, toDelete[side]){
			scene[side]->DeleteObject(id);
		}
	}
	
	// copy
	for(side = 0; side < 2; side++){
		foreach(int id, toCopy[side]){
			int type = scene[side]->GetObjectType(id);
			string name = scene[side]->GetName(id);

			int copyId = scene[!side]->CreateObject(type, name, false);
			// link
			idMap[side][id] = copyId;
			idMap[!side][copyId] = id;
		}
	}

	// parent/children relation and links
	for(side = 0; side < 2; side++){
		foreach(int id, toCopy[side]){
			int parId = scene[side]->GetParent(id);
			int copyId = idMap[side][id];
			int copyParId = idMap[side][parId];
			scene[!side]->AddChild(copyParId, copyId);
		}
	}

	// link
	for(side = 0; side < 2; side++){
		foreach(int id, toCopy[side]){
			int copyId = idMap[side][id];
			vector<int> links;
			vector<string>	names;
			scene[side]->GetLinks(id, links, names);
			for(uint i = 0; i < links.size(); i++){
				scene[!side]->AddLink(copyId, idMap[side][links[i]], names[i]);
			}
		}
	}

}

void Synchronizer::SyncProperty(){
	scene[0]->GetPropertyArray(&liveId[0][0], liveId[0].size(), buf);
	scene[1]->SetPropertyArray(&liveId[1][0], liveId[1].size(), buf);

	scene[1]->GetPropertyArray(&liveId[1][0], liveId[1].size(), buf);
	scene[0]->GetPropertyArray(&liveId[0][0], liveId[0].size(), buf);
}

/*
void Synchronizer::Sync(bool l_to_r){
	// 同じメモリ空間での同期なのでポインタを渡してコピーさせる
	for(uint i = 0; i < idMap.size(); i++){
		if(l_to_r)
			 acc1->SetStateAt(idMap[i].second, acc0->GetStateAt(idMap[i].first));
		else acc0->SetStateAt(idMap[i].first, acc1->GetStateAt(idMap[i].second));
	}
}
*/

/*
void Synchronizer::CopyRecursively(){
	// IDの対応を記憶
	if(l_to_r)
		 idMap.push_back(make_pair(acc0.GetID(), acc1.GetID()));
	else idMap.push_back(make_pair(acc1.GetID(), acc0.GetID()));

	// コピー元アクセッサを子オブジェクトにフォーカスする
	if(!acc0.EnterFront())
		return;

	while(true){
		// タイプと名前を取得
		int type = acc0->GetObjectType();
		string name = acc0->GetName();
		// 同じ属性のオブジェクトをコピー先に作成し，フォーカスする
		acc1.CreateObject(type, name);
		acc1.EnterBack();

		// その下のサブツリーを再帰コピー
		CopyRecursively();
		// コピー先のフォーカスを親にもどす
		acc1.Leave();

		// 次の子オブジェクトにフォーカスを移動（なければ終了）
		if(!acc0.Next())
			break;
	}
	// コピー元も親にフォーカスをもどす
	acc0.Leave();

	// copy descriptors and get id
	int id0 = acc0->GetID();
	Descriptor* desc = acc0->GetDescriptor();
	int id1 = acc1->CreateObject();
	acc1->SetDescriptor(desc);

	idMap.push_back(make_pair(id0, id1));
	
	// enter this object and copy children
	if(acc0->EnterFront()){
		acc1->EnterFront();

		while(true){
			CopyRecursively();
			if(!acc0->Next())
				break;
		}

		acc0->Leave();
	}
	acc1->Leave();
	return true;
}

void Synchronizer::Copy(Address& addr0, Address& addr1){
	if(!scene0 || !scene1)
		throw InvalidOperationException();

	if(l_to_r){
		acc0.Set(scene0);
		acc0.GoTo(addr0);
		acc1.Set(scene1);
		acc1.GoTo(addr1);
	}
	else{
		acc0.Set(scene1);
		acc0.GoTo(addr1);
		acc1.Set(scene0);
		acc1.GoTo(addr0);
	}

	// copy recursively
	CopyRecursively();
	CopyLinks();

}
*/

}
