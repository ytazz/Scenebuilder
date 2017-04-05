#include <sbscenelocal.h>

namespace Scenebuilder{;

SceneLocal::SceneLocal(){
}

void SceneLocal::Create(){
	Clear();
}

void SceneLocal::Clear(){
	objArray.clear();
}

int	SceneLocal::GetRoot(){
	return 0;
}

int	SceneLocal::GetParent(int id){
	return objArray[id]->parId;
}

void SceneLocal::GetChildren(int id, vector<int>& children){
	children = objArray[id]->children;
}

string SceneLocal::GetName(int id){
	return objArray[id]->name;
}

void SceneLocal::SetName(int id, string name){
	objArray[id]->name = name;
}

void SceneLocal::AddChild(int parId, int childId){
	Object* obj = objArray[parId];
	if(find(obj->children.begin(), obj->children.end(), childId) == obj->children.end())
		obj->children.push_back(childId);
	objArray[childId]->parId = parId;
}

void SceneLocal::RemoveChild(int parId, int childId){
	Object* obj = objArray[parId];
	obj->children.erase(find(obj->children.begin(), obj->children.end(), childId));
	objArray[childId]->parId = -1;
}

int SceneLocal::CreateObject(int type, const string& name, TypeDB* typedb){
	int id;
	for(id = 0; id < (int)objArray.size(); id++){
		if(!objArray[id]->alive)
			break;
	}

	Object* obj;
	if(id == objArray.size()){
		objArray.push_back(new Object());
		obj = objArray.back();
	}
	else{
		obj = objArray[id];
		obj->alive = true;
	}

	obj->type = type;
	obj->name = name;

	// プロパティサイズの配列を確保し，そのアドレスに対してコンストラクタを呼ぶ
	TypeInfo* typeInfo = typedb->GetType(type);
	obj->prop.resize(typeInfo->szProp);
	typeInfo->constructor((Property*)&(obj->prop[0]));

	return id;
}

void SceneLocal::DeleteObject(int id){
	Object* obj = objArray[id];
	
	// delete reference from parent
	if(obj->parId != -1)
		RemoveFromArray(objArray[obj->parId]->children, id);

	// delete all links pointing to this object
	for(uint i = 0; i < objArray.size(); i++){
		Object* o = objArray[i];
		for(vector<Link>::iterator it = o->links.begin(); it != o->links.end(); ){
			if(it->id == id)
				 it = o->links.erase(it);
			else it++;
		}
	}

	// delete children recursively
	while(!obj->children.empty())
		DeleteObject(obj->children[0]);

	// unset liveness
	obj->prop .clear();
	obj->links.clear();
	obj->alive = false;
	cout << "deleted " << id << endl;
}

int SceneLocal::GetObjectType(int id){
	return objArray[id]->type;
}

void SceneLocal::AddLink(int srcId, int destId, const string& name){
	Object* obj = objArray[srcId];
	uint i;
	for(i = 0; i < obj->links.size(); i++){
		if(obj->links[i].name == name){
			obj->links[i].id = destId;
			return;
		}
	}
	if(i == obj->links.size())
		obj->links.push_back(Link(destId, name));
}

void SceneLocal::RemoveLink(int srcId, int destId){
	Object* obj = objArray[srcId];
	for(vector<Link>::iterator it = obj->links.begin(); it != obj->links.end(); ){
		if(it->id == destId)
			 it = obj->links.erase(it);
		else it++;
	}
}

int SceneLocal::GetLink(int id, const string& name){
	Object* obj = objArray[id];
	for(uint i = 0; i < obj->links.size(); i++){
		Link& lnk = obj->links[i];
		if(lnk.name == name)
			return lnk.id;
	}
	return -1;
}

void SceneLocal::GetLinks(int id, vector<int>& links, vector<string>& names){
	Object* obj = objArray[id];
	for(uint i = 0; i < obj->links.size(); i++){
		Link& lnk = obj->links[i];
		links.push_back(lnk.id);
		names.push_back(lnk.name);
	}
}

Property* SceneLocal::GetProperty(int id){
	return (Property*)&(objArray[id]->prop[0]);
}

void SceneLocal::GetLivenessRecursively(int id, vector<byte>& alive){
	alive[id] = 1;
	vector<int> children;
	GetChildren(id, children);
	for(uint i = 0; i < children.size(); i++)
		GetLivenessRecursively(children[i], alive);
}

void SceneLocal::GetLivenessArray(Address& addr, vector<byte>& alive){
	alive.resize(objArray.size());
	fill(alive.begin(), alive.end(), false);
	
	int id = Find(addr, GetRoot());
	GetLivenessRecursively(id, alive);
}

void SceneLocal::SetLocation(int id, const string& path){
	bool found = false;
	for(uint i = 0; i < locArray.size(); i++){
		Location& loc = locArray[i];
		if(loc.id == id){
			loc.path = path;
			found = true;
		}
	}
	if(!found)
		locArray.push_back(Location(id, path));
}

string SceneLocal::GetLocation(int id){
	Location* loc = 0;
	for(uint i = 0; i < locArray.size(); i++){
		Location& l = locArray[i];
		if(l.id == id){
			loc = &l;
			break;
		}
	}

	// 絶対パスならそのまま返す
	if(loc && loc->path.IsAbsolute())
		return loc->path;

	Path path;

	// 親のロケーション
	if(id != GetRoot())
		path += GetLocation(GetParent(id));

	// 親のロケーションに自分の相対パスを付加
	if(loc)
		path += loc->path;

	return path;
}

}
