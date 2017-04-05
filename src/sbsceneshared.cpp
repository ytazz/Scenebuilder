#include <sbsceneshared.h>
#include <sbpath.h>

using namespace std;

namespace Scenebuilder{;

//-------------------------------------------------------------------------------------------------

void SceneShared::Header::Init(TypeDB* db, const SceneShared::Config& conf){
	code[0] = 'S';
	code[1] = 'B';

	// get type ids derived from SceneObject
	vector<int>	objTypes;
	db->EnumDerivedTypes(SceneObjectProp::id, objTypes);
	int maxId = 0;
	for(uint i = 0; i < objTypes.size(); i++)
		maxId = std::max(maxId, objTypes[i]);

	// number of object types
	numObjectTypes = objTypes.size();
	maxObjectTypeId = maxId;

	// calculate size of each element
	size_t szObject   = sizeof(Object);
	size_t szChild    = sizeof(ListNode<int>);
	size_t szLink     = sizeof(ListNode<Link>);
	size_t szLoc      = sizeof(Location);

	// calculate total size
	szTotal =  sizeof(Header);
	szTotal += sizeof(ArrayHeader) + szObject * conf.maxObjects;
	szTotal += sizeof(ArrayHeader) + szChild  * conf.maxChildren;
	szTotal += sizeof(ArrayHeader) + szLink   * conf.maxLinks;
	szTotal += sizeof(ArrayHeader) + szLoc    * conf.maxLocs; 

	for(uint i = 0; i < objTypes.size(); i++){
		int id = objTypes[i];
		szTotal += sizeof(ArrayHeader) + db->GetType(id)->szProp * conf.MaxProps(id);
	}
}

//-------------------------------------------------------------------------------------------------

void SceneShared::Object::Init(){
	alive		= true;
	original	= true;
	parId		= -1;
	numChildren = 0;
	numLinks	= 0;
	childFirst	= -1;
	childLast	= -1;
	linkFirst	= -1;
	linkLast	= -1;
	type		= NamespaceProp::id;
	propId		= -1;
	locId       = -1;
}

//-------------------------------------------------------------------------------------------------

void SceneShared::Create(const string& name, TypeDB* typedb, const SceneShared::Config& conf){
	// 仮のヘッダを作成して総バイト数を計算
	Header tmpHeader;
	tmpHeader.Init(typedb, conf);
	
	// create shared memory
	sharedMemory.Create(name.c_str(), tmpHeader.szTotal);
	sharedName = name;
	byte* start = (byte*)sharedMemory.Get(0, tmpHeader.szTotal);

	// clear memory
	memset(start, 0, tmpHeader.szTotal);

	// init information based on config
	header = (Header*)start;
	header->Init(typedb, conf);

	byte* ptr = start;
	ptr += sizeof(Header);

	objArray .Init(ptr, sizeof(Object),         conf.maxObjects);
	childList.Init(ptr, sizeof(ListNode<int>),  conf.maxChildren);
	linkList .Init(ptr, sizeof(ListNode<Link>), conf.maxLinks);
	locArray .Init(ptr, sizeof(Location),       conf.maxLocs);
	
	propArray.resize(header->maxObjectTypeId + 1);
	for(uint id = 0; id < propArray.size(); id++){
		if(!typedb->KindOf(id, SceneObjectProp::id))
			continue;
		TypeInfo* type = typedb->GetType(id);
		propArray[id].Init(ptr, type->szProp, conf.MaxProps(id));
	}
}

void SceneShared::Open(const string& name, TypeDB* typedb){
	sharedMemory.Open(name.c_str());
		
	// ヘッダのみメモリマップして総バイト数を取得
	header = (Header*)sharedMemory.Get(0, sizeof(Header));
	if(!(header->code[0] == 'S' && header->code[1] == 'B')){
		sharedMemory.Close();
		throw SceneFileError();
	}
	size_t sz = header->szTotal;
	
	// 総バイト数分をあらためてメモリマップ
	byte* start = (byte*)sharedMemory.Get(0, sz);
	header = (Header*)start;
	
	byte* ptr = start;
	ptr += sizeof(Header);

	objArray    .Init(ptr);
	childList   .Init(ptr);
	linkList    .Init(ptr);
	locArray    .Init(ptr);
	propArray.resize(header->maxObjectTypeId + 1);
	for(uint id = 0; id < propArray.size(); id++){
		if(!typedb->KindOf(id, SceneObjectProp::id))
			continue;
		propArray[id].Init(ptr);
	}

	sharedName = name;
}

bool SceneShared::IsOpen(){
	return sharedMemory.IsOpen();
}

void SceneShared::Close(){
	sharedMemory.Close();
	sharedName = "";
}

void SceneShared::PrintInfo(ostream& os, TypeDB* typedb){
	if(!IsOpen())
		return;
	os << "shared memory name: " << sharedName << endl;
	os << "total memory size: "  << header->szTotal << 'B' << endl;
	os << "num of objects: "     << objArray .header->num << '/' << objArray .header->nmax << endl;
	os << "num of children: "    << childList.header->num << '/' << childList.header->nmax << endl;
	os << "num of links: "       << linkList .header->num << '/' << linkList .header->nmax << endl;
	os << "num of locations: "   << locArray .header->num << '/' << locArray .header->nmax << endl;

	os << "num of properties:" << endl;
	for(uint id = 0; id < propArray.size(); id++){
		if(!typedb->KindOf(id, SceneObjectProp::id))
			continue;
		TypeInfo* type = typedb->GetType(id);
		os << ' ' << type->name << ": " << propArray[id].header->num << '/' << propArray[id].header->nmax << endl;
	}
}

void SceneShared::Clear(){
	// reset "alive" flags of all items
	objArray    .DeleteAll();
	childList   .DeleteAll();
	linkList    .DeleteAll();
	locArray    .DeleteAll();
	for(uint i = 0; i < propArray.size(); i++){
		propArray[i].DeleteAll();
	}
}

int	SceneShared::GetRoot(){
	return 0;
}

int	SceneShared::GetParent(int id){
	return objArray.Get(id)->parId;
}

void SceneShared::GetChildren(int id, vector<int>& children){
	Object* obj = objArray.Get(id);
	childList.GetValues(obj->childFirst, children);
}

string SceneShared::GetName(int id){
	return (const char*)objArray.Get(id)->name;
}

void SceneShared::SetName(int id, string name){
	objArray.Get(id)->name = name;
}

int SceneShared::CreateObject(int type, const string& name, TypeDB* typedb){
	// create child object node
	uint childId  = objArray.Create();
	Object* child = objArray.Get(childId);
	child->Init();
	child->type = type;
	child->name = name;
	// create property
	child->propId = propArray[type].Create();
	// call constructor
	typedb->GetType(type)->constructor(propArray[type].Get(child->propId));

	return childId;
}

void SceneShared::DeleteObject(int id){
	Object* obj = objArray.Get(id);

	// delete link from parent
	RemoveChild(obj->parId, id);

	// delete children recursively
	vector<int> children;
	GetChildren(id, children);
	for(uint i = 0; i < children.size(); i++)
		DeleteObject(children[i]);

	// delete links
	linkList.DeleteList(obj->linkFirst);
	
	// delete property
	propArray[obj->type].Delete(obj->propId);
	
	// delete object itself
	objArray.Delete(id);
}

void SceneShared::AddChild(int parId, int childId){
	// get parent object node
	Object* par = objArray.Get(parId);
	Object* child = objArray.Get(childId);

	// create new list node
	uint idx = childList.Create();
	ListNode<int>* node = childList.Get(idx);
	node->val = childId;

	childList.AddToList(idx, par->childFirst, par->childLast);
	par->numChildren++;
	
	child->parId = parId;
}

void SceneShared::RemoveChild(int parId, int childId){
	// get parent object node
	Object* par = objArray.Get(parId);
	Object* child = objArray.Get(childId);

	int idx = childList.FindInList(FindById(childId), par->childFirst);
	if(idx != -1){
		childList.RemoveFromList(idx, par->childFirst, par->childLast);
		childList.Delete(idx);
		par->numChildren--;
		child->parId = -1;
	}
}

void SceneShared::AddLink(int srcId, int destId, const string& name){
	// get source object node
	Object* src = objArray.Get(srcId);

	// create new list node
	uint idx = linkList.Create();
	ListNode<Link>* node = linkList.Get(idx);
	node->val.id = destId;
	node->val.name = name;
	
	// add it to source object's link list
	linkList.AddToList(idx, src->linkFirst, src->linkLast);
	src->numLinks++;
}

void SceneShared::RemoveLink(int srcId, int destId){
	Object* src = objArray.Get(srcId);

	int idx = linkList.FindInList(FindLinkById(destId), src->linkFirst);
	if(idx != -1){
		linkList.RemoveFromList(idx, src->linkFirst, src->linkLast);
		linkList.Delete(idx);
		src->numLinks--;
	}
}

int SceneShared::GetLink(int id, const string& name){
	Object* src = objArray.Get(id);
	int idx = linkList.FindInList(FindLinkByName(name.c_str()), src->linkFirst);
	if(idx == -1)
		return -1;
	ListNode<Link>* node = linkList.Get(idx);
	return node->val.id;
}

void SceneShared::GetLinks(int id, vector<int>& links, vector<string>& names){
	Object* obj = objArray.Get(id);
	links.clear();
	names.clear();
	int idx = obj->linkFirst;
	while(idx != -1){
		ListNode<Link>* node = linkList.Get(idx);
		links.push_back(node->val.id);
		names.push_back((const char*)node->val.name);
		idx = node->next;
	}
}

int	SceneShared::GetObjectType(int id){
	return objArray.Get(id)->type;
}
	
Property* SceneShared::GetProperty(int id){
	Object* obj = objArray.Get(id);
	return propArray[obj->type].Get(obj->propId);
}

/*void SceneShared::GetPropertyArray(const int* idArray, size_t num, Buffer& buf){
	buf.Reset();
	
	int id;
	int typeId;
	TypeInfo* type;

	for(uint i = 0; i < num; i++){
		id = idArray[i];
		typeId = GetObjectType(id);
		type = typedb->GetType(typeId);

		buf.Write(type->szProp);
		buf.Write((byte*)GetProperty(id), type->szProp);
	}
}

void SceneShared::SetPropertyArray(const int* idArray, size_t num, Buffer& buf){
	buf.Reset();
	for(uint i = 0; i < num; i++){
		int id = idArray[i];
		size_t sz;
		buf.Read(sz);
		buf.Read((byte*)GetProperty(id), sz);
	}
}*/

void SceneShared::GetLivenessRecursively(int id, vector<byte>& alive){
	alive[id] = 1;
	vector<int> children;
	GetChildren(id, children);
	for(uint i = 0; i < children.size(); i++)
		GetLivenessRecursively(children[i], alive);
}

void SceneShared::GetLivenessArray(Address& addr, vector<byte>& alive){
	alive.resize(objArray.header->nmax);
	fill(alive.begin(), alive.end(), false);

	int id = Find(addr, GetRoot());
	GetLivenessRecursively(id, alive);
}
	
void SceneShared::SetLocation(int id, const string& path){
	Object* obj = objArray.Get(id);

	// find existing location entry
	bool found = false;
	for(uint i = 0; i < locArray.header->num; i++){
		Location* loc = locArray.Get(i);
		if(loc->id == id){
			// modify path
			loc->path = path;
			found = true;
		}
	}
	// if not found, register one
	if(!found){
		int locId = locArray.Create();
		Location* loc = locArray.Get(locId);
		loc->id    = id;
		loc->path  = path;
		obj->locId = locId;
	}
}

string SceneShared::GetLocation(int id){
	Path locPath;
	for(uint i = 0; i < locArray.header->num; i++){
		Location* loc = locArray.Get(i);
		if(loc->id == id){
			locPath = loc->path;
			break;
		}
	}

	// 絶対パスならそのまま返す
	if(locPath.IsAbsolute())
		return locPath;

	Path path;

	// 親のロケーション
	if(id != GetRoot())
		path += GetLocation(GetParent(id));

	// 親のロケーションに自分の相対パスを付加
	path += locPath;

	return path;
}

}
