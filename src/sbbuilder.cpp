#include <sbbuilder.h>
#include <sbmessage.h>
#include <sbconverter.h>

namespace Scenebuilder{;

//-------------------------------------------------------------------------------------------------

int ImportProp::id;
int ModuleProp::id;
int LinkProp  ::id;
int ForProp   ::id;
int VarProp   ::id;

//-------------------------------------------------------------------------------------------------

Builder::Attrs::Attrs(){
}

//-------------------------------------------------------------------------------------------------

Builder::Builder(){
	scene = 0;
	//attrStack.push_back(Attrs());
	ctxStack.push_back(Context());
}

Builder::~Builder(){

}

void Builder::Register(TypeDB* db){
	ImportProp::Register(db);
	ModuleProp::Register(db);
	LinkProp  ::Register(db);
	ForProp   ::Register(db);
	VarProp   ::Register(db);
}

//-------------------------------------------------------------------------------------------------

string Builder::GetAttrValue(string name){
	string val;
	
	// find in temporary attributes
	{
		Attrs::iterator it = tempAttr.find(name);
		if(it != tempAttr.end())
			val = it->second;
	}
		
	// if not found, find in global attributes
	if(val.empty()){
		for(ContextStack::reverse_iterator rit = ctxStack.rbegin(); rit != ctxStack.rend(); rit++){
			Attrs::iterator it = rit->attrs.find(name);
			if(it != rit->attrs.end()){
				val = it->second;
				break;
			}
		}
	}
	
	// 登録されている変数を変数値に置換
	for(ContextStack::reverse_iterator rit = ctxStack.rbegin(); rit != ctxStack.rend(); rit++){
		for(Vars::iterator it = rit->vars.begin(); it != rit->vars.end(); it++){
			string varname = string("$") + it->first;
			string varval  = it->second;
			size_t pos = 0;
			while(true){
				pos = val.find(varname, pos);
				if(pos == string::npos)
					break;
				val.replace(pos, varname.size(), varval);
				pos++;
			}
		}
	}

	return val;
}

void Builder::FillProperty(TypeInfo* type, Property* prop){
	// 親タイプのプロパティをセット
	if(type->baseId != -1){
		TypeInfo* base = typedb->GetType(type->baseId);
		FillProperty(base, prop);
	}

	for(uint i = 0; i < type->attrs.size(); i++){
		auto& attr = type->attrs[i];

		if(attr->primId == Primitive::Path)
			continue;
		
		// まずデフォルト値をセット
		attr->Set(attr->def, prop);

		// コンテキストから値を探し，あればセット
		string val = GetAttrValue(attr->name);
		if(!val.empty()){
			if(!attr->FromString(val, prop))
				Message::Error("invalid attribute value: %s", val.c_str());
		}
	}
}

void Builder::AddLinks(int srcId, int refId, TypeInfo* type){
	// 親タイプのリンクを先に解決
	if(type->baseId != -1){
		TypeInfo* base = typedb->GetType(type->baseId);
		AddLinks(srcId, refId, base);
	}
	for(uint i = 0; i < type->attrs.size(); i++){
		auto& attr = type->attrs[i];

		if(attr->primId == Primitive::Path){
			string path = GetAttrValue(attr->name);	
			if(!path.empty())
				Link(srcId, refId, path, attr->name);
		}
	}
}

void Builder::Link(int srcId, int refId, const string& path, const string& name){
	try{
		scene->AddLink(srcId, scene->Find(Address(path), refId), name);
	}
	catch(Exception&){
		string srcname = scene->GetName(srcId);
		Message::Error("%s: path of attribute %s is invalid: %s", srcname.c_str(), name.c_str(), path.c_str());
	}
}

void Builder::Create(int typeId, string n){
	TypeInfo* type = typedb->GetType(typeId);
	if(!type){
		Message::Error("unknown type id %d", typeId);
		return;
	}
	if(!typedb->KindOf(typeId, SceneObjectProp::id)){
		Message::Error("not a scene object: %s", type->name.c_str());
	}

	// generate name for new object
	string name = scene->AssignName(n, type, curObjId);
	
	// create new object
	int childId = scene->CreateObject(typeId, name, typedb);
	scene->AddChild(curObjId, childId);

	// Parseにより作られる最上位オブジェクトのIDを記憶
	if(parsedRootId == -1){	
		parsedRootId = childId;
		// ロケーションを記憶する
		Path path = baseFilename;
		scene->SetLocation(parsedRootId, path.Dir());
	}

	// add links
	//  base of relative path will be the parent of created object
	AddLinks(childId, curObjId, type);

	// move focus to created object
	curObjId = childId;

	// 基本単位を設定
	string L, R, M, T;
	L = GetAttrValue("unit_length"  );
	R = GetAttrValue("unit_rotation");
	M = GetAttrValue("unit_mass"    );
	T = GetAttrValue("unit_time"    );
	if(!L.empty()) Calc::SetUnitLength  (L);
	if(!R.empty()) Calc::SetUnitRotation(R);
	if(!M.empty()) Calc::SetUnitMass    (M);
	if(!T.empty()) Calc::SetUnitTime    (T);

	// set property values
	FillProperty(type, scene->GetProperty(curObjId));

    // create mesh of primitive shape for convenience
    if(typeId == BoxProp::id){
        BoxProp* boxProp = (BoxProp*)scene->GetProperty(curObjId);
        models->CreateModel(curObjId, boxProp);
    }
    if(typeId == SphereProp::id){
        SphereProp* sphereProp = (SphereProp*)scene->GetProperty(curObjId);
        models->CreateModel(curObjId, sphereProp);
    }
	// 3Dモデル，画像のロード処理
	if(typeId == MeshProp::id){
		MeshProp* meshProp = (MeshProp*)scene->GetProperty(curObjId);
		// Meshのscaleは未指定（0.0）の場合単位にもとづき設定する
		if(meshProp->scale == 0.0)
			meshProp->scale = Calc::ScaleFromDimension(Dimension::L);

		// ロードしてモデルコンテナに格納
		// 複合モデルの場合はBodyやJointが同時に生成される
		models->LoadModel(curObjId, string(meshProp->filename), meshProp);
	}

	// clear temporary attributes
	tempAttr.clear();

	// trnとrotは階層ごとのローカル座標なので，階層を下るたびに原点に設定する
	ctxStack.push_back(Context());
}

void Builder::End(){
	// move focus to parent object
	curObjId = scene->GetParent(curObjId);
	// pop global attribute 
	ctxStack.pop_back();
}

//-------------------------------------------------------------------------------------------------
	
void Builder::ProcessNode(int nodeId){
	XMLNode* node = xmlStack.top()->GetNode(nodeId);

	// "attr"タグの場合，XMLノードの属性をglobal attributeに加える
	if(node->name == "attr"){
		for(XMLNode::Attrs::iterator it = node->attrs.begin(); it != node->attrs.end(); it++)
			ctxStack.back().attrs[it->first] = it->second;
		return;
	}

	// get type info
	TypeInfo* type = typedb->GetType(node->name);
	if(!type){
		Message::Error("unknown type %s", node->name.c_str());
		return;
	}

	// XMLノードの属性をtemporary attuributeに設定
	for(XMLNode::Attrs::iterator it = node->attrs.begin(); it != node->attrs.end(); it++)
		tempAttr[it->first] = it->second;

	// get name attribute
	string name = GetAttrValue("name");

	// case: scene object except for import
	if(typedb->KindOf(type->id, SceneObjectProp::id) && type->id != ImportProp::id){
		// create object specified by type id
		Create(type->id, name);

		// process child nodes
		for(uint i = 0; i < node->children.size(); i++)
			ProcessNode(node->children[i]);
		
		End();
	}
	// case: import type
	else if(type->id == ImportProp::id){
		ImportProp prop;
		FillProperty(type, &prop);

		string path     = (const char*)prop.path;
		string filename = (const char*)prop.filename;
		
		// if import path is specified:
		if(!path.empty()) try{
			// get xml node specified by path
			// - path is relative to parent node of import node
			int parId = xmlStack.top()->GetParent(nodeId);
			int impId = xmlStack.top()->Find(Address(path), parId);
			XMLNode* impNode = xmlStack.top()->GetNode(impId);
	
			// create namespace with the name of import
			Create(NamespaceProp::id, name);
			// process child nodes under this namespace
			for(uint i = 0; i < impNode->children.size(); i++){
				ProcessNode(impNode->children[i]);
			}
			End();
		}
		catch(InvalidOperation&){
			Message::Error("invalid import path: %s", path.c_str());
		}

		// if import filename is specified:
		if(!filename.empty()) try{
			// load xml of specified filename
			xmlStack.push(new XML());
			
			Message::Out("loading xml %s", filename.c_str());
			xmlStack.top()->Load(Path(scene->GetLocation(curObjId)) + Path(filename));
			xmlStack.top()->Print(DSTR);
			
			// create namespace with the name of import
			Create(NamespaceProp::id, name);
			// set location of namespace to directory of loaded xml
			Path subdir = filename;
			scene->SetLocation(curObjId, subdir.Dir());
			// process child nodes below root node under this namespace
			vector<int>& children = xmlStack.top()->GetRootNode()->children;
			for(uint i = 0; i < children.size(); i++)
				ProcessNode(children[i]);
			End();

			xmlStack.pop();
		}
		catch(Exception&){
			Message::Error("invalid import filename: %s", filename.c_str());
			xmlStack.pop();
		}
	}
	// case: module type
	// - do nothing
	// - module will be parsed when imported
	else if(type->id == ModuleProp::id){
	
	}
	// case: link type
	// - create link from focused object to target
	else if(type->id == LinkProp::id){
		LinkProp prop;
		FillProperty(type, &prop);
		string n    = (const char*)prop.name;
		string name = scene->AssignName(n, type, curObjId);
	
		Link(curObjId, curObjId, string(prop.path), name);
	}
	//
	else if(type->id == ForProp::id){
		ForProp prop;
		FillProperty(type, &prop);

		// 変数登録
		RegisterVar(string(prop.var));

		// ループしながらノードを処理
		for(int i = prop.begin; i < prop.end; i++){
			SetVarValue(string(prop.var), i);
			for(uint j = 0; j < node->children.size(); j++)
				ProcessNode(node->children[j]);
		}

		// 変数削除
		UnregisterVar(string(prop.var));
	}
	else if(type->id == VarProp::id){
		VarProp prop;
		FillProperty(type, &prop);

		RegisterVar(string(prop.name));
		SetVarValue(string(prop.name), prop.value);
	}

	// clear temporary attributes
	tempAttr.clear();
}

void Builder::RegisterVar(string name){
	if(name.empty())
		return;
	ctxStack.back().vars[name] = string();
}

void Builder::UnregisterVar(string name){
	if(name.empty())
		return;
	Vars::iterator it = ctxStack.back().vars.find(name);
	ctxStack.back().vars.erase(it);
}

void Builder::SetVarValue(string name, int val){
	if(name.empty())
		return;
	stringstream ss;
	Vars::iterator it = ctxStack.back().vars.find(name);
	if(it != ctxStack.back().vars.end()){
		ss << val;
		it->second = ss.str();
	}
}

void Builder::Set(SceneBase* s, TypeDB* db, ModelContainer* m){
	scene  = s;
	typedb = db;
	models = m;
}


int Builder::Parse(string filename, int base){
	baseFilename = filename;
	curObjId = (base == -1 ? scene->GetRoot() : base);
	parsedRootId = -1;

	Message::Out("loading xml %s", baseFilename.c_str());

	// create xml tree
	xmlStack.push(new XML());
	xmlStack.top()->Load(baseFilename);
	xmlStack.top()->Print(DSTR);

	Message::Out("parsing xml");

	// build scene from xml tree
	ProcessNode(xmlStack.top()->GetRoot());
	
	if(parsedRootId != -1){
		scene->PrintRecursive(parsedRootId, DSTR, 0);
	}

	return parsedRootId;
}

}
