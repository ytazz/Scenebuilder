#include <sbmodel.h>
#include <sbmodelstl.h>
#include <sbmodelpcd.h>
#include <sbmodel3ds.h>
#include <sbmodelobj.h>
#include <sbmodelpmx.h>
#include <sbmessage.h>
#include <sbscene.h>
#include <sbpath.h>

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

Material::Material(){
	// Springhead GRMaterialDescのデフォルト値と同じ
	ambient   = Vec4f(0.2, 0.2, 0.2, 1.0);
	diffuse   = Vec4f(0.8, 0.8, 0.8, 1.0);
	specular  = Vec4f(1.0, 1.0, 1.0, 1.0);
	emissive  = Vec4f(0.0, 0.0, 0.0, 1.0);
	shininess = 20.0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Bone::Bone(){
	parent   = 0;
	terminal = false;
}

void Bone::SetParent(Bone* par){
	parent = par;
	if(parent)
		parent->children.push_back(this);
}
	
void Bone::SetTransform(const Affinef& a){
	terminal = true;
	affRel = a;
	aff    = affRel * affIni;
	CalcTransformRecurs();
}

void Bone::CalcTransformRecurs(){
	if(!terminal && parent){
		aff = parent->aff * affLocal;
		affRel = aff * affIni.inv();
	}
	for(uint i = 0; i < children.size(); i++)
		children[i]->CalcTransformRecurs();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void BonePalette::Clear(){
	fill(begin(), end(), -1);
}

int BonePalette::Find(int bone){
	// ボーン未参照は0番（単位変換）にわりあて
	if(bone == -1)
		return 0;
	// 登録済の中から探す
	for(uint i = 1; i < size(); i++){
		if(at(i) == bone)
			return i;
	}
	// なければ空きを探して登録
	for(uint i = 1; i < size(); i++){
		if(at(i) == -1){
			at(i) = bone;
			return i;
		}
	}
	return -1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Model::Model(){
	id = -1;
	bonePaletteSize = 8;
}

void Model::CalcBound(){
	float inf = numeric_limits<float>::max();
	for(uint k = 0; k < 3; k++){
		bbmin[k] =  inf;
		bbmax[k] = -inf;
	}
	for(uint i = 0; i < meshSolid.size(); i++){
		meshSolid[i].CalcBBox();
		for(uint k = 0; k < 3; k++){
			bbmin[k] = std::min(bbmin[k], meshSolid[i].bbmin[k]);
			bbmax[k] = std::max(bbmax[k], meshSolid[i].bbmax[k]);
		}
	}
	if(bbmin[0] <= bbmax[0] && bbmin[1] <= bbmax[1] && bbmin[2] <= bbmax[2]){
		bbsize   = bbmax - bbmin;
		bbcenter = 0.5f * (bbmax + bbmin);
		bbradius = 0.5f * bbsize.norm();
	}
	else{
		bbsize  .clear();
		bbcenter.clear();
		bbradius = 0.0f;
	}
}

void Model::InitBone(){
	for(uint i = 0; i < bones.size(); i++){
		Bone* b  = &bones[i];
		if(b->parent)
			b->affLocal = b->parent->affIni.inv() * b->affIni;
	}
}

Bone* Model::GetBone(int i){
	if(0 <= i && i < (int)bones.size())
		return &bones[i];
	return 0;
}

int Model::GetBoneIndex(const string& bonename){
	for(uint i = 0; i < bones.size(); i++){
		if(bones[i].name == bonename)
			return i;
	}
	return -1;
}

void Model::CalcBoneTransform(){

}

///////////////////////////////////////////////////////////////////////////////////////////////////

ModelContainer::ModelContainer(){
	scene = 0;
}

void ModelContainer::Set(SceneBase* s){
	scene = s;
}

Model* ModelContainer::GetModel(int id){
	for(uint i = 0; i < models.size(); i++){
		if(models[i]->id == id)
			return models[i];
	}
	return 0;
}

bool ModelContainer::LoadModel(int id, const string& filename, MeshProp* meshProp){
	Path   path = Path(scene->GetLocation(id)) + Path(filename);
	string ext  = path.Ext();

	UTRef<ModelLoader> loader;
	
	if(ext == "stl")
		loader = new LoaderSTL(meshProp->stl_color, meshProp->stl_binary);
	else if(ext == "pcd")
		loader = new LoaderPCD();
	else if(ext == "3ds")
		loader = new Loader3DS();
	else if(ext == "obj")
		loader = new LoaderOBJ();
	else if(ext == "pmx")
		loader = new LoaderPMX();
	else{
		Message::Error("unsupported file extension: %s", ext.c_str());
		return false;
	}

	/// ロード
	try{
		loader->Load(path);
	}
	catch(Exception&){
		Message::Error("failed to load %s", path.c_str());
		return false;
	}

	/// メッシュへコンバート
	UTRef<Model> model(new Model());
	try{
		float s = (float)meshProp->scale;
		Affinef aff = Affinef::Scale(s,s,s);
		loader->Convert(model, aff);
	}
	catch(Exception&){
		return false;
	}

	/// BBox計算
	model->CalcBound();

	/// テクスチャ座標生成
	int auto_texcoord = meshProp->AutoTexCoord();
	if(auto_texcoord != ShapeProp::None){
		for(uint i = 0; i < model->meshSolid.size(); i++){
			model->meshSolid[i].texture_scale  = meshProp->texture_scale;
			model->meshSolid[i].texture_offset = meshProp->texture_offset;
			model->meshSolid[i].GenerateTexCoords(auto_texcoord);
		}
	}

	model->id       = id;
	model->filename = path;
	models.push_back(model);

	return true;
}

void ModelContainer::UnloadModel(int id){
	for(uint i = 0; i < models.size(); ){
		if(models[i]->id == id)
			 models.erase(models.begin()+i);
		else i++;
	}
}

void ModelContainer::UnloadModelRecurs(int id){
	vector<int> children;
	scene->GetChildren(id, children);
	for(uint i = 0; i < children.size(); i++)
		UnloadModelRecurs(children[i]);
	UnloadModel(id);
}

}
