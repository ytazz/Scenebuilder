#include "sbsprgraphics.h"
#include "sbmodelspr.h"
#include <sbpath.h>
#include <sbmodelstl.h>
#include <sbmodel3ds.h>
#include <sbmodelobj.h>
#include <sbmessage.h>
#include <sbmesh.h>

#include <GL/glew.h>

#define AUTO(T, X, EXP) T X = static_cast<T>(EXP)

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorSprGR::BodyAux::BodyAux(){
	bbradius = 0.0f;
	initial  = true;
}
AdaptorSprGR::BodyAux::~BodyAux(){
	for(uint i = 0; i < cons.size(); i++)
		cons[i]->body = 0;
}

AdaptorSprGR::MaterialAux::MaterialAux(){
	alpha     = 1.0f;
	color     = Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
}
AdaptorSprGR::MaterialAux::~MaterialAux(){
	for(uint i = 0; i < attaches.size(); i++)
		attaches[i]->mat = 0;
}

AdaptorSprGR::ConnectorAux::ConnectorAux(BodyAux* b):body(b){
	body->cons.push_back(this);
	initial = true;
}
AdaptorSprGR::ConnectorAux::~ConnectorAux(){
	if(body)
		RemoveFromArray(body->cons, this);

	for(uint i = 0; i < attaches.size(); i++)
		attaches[i]->con = 0;
}

AdaptorSprGR::ShapeAux::ShapeAux(){
	bbradius       = 0.0f;
	auto_texcoord  = ShapeProp::None;
	texture_scale  = Vec2f(1.0f, 1.0f);
	texture_offset = Vec2f(0.0f, 0.0f);
}
AdaptorSprGR::ShapeAux::~ShapeAux(){
	for(uint i = 0; i < attaches.size(); i++)
		attaches[i]->shape = 0;
}

AdaptorSprGR::BoxAux::BoxAux(){
	size = Vec3f(1.0f, 1.0f, 1.0f);
}

AdaptorSprGR::SphereAux::SphereAux(){
	radius = 1.0f;
	slice  = 16;
	stack  = 16;
}

AdaptorSprGR::CylinderAux::CylinderAux(){
	radius = 1.0f;
	height = 1.0f;
	slice  = 16;
}

AdaptorSprGR::CapsuleAux::CapsuleAux(){
	radius = 1.0f;
	height = 1.0f;
	slice  = 16;
	stack  = 16;
}

AdaptorSprGR::MeshAux::MeshAux(){

}

int AdaptorSprGR::MeshAux::GetBoneIndex(const string& bonename){
	if(!model)
		return -1;
	return model->GetBoneIndex(bonename);
}

AdaptorSprGR::LightAux::LightAux(){

}
AdaptorSprGR::LightAux::~LightAux(){
	for(uint i = 0; i < attaches.size(); i++)
		attaches[i]->light = 0;
}

AdaptorSprGR::CameraAux::CameraAux(){

}
AdaptorSprGR::CameraAux::~CameraAux(){
	for(uint i = 0; i < attaches.size(); i++)
		attaches[i]->camera = 0;
}

AdaptorSprGR::AttachAux::AttachAux(ShapeAux* sh, LightAux* li, CameraAux* cam, ConnectorAux* c, MaterialAux* m):
	shape(sh), light(li), camera(cam), con(c), mat(m){
	if(shape ) shape ->attaches.push_back(this);
	if(light ) light ->attaches.push_back(this);
	if(camera) camera->attaches.push_back(this);
	if(mat   ) mat   ->attaches.push_back(this);

	con->attaches.push_back(this);
	
	boneIndex = -1;
}
AdaptorSprGR::AttachAux::~AttachAux(){
	if(shape ) RemoveFromArray(shape ->attaches, this);
	if(light ) RemoveFromArray(light ->attaches, this);
	if(camera) RemoveFromArray(camera->attaches, this);
	if(con   ) RemoveFromArray(con   ->attaches, this);
	if(mat   ) RemoveFromArray(mat   ->attaches, this);
}

AdaptorSprGR::DrawOption::DrawOption(){
	useVao     = false;
	drawSolid  = true;
	drawWire   = false;
	drawPoints = false;
	drawAxis   = false;
	drawCoM    = false;
	drawBBox   = false;
	//axisLength = 1.0f;
	Converter::ColorFromName(string("red"  ), axisColorX);
	Converter::ColorFromName(string("green"), axisColorY);
	Converter::ColorFromName(string("blue" ), axisColorZ);
	Converter::ColorFromName(string("cyan" ), comColor  );
	Converter::ColorFromName(string("gray" ), bboxColor );
}

//-------------------------------------------------------------------------------------------------

void SetColor(Vec4f c){
	glMaterialfv(GL_FRONT, GL_DIFFUSE, c);
	glMaterialfv(GL_FRONT, GL_AMBIENT, c);
	glColor4fv(c);
}

void AdaptorSprGR::MaterialAux::Set(GRRenderIf* render){
	// 色名が指定されている場合はこちらを優先
	if(!colorname.empty()){
		// テクスチャ解除のためにデフォルトマテリアルを設定
		render->SetMaterial(GRMaterialDesc());

		Vec4f c = color;
		c[3]    = alpha;
		SetColor(c);
	}
	else{
		render->SetMaterial(mat);
	}
}

//-------------------------------------------------------------------------------------------------
// CalcBound

void AdaptorSprGR::CalcBound(){
	for(uint i = 0; i < bodies.size(); i++)
		bodies[i]->CalcBound();

	sceneBound = 0.0f;

	for(uint i = 0; i < bodies.size(); i++)
		sceneBound = std::max(sceneBound, bodies[i]->aff.Trn().norm() + bodies[i]->bbradius);

}

void AdaptorSprGR::BodyAux::CalcBound(){
	bbradius = 0.0f;

	for(uint i = 0; i < cons.size(); i++){
		for(uint j = 0; j < cons[i]->attaches.size(); j++){
			ShapeAux* sh = cons[i]->attaches[j]->shape;
			bbradius = std::max(bbradius, (cons[i]->aff * sh->bbcenter).norm() + sh->bbradius);
		}
	}
}

void AdaptorSprGR::BoxAux::CalcBound(){
	bbsize = size;
	bbcenter.clear();
	bbradius = 0.5f * bbsize.norm();
}
void AdaptorSprGR::SphereAux::CalcBound(){
	float d = 2.0f * radius;
	bbsize = Vec3f(d, d, d);
	bbcenter.clear();
	bbradius = radius;
}
void AdaptorSprGR::CylinderAux::CalcBound(){
	float d = 2.0f * radius;
	bbsize = Vec3f(d, d, height);
	bbcenter.clear();
	bbradius = 0.5f * bbsize.norm();
}
void AdaptorSprGR::CapsuleAux::CalcBound(){
	float d = 2.0f * radius;
	bbsize = Vec3f(d, d, height);
	bbcenter.clear();
	bbradius = 0.5f * bbsize.norm();
}
void AdaptorSprGR::MeshAux::CalcBound(){
	if(model){
		bbsize   = model->bbsize;
		bbcenter = model->bbcenter;
		bbradius = model->bbradius;
	}
	else{
		bbsize  .clear();
		bbcenter.clear();
		bbradius = 0.0f;
	}
}

//-------------------------------------------------------------------------------------------------
// Create

void AdaptorSprGR::PrimitiveShapeAux::Create(const DrawOption& opt){
	arraySolid .Create(&meshSolid , opt.useVao, false);
	arrayWire  .Create(&meshWire  , opt.useVao, false);
	arrayPoints.Create(&meshPoints, opt.useVao, false);
}
void AdaptorSprGR::BoxAux::Create(const DrawOption& opt){
	meshSolid.type = Mesh::Type::Solid;
	meshSolid.texture_scale  = texture_scale;
	meshSolid.texture_offset = texture_offset;
	meshSolid.Clear();
	meshSolid.Box(size.x, size.y, size.z);

	meshWire .type = Mesh::Type::Wireframe;
	meshWire .Clear();
	meshWire .Box(size.x, size.y, size.z);

	PrimitiveShapeAux::Create(opt);
}
void AdaptorSprGR::SphereAux::Create(const DrawOption& opt){
	meshSolid.type = Mesh::Type::Solid;
	meshSolid.texture_scale  = texture_scale;
	meshSolid.texture_offset = texture_offset;
	meshSolid.Clear();
	meshSolid.Sphere(radius, slice, stack);

	meshWire .type = Mesh::Type::Wireframe;
	meshWire .Clear();
	meshWire .Sphere(radius, slice, stack);

	PrimitiveShapeAux::Create(opt);
}
void AdaptorSprGR::CylinderAux::Create(const DrawOption& opt){
	meshSolid.type = Mesh::Type::Solid;
	meshSolid.texture_scale  = texture_scale;
	meshSolid.texture_offset = texture_offset;
	meshSolid.Clear();
	meshSolid.Cylinder(radius, height, slice);

	meshWire .type = Mesh::Type::Wireframe;
	meshWire .Clear();
	meshWire .Cylinder(radius, height, slice);

	PrimitiveShapeAux::Create(opt);
}
void AdaptorSprGR::CapsuleAux::Create(const DrawOption& opt){
	meshSolid.type = Mesh::Type::Solid;
	meshSolid.texture_scale  = texture_scale;
	meshSolid.texture_offset = texture_offset;
	meshSolid.Clear();
	meshSolid.Capsule(radius, height, slice, stack);

	meshWire .type = Mesh::Type::Wireframe;
	meshWire .Clear();
	meshWire .Capsule(radius, height, slice, stack);

	PrimitiveShapeAux::Create(opt);
}
void AdaptorSprGR::MeshAux::Create(const DrawOption& opt){
	if(!model)
		return;

	materials  .resize(model->materials .size());
	arraySolid .resize(model->meshSolid .size());
	arrayWire  .resize(model->meshWire  .size());
	arrayPoints.resize(model->meshPoints.size());

	AdaptorSprGR* adaptorGR = (AdaptorSprGR*)adaptor;

	for(uint i = 0; i < model->materials.size(); i++)
		adaptorGR->ConvertMaterial(materials[i], model->materials[i], id);
	for(uint i = 0; i < model->meshSolid .size(); i++)
		arraySolid [i].Create(&model->meshSolid [i], opt.useVao, false);
	for(uint i = 0; i < model->meshWire  .size(); i++)
		arrayWire  [i].Create(&model->meshWire  [i], opt.useVao, false);
	for(uint i = 0; i < model->meshPoints.size(); i++)
		arrayPoints[i].Create(&model->meshPoints[i], opt.useVao, false);
}

//-------------------------------------------------------------------------------------------------
// Draw

void AdaptorSprGR::ShapeAux::DrawBBox(GRRenderIf* render, const DrawOption& opt){
	// bounding box 描画
	if(opt.drawBBox){
		render->PushModelMatrix();
		render->MultModelMatrix(Affinef::Trn(bbcenter.x, bbcenter.y, bbcenter.z));
		SetColor(opt.bboxColor);
		render->DrawBox(bbsize.x, bbsize.y, bbsize.z, false);
		render->PopModelMatrix();
	}
}

void AdaptorSprGR::PrimitiveShapeAux::Draw(GRRenderIf* render, const DrawOption& opt){
	if(opt.drawSolid )
		arraySolid .Draw(VertexArray::Primitive::Triangles);
	if(opt.drawWire  )
		arrayWire  .Draw(VertexArray::Primitive::Lines);
	if(opt.drawPoints)
		arrayPoints.Draw(VertexArray::Primitive::Points);
}

void AdaptorSprGR::MeshAux::Draw(GRRenderIf* render, const DrawOption& opt){
	int narray = std::max(std::max( arraySolid.size(), arrayWire.size() ), arrayPoints.size());
	for(uint i = 0; i < narray; i++){
		if(i < model->materialList.size()){
			int imat = model->materialList[i];
			render->SetMaterial(materials[imat]);
		}
		if(opt.drawSolid  && i < arraySolid .size())
			arraySolid [i].Draw(VertexArray::Primitive::Triangles);
		if(opt.drawWire   && i < arrayWire  .size())
			arrayWire  [i].Draw(VertexArray::Primitive::Lines);
		if(opt.drawPoints && i < arrayPoints.size())
			arrayPoints[i].Draw(VertexArray::Primitive::Points);
	}
}

void AdaptorSprGR::MeshAux::DrawBoned(GRRenderIf* render, const DrawOption& opt){
	// コネクタの位置と向きをボーンに反映
	bool boned = false;
	for(uint i = 0; i < attaches.size(); i++){
		AttachAux*    att  = attaches[i];
		ConnectorAux* con  = att->con;
		BodyAux*      body = con->body;
		Bone*         bone = model->GetBone(att->boneIndex);
		if(!bone || !con || !body)
			continue;

		boned = true;
		Affinef aff    = body->aff    * con->aff;
		Affinef affIni = body->affIni * con->affIni;
		bone->SetTransform(aff * affIni.inv());
	}

	if(!boned)
		return;

	render->SetBlending(true);

	int narray = std::max(std::max( arraySolid.size(), arrayWire.size() ), arrayPoints.size());
	for(uint i = 0; i < narray; i++){
		if(i < model->materialList.size()){
			int imat = model->materialList[i];
			render->SetMaterial(materials[imat]);
		}
		if(i < model->bonePaletteList.size()){
			BonePalette& bp = model->bonePaletteList[i];
			// 0番目は単位変換で固定
			render->SetBlendMatrix(Affinef(), 0);
			for(uint j = 1; j < bp.size(); j++){
				Bone* bone = model->GetBone(bp[j]);
				if(bone)
					 render->SetBlendMatrix(bone->affRel, j);
				else render->SetBlendMatrix(Affinef()   , j);
			}
		}
		if(opt.drawSolid  && i < arraySolid .size())
			arraySolid [i].Draw(VertexArray::Primitive::Triangles);
		if(opt.drawWire   && i < arrayWire  .size())
			arrayWire  [i].Draw(VertexArray::Primitive::Lines);
		if(opt.drawPoints && i < arrayPoints.size())
			arrayPoints[i].Draw(VertexArray::Primitive::Points);
	}

	render->SetBlending(false);
}

void AdaptorSprGR::AttachAux::Draw(GRRenderIf* render, const DrawOption& opt){
	if(!con || !con->body)
		return;

	// ボーン参照している場合はここでは描画しない
	if(boneIndex != -1)
		return;

	render->PushModelMatrix();
	// body pose
	render->MultModelMatrix(con->body->aff);
	// connector pose w.r.t. body
	render->MultModelMatrix(con->aff);

	// draw bbox
	shape->DrawBBox(render, opt);

	// apply material
	if(mat)
		mat->Set(render);
	// draw shape
	shape->Draw(render, opt);

	render->PopModelMatrix();
}
void AdaptorSprGR::Draw(GRRenderIf* render){
	// Bodyに割り当てられた形状の描画
	for(uint i = 0; i < attaches.size(); i++)
		attaches[i]->Draw(render, opt);

	// ボーンありメッシュの描画
	for(uint i = 0; i < meshes.size(); i++)
		meshes[i]->DrawBoned(render, opt);
	
	render->SetLighting (false);
	render->SetDepthTest(false);
	if(opt.drawAxis || opt.drawCoM){
		// ワールドフレームの座標軸
		DrawAxis(render, sceneBound);

		// フレームの座標軸の描画
		for(uint i = 0; i < bodies.size(); i++){
			BodyAux* body = bodies[i];

			render->PushModelMatrix();
			render->MultModelMatrix(body->aff);

			if(opt.drawCoM){
				AUTO(BodyProp*, bodyProp, scene->GetProperty(body->id));
				render->SetPointSize(5.0f);
				SetColor(opt.comColor);
				render->DrawPoint(bodyProp->center);
			}
			
			if(opt.drawAxis){
				// 大きさはbounding sphereに合わせる
				render->SetLineWidth(2.0f);
				DrawAxis(render, 0.1f * body->bbradius);

				// コネクタの座標軸の描画
				for(uint j = 0; j < body->cons.size(); j++){
					render->PushModelMatrix();
					render->MultModelMatrix(body->cons[j]->aff);
			
					// コネクタの座標軸は少し小さめに
					render->SetLineWidth(1.0f);
					DrawAxis(render, 0.3f * body->bbradius);

					render->PopModelMatrix();
				}
			}

			render->PopModelMatrix();
		}
		render->SetLineWidth(1.0f);
	}
	render->SetDepthTest(true);
	render->SetLighting(true);
}

void AdaptorSprGR::ConvertMaterial(GRMaterialDesc& grMat, const Material& mat, int id){
	grMat.ambient  = mat.ambient;
	grMat.diffuse  = mat.diffuse;
	grMat.specular = mat.specular;
	grMat.emissive = mat.emissive;
	grMat.power    = mat.shininess;

	if(!mat.texture.empty())
		 grMat.texname = Path(scene->GetLocation(id)) + Path(mat.texture);
	else grMat.texname.clear();
}

//-------------------------------------------------------------------------------------------------

AdaptorSprGR::AdaptorSprGR(){
	sceneBound = 0.0f;

	//glMajorVersion = 2;
	//glMinorVersion = 0;
}

Affinef	AdaptorSprGR::GetBodyPose(int id){
	AUTO(BodyAux*, bodyAux, GetAux(id));
	return bodyAux->aff;
}

float AdaptorSprGR::GetSceneBound(){
	return sceneBound;
}

float AdaptorSprGR::GetBodyBound(int id){
	AUTO(BodyAux*, bodyAux, GetAux(id));
	return bodyAux->bbradius;
}

void AdaptorSprGR::EnableVao(bool on){
	opt.useVao = on;
}

void AdaptorSprGR::ShowSolid(bool on){
	opt.drawSolid = on;
}

void AdaptorSprGR::ShowWireframe(bool on){
	opt.drawWire = on;
}

void AdaptorSprGR::ShowPointcloud(bool on){
	opt.drawPoints = on;
}

void AdaptorSprGR::ShowAxis(bool on){
	opt.drawAxis = on;
}

void AdaptorSprGR::SetAxisColor(const char* x, const char* y, const char* z){
	Converter::ColorFromName(string(x), opt.axisColorX);
	Converter::ColorFromName(string(y), opt.axisColorY);
	Converter::ColorFromName(string(z), opt.axisColorZ);
}

void AdaptorSprGR::ShowCenterOfMass(bool on){
	opt.drawCoM = on;
}

void AdaptorSprGR::SetCenterOfMassColor(const char* c){
	Converter::ColorFromName(string(c), opt.comColor);
}

void AdaptorSprGR::ShowBBox(bool on){
	opt.drawBBox = on;
}

void AdaptorSprGR::SetBBoxColor(const char* c){
	Converter::ColorFromName(string(c), opt.bboxColor);
}

void AdaptorSprGR::DrawAxis(GRRenderIf* render, float length){
	SetColor(opt.axisColorX);
	render->DrawLine(Vec3f(), Vec3f(length, 0.0f, 0.0f));
	SetColor(opt.axisColorY);
	render->DrawLine(Vec3f(), Vec3f(0.0f, length, 0.0f));
	SetColor(opt.axisColorZ);
	render->DrawLine(Vec3f(), Vec3f(0.0f, 0.0f, length));
}

int AdaptorSprGR::CreateObject(int id){
	int type = scene->GetObjectType(id);
	Property* prop = scene->GetProperty(id);
	string name = scene->GetName(id);
	
	if(type == BodyProp::id){
		BodyAux* bodyAux = new BodyAux();
		bodies.push_back(bodyAux);
		RegAux(id, bodyAux);
		Message::Extra("created BodyAux for %s", name.c_str());
		return SupportState::Supported;
	}
	else if(type == ConnectorProp::id){
		int bodyId = scene->FindAncestor(id, BodyProp::id, typedb);
		if(!IsValidID(bodyId) || IsIgnored(bodyId))
			return SupportState::Ignored;
		if(IsUndefined(bodyId))
			return SupportState::Undefined;

		AUTO(BodyAux*, bodyAux, GetAux(bodyId));
		
		RegAux(id, new ConnectorAux(bodyAux));
		Message::Extra("created ConnectorAux for %s", name.c_str());
		return SupportState::Supported;
	}
	else if(type == VisualMaterialProp::id){
		RegAux(id, new MaterialAux());
		Message::Extra("created MaterialAux for %s", name.c_str());
		return SupportState::Supported;
	}
	else if(typedb->KindOf(type, ShapeProp::id)){
		AUTO(ShapeProp*, shapeProp, prop);

		// 視覚化用形状じゃなければ無視
		if(!shapeProp->visual)
			return SupportState::Ignored;

		ShapeAux* shapeAux = 0;

		if(type == BoxProp::id){
			shapeAux = new BoxAux();
			Message::Extra("created BoxAux for %s", name.c_str());
		}
		else if(type == SphereProp::id){
			shapeAux = new SphereAux();
			Message::Extra("created SphereAux for %s", name.c_str());
		}
		else if(type == CylinderProp::id){
			shapeAux = new CylinderAux();
			Message::Extra("created CylinderAux for %s", name.c_str());
		}
		else if(type == CapsuleProp::id){
			shapeAux = new CapsuleAux();
			Message::Extra("created CapsuleAux for %s", name.c_str());
		}
		else if(type == MeshProp::id){
			MeshAux* meshAux = new MeshAux;
			meshes.push_back(meshAux);
			shapeAux = meshAux;
			Message::Extra("created MeshAux for %s", name.c_str());
		}
		if(shapeAux){
			RegAux(id, shapeAux);
			return SupportState::Supported;
		}
	}
	if(type == AttachProp::id){
		int shapeId = scene->GetLink(id, "shape");
		int conId   = scene->GetLink(id, "connector");
		int matId   = scene->GetLink(id, "vmat");
		
		if(!IsValidID(shapeId) || IsIgnored(shapeId)){
			Message::Out("%s: no valid link to shape. skipped", name.c_str());
			return SupportState::Ignored;
		}
		if(!typedb->KindOf(scene->GetObjectType(shapeId), ShapeProp::id)){
			Message::Error("%s: shape must link to Shape", name.c_str());
			return SupportState::Ignored;
		}
		if(!IsValidID(conId) || IsIgnored(conId)){
			Message::Out("%s: no valid link to connector. skipped", name.c_str());
			return SupportState::Ignored;
		}
		if(scene->GetObjectType(conId) != ConnectorProp::id){
			Message::Error("%s: connector must link to Connector", name.c_str());
			return SupportState::Ignored;
		}
		if(IsUndefined(shapeId) || IsUndefined(conId))
			return SupportState::Undefined;
		
		// visual material is optional
		if(IsValidID(matId) && IsUndefined(matId))
			return SupportState::Undefined;
		
		AUTO(ShapeAux*,     shapeAux, GetAux(shapeId));
		AUTO(ConnectorAux*, conAux,   GetAux(conId));
		AUTO(MaterialAux*,  matAux,   GetAux(matId));

		AttachAux* attachAux = new AttachAux(shapeAux, 0, 0, conAux, matAux);
		Message::Extra("created AttachAux for %s", name.c_str());
				
		attaches.push_back(attachAux);
		RegAux(id, attachAux);
		return SupportState::Supported;
	}
	return SupportState::Ignored;
}

void AdaptorSprGR::DeleteObject(int id){
	int type = scene->GetObjectType(id);
	Aux* aux = GetAux(id);
	
	if(type == BodyProp::id){
		AUTO(BodyAux*, bodyAux, aux);
		RemoveFromArray(bodies, bodyAux);
	}
	else if(type == ConnectorProp::id){

	}
	else if(type == VisualMaterialProp::id){

	}
	else if(typedb->KindOf(type, ShapeProp::id)){
		if(type == MeshProp::id){
			AUTO(MeshAux*, meshAux, aux);
			RemoveFromArray(meshes, meshAux);
		}
	}
	if(type == AttachProp::id){
		AUTO(AttachAux*, attachAux, aux);
		RemoveFromArray(attaches, attachAux);
	}

	RemoveAux(id);
}

void AdaptorSprGR::SyncObjectProperty(int id, bool download, int cat){
	if(!IsSupported(id))
		return;
	int type = scene->GetObjectType(id);
	Property* prop = scene->GetProperty(id);

	if(type == BodyProp::id){
		// Bodyの位置をフレームに反映
		AUTO(BodyProp*, bodyProp, prop);
		AUTO(BodyAux*, bodyAux, GetAux(id));
		if(cat & AttrCategory::State){
			Transform tr;
			int par = scene->FindAncestor(id, SpatialObjectProp::id, typedb);
			if(par != -1)
				scene->CalcRelativeTransform(-1, par, typedb, tr);

			Affinef aff;
			pose_t pose;
			if(download){
				pose.Pos() = tr.rot * bodyProp->trn + tr.trn;
				pose.Ori() = tr.rot * bodyProp->rot;
				pose.ToAffine(bodyAux->aff);
				if(bodyAux->initial){
					bodyAux->affIni  = bodyAux->aff;
					bodyAux->initial = false;
				}
			}
			else{
				pose.FromAffine(bodyAux->aff);
				quat_t qinv = tr.rot.Conjugated();
				bodyProp->trn = qinv * (pose.Pos() - tr.trn);
				bodyProp->rot = qinv * pose.Ori();
			}
		}
	}
	else if(type == ConnectorProp::id){
		AUTO(ConnectorProp*, conProp, prop);
		AUTO(ConnectorAux*, conAux, GetAux(id));
		if(cat & AttrCategory::State){
			if(download){
				// calculate relative pose between owner body
				int bodyId = scene->FindAncestor(id, BodyProp::id, typedb);
				Transform trans;
				scene->CalcRelativeTransform(bodyId, id, typedb, trans);
				pose_t(trans.trn, trans.rot).ToAffine(conAux->aff);
				if(conAux->initial){
					conAux->affIni  = conAux->aff;
					conAux->initial = false;
				}
			}
		}
	}
	else if(type == AttachProp::id){
		AUTO(AttachProp*, attachProp, prop);
		AUTO(AttachAux* , attachAux , GetAux(id));
		if(cat & AttrCategory::Param){
			if(download){
				attachAux->bone = attachProp->bone;
				if(attachAux->shape){
					attachAux->boneIndex = attachAux->shape->GetBoneIndex(attachAux->bone);
				}
			}
		}
	}
	else if(typedb->KindOf(type, ShapeProp::id)){
		AUTO(ShapeProp*, shapeProp, prop);
		AUTO(ShapeAux* , shapeAux , GetAux(id));
		if(cat & AttrCategory::Param){
			if(download){
				shapeAux->auto_texcoord  = shapeProp->AutoTexCoord();
				shapeAux->texture_scale  = shapeProp->texture_scale;
				shapeAux->texture_offset = shapeProp->texture_offset;
			}
		}

		if(type == BoxProp::id){
			AUTO(BoxProp*, boxProp, prop);
			AUTO(BoxAux*, boxAux, GetAux(id));
			if(cat & AttrCategory::Param){
				if(download){
					boxAux->size = boxProp->size;
					boxAux->Create(opt);
					boxAux->CalcBound();
				}
				else{
					boxProp->size = boxAux->size;
				}
			}
		}
		else if(type == SphereProp::id){
			AUTO(SphereProp*, sphereProp, prop);
			AUTO(SphereAux*, sphereAux, GetAux(id));
			if(cat & AttrCategory::Param){
				if(download){
					sphereAux->radius = (float)sphereProp->radius;
					sphereAux->slice  = sphereProp->slice;
					sphereAux->stack  = sphereProp->stack;
					sphereAux->Create(opt);
					sphereAux->CalcBound();
				}
				else{
					sphereProp->radius = (real_t)sphereAux->radius;
					sphereProp->slice  = (uint  )sphereAux->slice ;
					sphereProp->stack  = (uint  )sphereAux->stack ;
				}
			}
		}
		else if(type == CylinderProp::id){
			AUTO(CylinderProp*, cylinderProp, prop);
			AUTO(CylinderAux*, cylinderAux, GetAux(id));
			if(cat & AttrCategory::Param){
				if(download){
					cylinderAux->radius = (float)cylinderProp->radius;
					cylinderAux->height = (float)cylinderProp->height;
					cylinderAux->slice  = (uint )cylinderProp->slice ;
					cylinderAux->Create(opt);
					cylinderAux->CalcBound();
				}
				else{
					cylinderProp->radius = (real_t)cylinderAux->radius;
					cylinderProp->height = (real_t)cylinderAux->height;
					cylinderProp->slice  = (uint  )cylinderAux->slice ;
				}
			}
		}
		else if(type == CapsuleProp::id){
			AUTO(CapsuleProp*, capsuleProp, prop);
			AUTO(CapsuleAux* , capsuleAux , GetAux(id));
			if(cat & AttrCategory::Param){
				if(download){
					capsuleAux->radius = (float)capsuleProp->radius;
					capsuleAux->height = (float)capsuleProp->height;
					capsuleAux->slice  = (uint )capsuleProp->slice ;
					capsuleAux->stack  = (uint )capsuleProp->stack ;
					capsuleAux->Create(opt);
					capsuleAux->CalcBound();
				}
				else{
					capsuleProp->radius = (real_t)capsuleAux->radius;
					capsuleProp->height = (real_t)capsuleAux->height;
					capsuleProp->slice  = (uint  )capsuleAux->slice ;
					capsuleProp->stack  = (uint  )capsuleAux->stack ;
				}
			}
		}
		else if(type == MeshProp::id){
			AUTO(MeshProp*, meshProp, prop);
			AUTO(MeshAux*, meshAux, GetAux(id));
			if(cat & AttrCategory::Param && download){
				// ファイル名が変わっていたらロードする
				if(!(meshAux->filename == meshProp->filename)){
					// モデルコンテナから取得
					Model* model = models->GetModel(id);
					if(!model)
						Message::Error("model %s not found in the container", (const char*)meshProp->filename);
					meshAux->model = model;
					meshAux->filename = meshProp->filename;
					meshAux->Create(opt);
					meshAux->CalcBound();
				}
			}
		}
	}
	else if(type == VisualMaterialProp::id){
		AUTO(VisualMaterialProp*, matProp, prop);
		AUTO(MaterialAux*, matAux, GetAux(id));
		if(cat & AttrCategory::Param){
			if(download){
				matAux->mat.ambient  = matProp->ambient;
				matAux->mat.diffuse  = matProp->diffuse;
				matAux->mat.specular = matProp->specular;
				matAux->mat.emissive = matProp->emissive;
				matAux->mat.power    = (float)matProp->shininess;
				if(!(matProp->texture == ""))
					matAux->mat.texname  = Path(scene->GetLocation(id)) + Path(string(matProp->texture));

				matAux->colorname      = matProp->colorname;
				matAux->alpha          = (float)matProp->alpha;
				Converter::ColorFromName(string(matProp->colorname), matAux->color);
			}
		}
	}
}
/*
int AdaptorSprGR::ColorFromName(const char* name){
	if(!strcmp(name, "indianred"           )) return GRRenderBaseIf::INDIANRED;
	if(!strcmp(name, "lightcoral"          )) return GRRenderBaseIf::LIGHTCORAL;
	if(!strcmp(name, "salmon"              )) return GRRenderBaseIf::SALMON;
	if(!strcmp(name, "darksalmon"          )) return GRRenderBaseIf::DARKSALMON;
	if(!strcmp(name, "lightsalmon"         )) return GRRenderBaseIf::LIGHTSALMON;
	if(!strcmp(name, "red"                 )) return GRRenderBaseIf::RED;
	if(!strcmp(name, "crimson"             )) return GRRenderBaseIf::CRIMSON;
	if(!strcmp(name, "firebrick"           )) return GRRenderBaseIf::FIREBRICK;
	if(!strcmp(name, "darkred"             )) return GRRenderBaseIf::DARKRED;
	if(!strcmp(name, "pink"                )) return GRRenderBaseIf::PINK;
	if(!strcmp(name, "lightpink"           )) return GRRenderBaseIf::LIGHTPINK;
	if(!strcmp(name, "hotpink"             )) return GRRenderBaseIf::HOTPINK;
	if(!strcmp(name, "deeppink"            )) return GRRenderBaseIf::DEEPPINK;
	if(!strcmp(name, "mudiumvioletred"     )) return GRRenderBaseIf::MEDIUMVIOLETRED;
	if(!strcmp(name, "palevioletred"       )) return GRRenderBaseIf::PALEVIOLETRED;
	if(!strcmp(name, "coral"               )) return GRRenderBaseIf::CORAL;
	if(!strcmp(name, "tomato"              )) return GRRenderBaseIf::TOMATO;
	if(!strcmp(name, "orangered"           )) return GRRenderBaseIf::ORANGERED;
	if(!strcmp(name, "darkorange"          )) return GRRenderBaseIf::DARKORANGE;
	if(!strcmp(name, "orange"              )) return GRRenderBaseIf::ORANGE;
	if(!strcmp(name, "gold"                )) return GRRenderBaseIf::GOLD;
	if(!strcmp(name, "yellow"              )) return GRRenderBaseIf::YELLOW;
	if(!strcmp(name, "lightyellow"         )) return GRRenderBaseIf::LIGHTYELLOW;
	if(!strcmp(name, "lemonchiffon"        )) return GRRenderBaseIf::LEMONCHIFFON;
	if(!strcmp(name, "lightgoldenrodyellow")) return GRRenderBaseIf::LIGHTGOLDENRODYELLOW;
	if(!strcmp(name, "papayawhip"          )) return GRRenderBaseIf::PAPAYAWHIP;
	if(!strcmp(name, "moccasin"            )) return GRRenderBaseIf::MOCCASIN;
	if(!strcmp(name, "peachpuff"           )) return GRRenderBaseIf::PEACHPUFF;
	if(!strcmp(name, "palegoldenrod"       )) return GRRenderBaseIf::PALEGOLDENROD;
	if(!strcmp(name, "khaki"               )) return GRRenderBaseIf::KHAKI;
	if(!strcmp(name, "darkkhaki"           )) return GRRenderBaseIf::DARKKHAKI;
	if(!strcmp(name, "lavender"            )) return GRRenderBaseIf::LAVENDER;
	if(!strcmp(name, "thistle"             )) return GRRenderBaseIf::THISTLE;
	if(!strcmp(name, "plum"                )) return GRRenderBaseIf::PLUM;
	if(!strcmp(name, "violet"              )) return GRRenderBaseIf::VIOLET;
	if(!strcmp(name, "orchild"             )) return GRRenderBaseIf::ORCHILD;
	if(!strcmp(name, "fuchsia"             )) return GRRenderBaseIf::FUCHSIA;
	if(!strcmp(name, "magenta"             )) return GRRenderBaseIf::MAGENTA;
	if(!strcmp(name, "mediumorchild"       )) return GRRenderBaseIf::MEDIUMORCHILD;
	if(!strcmp(name, "mediumpurple"        )) return GRRenderBaseIf::MEDIUMPURPLE;
	if(!strcmp(name, "blueviolet"          )) return GRRenderBaseIf::BLUEVIOLET;
	if(!strcmp(name, "darkviolet"          )) return GRRenderBaseIf::DARKVIOLET;
	if(!strcmp(name, "darkorchild"         )) return GRRenderBaseIf::DARKORCHILD;
	if(!strcmp(name, "darkmagenta"         )) return GRRenderBaseIf::DARKMAGENTA;
	if(!strcmp(name, "purple"              )) return GRRenderBaseIf::PURPLE;
	if(!strcmp(name, "indigo"              )) return GRRenderBaseIf::INDIGO;
	if(!strcmp(name, "darkslateblue"       )) return GRRenderBaseIf::DARKSLATEBLUE;
	if(!strcmp(name, "slateblue"           )) return GRRenderBaseIf::SLATEBLUE;
	if(!strcmp(name, "mediumslateblue"     )) return GRRenderBaseIf::MEDIUMSLATEBLUE;
	if(!strcmp(name, "greenyellow"         )) return GRRenderBaseIf::GREENYELLOW;
	if(!strcmp(name, "chartreuse"          )) return GRRenderBaseIf::CHARTREUSE;
	if(!strcmp(name, "lawngreen"           )) return GRRenderBaseIf::LAWNGREEN;
	if(!strcmp(name, "lime"                )) return GRRenderBaseIf::LIME;
	if(!strcmp(name, "limegreen"           )) return GRRenderBaseIf::LIMEGREEN;
	if(!strcmp(name, "palegreen"           )) return GRRenderBaseIf::PALEGREEN;
	if(!strcmp(name, "lightgreen"          )) return GRRenderBaseIf::LIGHTGREEN;
	if(!strcmp(name, "mediumspringgreen"   )) return GRRenderBaseIf::MEDIUMSPRINGGREEN;
	if(!strcmp(name, "springgreen"         )) return GRRenderBaseIf::SPRINGGREEN;
	if(!strcmp(name, "mediumseagreen"      )) return GRRenderBaseIf::MEDIUMSEAGREEN;
	if(!strcmp(name, "seagreen"            )) return GRRenderBaseIf::SEAGREEN;
	if(!strcmp(name, "forestgreen"         )) return GRRenderBaseIf::FORESTGREEN;
	if(!strcmp(name, "green"               )) return GRRenderBaseIf::GREEN;
	if(!strcmp(name, "darkgreen"           )) return GRRenderBaseIf::DARKGREEN;
	if(!strcmp(name, "yellowgreen"         )) return GRRenderBaseIf::YELLOWGREEN;
	if(!strcmp(name, "olivedrab"           )) return GRRenderBaseIf::OLIVEDRAB;
	if(!strcmp(name, "olive"               )) return GRRenderBaseIf::OLIVE;
	if(!strcmp(name, "darkolivegreen"      )) return GRRenderBaseIf::DARKOLIVEGREEN;
	if(!strcmp(name, "mediumaquamarine"    )) return GRRenderBaseIf::MEDIUMAQUAMARINE;
	if(!strcmp(name, "darkseagreen"        )) return GRRenderBaseIf::DARKSEAGREEN;
	if(!strcmp(name, "lightseagreen"       )) return GRRenderBaseIf::LIGHTSEAGREEN;
	if(!strcmp(name, "darkcyan"            )) return GRRenderBaseIf::DARKCYAN;
	if(!strcmp(name, "teal"                )) return GRRenderBaseIf::TEAL;
	if(!strcmp(name, "aqua"                )) return GRRenderBaseIf::AQUA;
	if(!strcmp(name, "cyan"                )) return GRRenderBaseIf::CYAN;
	if(!strcmp(name, "lightcyan"           )) return GRRenderBaseIf::LIGHTCYAN;
	if(!strcmp(name, "paleturquoise"       )) return GRRenderBaseIf::PALETURQUOISE;
	if(!strcmp(name, "aquamarine"          )) return GRRenderBaseIf::AQUAMARINE;
	if(!strcmp(name, "turquoise"           )) return GRRenderBaseIf::TURQUOISE;
	if(!strcmp(name, "mediumturquoise"     )) return GRRenderBaseIf::MEDIUMTURQUOISE;
	if(!strcmp(name, "darkturquoise"       )) return GRRenderBaseIf::DARKTURQUOISE;
	if(!strcmp(name, "cadetblue"           )) return GRRenderBaseIf::CADETBLUE;
	if(!strcmp(name, "steelblue"           )) return GRRenderBaseIf::STEELBLUE;
	if(!strcmp(name, "lightsteelblue"      )) return GRRenderBaseIf::LIGHTSTEELBLUE;
	if(!strcmp(name, "powderblue"          )) return GRRenderBaseIf::POWDERBLUE;
	if(!strcmp(name, "lightblue"           )) return GRRenderBaseIf::LIGHTBLUE;
	if(!strcmp(name, "skyblue"             )) return GRRenderBaseIf::SKYBLUE;
	if(!strcmp(name, "lightskyblue"        )) return GRRenderBaseIf::LIGHTSKYBLUE;
	if(!strcmp(name, "deepskyblue"         )) return GRRenderBaseIf::DEEPSKYBLUE;
	if(!strcmp(name, "dodgerblue"          )) return GRRenderBaseIf::DODGERBLUE;
	if(!strcmp(name, "cornflowerblue"      )) return GRRenderBaseIf::CORNFLOWERBLUE;
	if(!strcmp(name, "royalblue"           )) return GRRenderBaseIf::ROYALBLUE;
	if(!strcmp(name, "blue"                )) return GRRenderBaseIf::BLUE;
	if(!strcmp(name, "mediumblue"          )) return GRRenderBaseIf::MEDIUMBLUE;
	if(!strcmp(name, "darkblue"            )) return GRRenderBaseIf::DARKBLUE;
	if(!strcmp(name, "navy"                )) return GRRenderBaseIf::NAVY;
	if(!strcmp(name, "midnightblue"        )) return GRRenderBaseIf::MIDNIGHTBLUE;
	if(!strcmp(name, "cornsilk"            )) return GRRenderBaseIf::CORNSILK;
	if(!strcmp(name, "blanchedalmond"      )) return GRRenderBaseIf::BLANCHEDALMOND;
	if(!strcmp(name, "bisque"              )) return GRRenderBaseIf::BISQUE;
	if(!strcmp(name, "navajowhite"         )) return GRRenderBaseIf::NAVAJOWHITE;
	if(!strcmp(name, "wheat"               )) return GRRenderBaseIf::WHEAT;
	if(!strcmp(name, "burlywood"           )) return GRRenderBaseIf::BURLYWOOD;
	if(!strcmp(name, "tan"                 )) return GRRenderBaseIf::TAN;
	if(!strcmp(name, "rosybrown"           )) return GRRenderBaseIf::ROSYBROWN;
	if(!strcmp(name, "sandybrown"          )) return GRRenderBaseIf::SANDYBROWN;
	if(!strcmp(name, "goldenrod"           )) return GRRenderBaseIf::GOLDENROD;
	if(!strcmp(name, "darkgoldenrod"       )) return GRRenderBaseIf::DARKGOLDENROD;
	if(!strcmp(name, "peru"                )) return GRRenderBaseIf::PERU;
	if(!strcmp(name, "chocolate"           )) return GRRenderBaseIf::CHOCOLATE;
	if(!strcmp(name, "saddlebrown"         )) return GRRenderBaseIf::SADDLEBROWN;
	if(!strcmp(name, "sienna"              )) return GRRenderBaseIf::SIENNA;
	if(!strcmp(name, "brown"               )) return GRRenderBaseIf::BROWN;
	if(!strcmp(name, "maroon"              )) return GRRenderBaseIf::MAROON;
	if(!strcmp(name, "white"               )) return GRRenderBaseIf::WHITE;
	if(!strcmp(name, "snow"                )) return GRRenderBaseIf::SNOW;
	if(!strcmp(name, "honeydew"            )) return GRRenderBaseIf::HONEYDEW;
	if(!strcmp(name, "mintcream"           )) return GRRenderBaseIf::MINTCREAM;
	if(!strcmp(name, "azure"               )) return GRRenderBaseIf::AZURE;
	if(!strcmp(name, "aliceblue"           )) return GRRenderBaseIf::ALICEBLUE;
	if(!strcmp(name, "ghostwhite"          )) return GRRenderBaseIf::GHOSTWHITE;
	if(!strcmp(name, "whitesmoke"          )) return GRRenderBaseIf::WHITESMOKE;
	if(!strcmp(name, "seashell"            )) return GRRenderBaseIf::SEASHELL;
	if(!strcmp(name, "beige"               )) return GRRenderBaseIf::BEIGE;
	if(!strcmp(name, "oldlace"             )) return GRRenderBaseIf::OLDLACE;
	if(!strcmp(name, "floralwhite"         )) return GRRenderBaseIf::FLORALWHITE;
	if(!strcmp(name, "ivory"               )) return GRRenderBaseIf::IVORY;
	if(!strcmp(name, "antiquewhite"        )) return GRRenderBaseIf::ANTIQUEWHITE;
	if(!strcmp(name, "linen"               )) return GRRenderBaseIf::LINEN;
	if(!strcmp(name, "lavenderblush"       )) return GRRenderBaseIf::LAVENDERBLUSH;
	if(!strcmp(name, "mistyrose"           )) return GRRenderBaseIf::MISTYROSE;
	if(!strcmp(name, "gainsboro"           )) return GRRenderBaseIf::GAINSBORO;
	if(!strcmp(name, "lightgray"           )) return GRRenderBaseIf::LIGHTGRAY;
	if(!strcmp(name, "silver"              )) return GRRenderBaseIf::SILVER;
	if(!strcmp(name, "darkgray"            )) return GRRenderBaseIf::DARKGRAY;
	if(!strcmp(name, "gray"                )) return GRRenderBaseIf::GRAY;
	if(!strcmp(name, "dimgray"             )) return GRRenderBaseIf::DIMGRAY;
	if(!strcmp(name, "lightslategray"      )) return GRRenderBaseIf::LIGHTSLATEGRAY;
	if(!strcmp(name, "slategray"           )) return GRRenderBaseIf::SLATEGRAY;
	if(!strcmp(name, "darkslategray"       )) return GRRenderBaseIf::DARKSLATEGRAY;
	if(!strcmp(name, "black"               )) return GRRenderBaseIf::BLACK;
	return -1;
}
*/

}
