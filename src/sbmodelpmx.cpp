#include <sbmodelpmx.h>
#include <sbmessage.h>
#include <sbconverter.h>

#include <boost/cstdint.hpp>
using namespace boost;

namespace Scenebuilder{;

LoaderPMX::LoaderPMX(){
	Clear();
}

void LoaderPMX::Clear(){

}

void LoaderPMX::GetIndex(int sz, int* idx){
	if(sz == 1){
		int8_t i;
		Get(&i);
		*idx = i;
	}
	if(sz == 2){
		int16_t i;
		Get(&i);
		*idx = i;
	}
	if(sz == 4){
		int32_t i;
		Get(&i);
		*idx = i;
	}
}

void LoaderPMX::GetVertexIndex(int* idx){
	GetIndex(header.vertexIndex, idx);
}

void LoaderPMX::GetTextureIndex(int* idx){
	GetIndex(header.textureIndex, idx);
}

void LoaderPMX::GetBoneIndex(int* idx){
	GetIndex(header.boneIndex, idx);
}

void LoaderPMX::GetBodyIndex(int* idx){
	GetIndex(header.bodyIndex, idx);
}

void LoaderPMX::ParseBinary(){
	ptr = (byte*)&contents[0];
	ptr_end = ptr + contents.size();

	// ヘッダ
	Get(&header);
	// 2.0, UTF16限定
	if(header.version != 2.0){
		Message::Error("PMX version must be 2.0");
		throw Failure();
	}
	if(header.encoding != 0){
		Message::Error("PMX character encoding must be UTF16");
		throw Failure();
	}

	// モデル情報
	int sz;
	Get(&sz); GetNString(sz/2, info.modelName  );
	Get(&sz); GetNString(sz/2, info.modelNameEn);
	Get(&sz); GetNString(sz/2, info.comment    );
	Get(&sz); GetNString(sz/2, info.commentEn  );

	// 頂点
	int nVertices;
	Get(&nVertices);
	vertices.resize(nVertices);
	for(int i = 0; i < nVertices; i++){
		Vertex& v = vertices[i];
		Get(&v.pos);
		Get(&v.normal);
		Get(&v.uv);
		byte blend;
		Get(&blend);
		if(blend == Blending::BDEF1){
			GetBoneIndex(&v.bone[0]);
			v.bone[1] = -1;
			v.weight = 1.0f;
		}
		if(blend == Blending::BDEF2){
			GetBoneIndex(&v.bone[0]);
			GetBoneIndex(&v.bone[1]);
			Get(&v.weight);
		}
		if(blend == Blending::BDEF4){
			GetBoneIndex(&v.bone[0]);
			GetBoneIndex(&v.bone[1]);
			Skip(2*header.boneIndex);
			Get(&v.weight);
			Skip(3*sizeof(float));
		}
		if(blend == Blending::SDEF){
			GetBoneIndex(&v.bone[0]);
			GetBoneIndex(&v.bone[1]);
			Get(&v.weight);
			Skip(3*3*sizeof(float));
		}
		Get(&v.edge);
	}

	int nFaces;
	Get(&nFaces);
	nFaces /= 3;	///< 面数ではなく総頂点数が入っている？
	faces.resize(nFaces);
	for(int i = 0; i < nFaces; i++){
		Face& f = faces[i];
		GetVertexIndex(&f.vertices[0]);
		GetVertexIndex(&f.vertices[1]);
		GetVertexIndex(&f.vertices[2]);
	}

	int nTextures;
	Get(&nTextures);
	textures.resize(nTextures);
	wstring wstr;
	for(int i = 0; i < nTextures; i++){
		Texture& tex = textures[i];
		Get(&sz); GetNString(sz/2, wstr);
		Converter::ConvertString(tex.filename, wstr);
	}

	int nMaterials;
	Get(&nMaterials);
	materials.resize(nMaterials);
	for(int i = 0; i < nMaterials; i++){
		Material& mat = materials[i];
		Get(&sz); GetNString(sz/2, mat.name);
		Get(&sz); GetNString(sz/2, mat.nameEn);
		Get(&mat.diffuse);
		Get(&mat.specular);
		Get(&mat.specularCoef);
		Get(&mat.ambient);
		Get(&mat.flag);
		Get(&mat.edgeColor);
		Get(&mat.edgeSize);
		GetTextureIndex(&mat.texture);
		GetTextureIndex(&mat.textureSphere);
		Get(&mat.sphereMode);
		Get(&mat.toonFlag);
		if(mat.toonFlag == 0)
			GetTextureIndex(&mat.textureToon);
		else{
			byte commonToon;
			Get(&commonToon);
			mat.textureToon = commonToon;
		}
		Get(&sz); GetNString(sz/2, mat.note);
		// ここも面数ではなく頂点数で入っているので3で割る
		Get(&mat.nFaces);
		mat.nFaces /= 3;
	}

	int nBones;
	Get(&nBones);
	bones.resize(nBones);
	for(int i = 0; i < nBones; i++){
		Bone& bone = bones[i];
		Get(&sz); GetNString(sz/2, bone.name);
		Get(&sz); GetNString(sz/2, bone.nameEn);
		Get(&bone.pos);
		GetBoneIndex(&bone.parent);
		Get(&bone.hierarchy);
		Get(&bone.flag);
		if(bone.flag & Bone::Connection)
			 Skip(header.boneIndex);
		else Skip(sizeof(Vec3f));
		if(bone.flag & Bone::AddRotate){
			Skip(header.boneIndex);
			Skip(sizeof(float));
		}
		if(bone.flag & Bone::AddTranslate){
			Skip(header.boneIndex);
			Skip(sizeof(float));
		}
		if(bone.flag & Bone::FixAxis)
			Skip(sizeof(Vec3f));
		if(bone.flag & Bone::LocalAxis){
			Get(&bone.localX);
			Get(&bone.localZ);
		}
		else{
			bone.localX = Vec3f(1.0f, 0.0f, 0.0f);
			bone.localZ = Vec3f(0.0f, 0.0f, 1.0f);
		}
		if(bone.flag & Bone::External)
			Skip(sizeof(int));
		if(bone.flag & Bone::IK){
			GetBoneIndex(&bone.ikTarget);
			Get(&bone.ikIter);
			Get(&bone.ikAngleDiffLimit);
			int nLinks;
			Get(&nLinks);
			bone.ikLinks.resize(nLinks);
			for(int j = 0; j < nLinks; j++){
				Bone::IKLink& link = bone.ikLinks[j];
				GetBoneIndex(&link.index);
				Get(&link.angleLimit);
				if(link.angleLimit){
					Get(&link.angleMin);
					Get(&link.angleMax);
				}
			}
		}
	}

	/// モーフと表示枠は読み飛ばす
	int nMorphs;
	Get(&nMorphs);
	morphs.resize(nMorphs);
	for(int i = 0; i < nMorphs; i++){
		Morph& morph = morphs[i];
		Get(&sz); GetNString(sz/2, morph.name);
		Get(&sz); GetNString(sz/2, morph.nameEn);
		Skip(sizeof(byte));
		Get(&morph.type);
		int num;
		Get(&num);
		for(int j = 0; j < num; j++){
			if(morph.type == Morph::Group){
				Skip(header.morphIndex);
				Skip(sizeof(float));
			}
			if(morph.type == Morph::Vertex){
				Skip(header.vertexIndex);
				Skip(sizeof(Vec3f));
			}
			if(morph.type == Morph::Bone){
				Skip(header.boneIndex);
				Skip(sizeof(Vec3f));
				Skip(sizeof(Vec4f));
			}
			if(morph.type >= Morph::UV && morph.type <= Morph::UV4){
				Skip(header.vertexIndex);
				Skip(sizeof(Vec4f));
			}
			if(morph.type == Morph::Material){
				Skip(header.materialIndex);
				Skip(sizeof(byte));
				Skip(sizeof(Vec4f));
				Skip(sizeof(Vec3f));
				Skip(sizeof(float));
				Skip(sizeof(Vec3f));
				Skip(sizeof(Vec4f));
				Skip(sizeof(float));
				Skip(sizeof(Vec4f));
				Skip(sizeof(Vec4f));
				Skip(sizeof(Vec4f));
			}
		}
	}

	int nFrames;
	Get(&nFrames);
	frames.resize(nFrames);
	for(int i = 0; i < nFrames; i++){
		Frame& frame = frames[i];
		Get(&sz); GetNString(sz/2, frame.name);
		Get(&sz); GetNString(sz/2, frame.nameEn);
		Skip(sizeof(byte));
		int num;
		Get(&num);
		for(int j = 0; j < num; j++){
			byte type;
			Get(&type);
			if(type == 0)
				 Skip(header.boneIndex);
			else Skip(header.morphIndex);
		}
	}

	int nBodies;
	Get(&nBodies);
	bodies.resize(nBodies);
	for(int i = 0; i < nBodies; i++){
		Body& body = bodies[i];
		Get(&sz); GetNString(sz/2, body.name);
		Get(&sz); GetNString(sz/2, body.nameEn);
		GetBoneIndex(&body.bone);
		Get(&body.group);
		Get(&body.noCollision);
		Get(&body.shape);
		Get(&body.size);
		Get(&body.pos);
		Get(&body.rot);
		Get(&body.mass);
		Get(&body.trnDecay);
		Get(&body.rotDecay);
		Get(&body.cor);
		Get(&body.friction);
		Get(&body.type);
	}

	int nJoints;
	Get(&nJoints);
	joints.resize(nJoints);
	for(int i = 0; i < nJoints; i++){
		Joint& joint = joints[i];
		Get(&sz); GetNString(sz/2, joint.name);
		Get(&sz); GetNString(sz/2, joint.nameEn);
		Skip(sizeof(byte));
		GetBodyIndex(&joint.body[0]);
		GetBodyIndex(&joint.body[1]);
		Get(&joint.pos);
		Get(&joint.rot);
		Get(&joint.posMin);
		Get(&joint.posMax);
		Get(&joint.rotMin);
		Get(&joint.rotMax);
		Get(&joint.posSpring);
		Get(&joint.rotSpring);
	}

}

int LoaderPMX::GetFaceMaterial(int f){
	for(uint i = 0; i < materials.size(); i++){
		Material& mat = materials[i];
		if(f < mat.nFaces)
			return i;
		f -= mat.nFaces;
	}
	return -1;
}

void LoaderPMX::Load(const string& filename){
	ifstream ifs;
	// 改行コードの変換を避けるためbinary指定
	ifs.open(filename.c_str(), ios_base::in | ios_base::binary);

	if(!ifs.is_open())
		throw FileError();

	ifs.seekg(0, ifs.end);
	int len = (int)ifs.tellg();
	ifs.seekg(0, ifs.beg);

	contents.resize(len);
	ifs.read(&contents[0], len);
	
	ifs.close();
	
	// parse
	ParseBinary();

	contents.clear();
}

void LoaderPMX::Convert(Model* model, const Affinef& aff){
	uint iBegin = 0;
	uint iEnd   = 0;

	model->materials   .resize(materials.size());
	model->bones       .resize(bones    .size());

	// マテリアルコピー
	for(uint i = 0; i < materials.size(); i++){
		Scenebuilder::Material& md = model->materials[i];
		LoaderPMX   ::Material& ms =        materials[i];

		md.ambient   = Vec4f(ms.ambient [0], ms.ambient [1], ms.ambient [2], 1.0f);
		md.diffuse   = ms.diffuse;
		md.specular  = Vec4f(ms.specular[0], ms.specular[1], ms.specular[2], 1.0f);
		md.shininess = ms.specularCoef;
		md.texture   = textures[ms.texture].filename;
	}

	// ボーンコピー
	for(uint i = 0; i < bones.size(); i++){
		Scenebuilder::Bone* bd = &model->bones[i];
		LoaderPMX   ::Bone* bs = &bones[i];

		Converter::ConvertString(bd->name, bs->name);
		bd->affIni.Trn() = bs->pos;
		bd->affIni.Ex () = bs->localX;
		bd->affIni.Ez () = bs->localZ;
		bd->affIni.Ey () = bd->affIni.Ez() % bd->affIni.Ex();
		bd->affIni.Ez () = bd->affIni.Ex() % bd->affIni.Ey();
		if(bs->parent != -1)
			 bd->SetParent(&model->bones[bs->parent]);
		else bd->SetParent(0);
	}
	model->InitBone();

	// メッシュコピー 
	//  同じマテリアル・ボーンインデックスごとに分解
	int imat   = -1;
	BonePalette ibone(model->bonePaletteSize);
	Mesh* m[2] = {0, 0};

	Face* face;
	Vertex* v[3];
	int im;
	int ib[3][2];
	for(uint f = 0; f < faces.size(); f++){
		face = &faces[f];
		v[0] = &vertices[face->vertices[0]];
		v[1] = &vertices[face->vertices[1]];
		v[2] = &vertices[face->vertices[2]];

		bool create = false;
		im  = GetFaceMaterial(f);
		create |= (imat != im);
		for(int i = 0; i < 3; i++)for(int j = 0; j < 2; j++){
			ib[i][j] = ibone.Find(v[i]->bone[j]);
			create |= (ib[i][j] == -1);
		}

		// マテリアルが変わった/ボーンパレットの空きが無くなったら新しいメッシュを作成
		if(create){
			if(f != 0){
				model->materialList   .push_back(imat );
				model->bonePaletteList.push_back(ibone);
			}
			model->meshSolid.push_back(Mesh());
			model->meshWire .push_back(Mesh());
			m[0] = &model->meshSolid.back();
			m[1] = &model->meshWire .back();
			m[0]->solid = true;
			m[1]->solid = false;
			m[0]->aff = aff;
			m[1]->aff = aff;
			imat = im;
			ibone.Clear();
			for(int i = 0; i < 3; i++)for(int j = 0; j < 2; j++)
				ib[i][j] = ibone.Find(v[i]->bone[j]);
		}

		for(int k = 0; k < 2; k++){
			m[k]->Begin(Mesh::Triangles);
			for(int i = 0; i < 3; i++){
				m[k]->Normal    (v[i]->normal);
				m[k]->TexCoord  (v[i]->uv    );
				m[k]->BoneWeight(v[i]->weight);
				m[k]->BoneIndex (ib[i][0], ib[i][1]);
				m[k]->Vertex    (v[i]->pos   );
			}
			m[k]->End();
		}
	}
	model->materialList   .push_back(imat );
	model->bonePaletteList.push_back(ibone);
	
}

}
