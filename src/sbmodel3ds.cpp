#include <sbmodel3ds.h>
#include <sbmessage.h>

#include <boost/cstdint.hpp>
using namespace boost;

namespace Scenebuilder{;

Loader3DS::Loader3DS(){
	DefChunk(ChunkInfo::Main,              "main");
	DefChunk(ChunkInfo::Version,           "version",            ChunkInfo::Main);
	DefChunk(ChunkInfo::Editor,            "3d editor",          ChunkInfo::Main);
	DefChunk(ChunkInfo::MeshVersion,       "mesh version",       ChunkInfo::Editor);
	DefChunk(ChunkInfo::MaterialBlock,     "material block",     ChunkInfo::Editor);
	DefChunk(ChunkInfo::MaterialName,      "material name",      ChunkInfo::MaterialBlock);
	DefChunk(ChunkInfo::MaterialAmbient,   "material ambient",   ChunkInfo::MaterialBlock);
	DefChunk(ChunkInfo::MaterialDiffuse,   "material diffuse",   ChunkInfo::MaterialBlock);
	DefChunk(ChunkInfo::MaterialSpecular,  "material specular",  ChunkInfo::MaterialBlock);
	DefChunk(ChunkInfo::MaterialShininess, "material shininess", ChunkInfo::MaterialBlock);
	DefChunk(ChunkInfo::MaterialStrength,  "material strength",  ChunkInfo::MaterialBlock);
	DefChunk(ChunkInfo::RGBByte,           "rgb byte");
	DefChunk(ChunkInfo::ObjectBlock,       "object block",       ChunkInfo::Editor);
	DefChunk(ChunkInfo::TriangleMesh,      "triangle mesh",      ChunkInfo::ObjectBlock);
	DefChunk(ChunkInfo::VertexList,        "vertex list",        ChunkInfo::TriangleMesh);
	DefChunk(ChunkInfo::FaceDescription,   "face description",   ChunkInfo::TriangleMesh);
	DefChunk(ChunkInfo::FaceMaterialList,  "face material list", ChunkInfo::FaceDescription);
	Clear();
}

void Loader3DS::Clear(){
	materials.clear();
	objects.clear();
}

void Loader3DS::DefChunk(int id, string name, int par){
	chunkInfo.push_back(ChunkInfo());
	chunkInfo.back().id     = id;
	chunkInfo.back().name   = name;
	chunkInfo.back().parent = par;
}

Loader3DS::ChunkInfo* Loader3DS::GetChunkInfo(int id){
	for(uint i = 0; i < chunkInfo.size(); i++){
		ChunkInfo& info = chunkInfo[i];
		if(info.id == id)
			return &info;
	}
	return 0;
}

void Loader3DS::ReadChunks(){
	byte* ptr_begin = ptr;
	uint16_t id = Get<uint16_t>();
	uint32_t sz = Get<uint32_t>();
	byte* ptr_end = ptr_begin + sz;
	
	ChunkInfo* info = GetChunkInfo(id);
	ChunkInfo* parInfo = (chunkStack.empty() ? 0 : chunkStack.top());

	if(!info){
		Message::Out("unknown chunk %x", id);
	}
	else if(info->parent != -1 && parInfo && info->parent != parInfo->id){
		Message::Out("chunk %s cannot be child of %s", info->name.c_str(), parInfo->name.c_str());
	}
	else{
		if(info->id == ChunkInfo::Main){
			
		}
		if(info->id == ChunkInfo::Version){
			version = Get<uint32_t>();
		}
		if(info->id == ChunkInfo::Editor){

		}
		if(info->id == ChunkInfo::MeshVersion){
			uint32_t meshVersion = Get<uint32_t>();
		}
		if(info->id == ChunkInfo::MaterialBlock){
			materials.push_back(Material());
			curMat = &materials.back();
		}
		if(info->id == ChunkInfo::MaterialName){
			GetCString(curMat->name);
		}
		if(info->id == ChunkInfo::MaterialAmbient){
			curColor = &curMat->ambient;
		}
		if(info->id == ChunkInfo::MaterialDiffuse){
			curColor = &curMat->diffuse;
		}
		if(info->id == ChunkInfo::MaterialSpecular){
			curColor = &curMat->specular;
		}
		if(info->id == ChunkInfo::MaterialShininess){
			curFloat = &curMat->shininess;
		}
		if(info->id == ChunkInfo::MaterialStrength){
			curFloat = &curMat->strength;
		}
		if(info->id == ChunkInfo::RGBByte){
			if(!curColor){
				Message::Error("%s in wrong position", info->name.c_str());
			}
			else{
				(*curColor)[0] = (float)Get<byte>() / 255.0f;
				(*curColor)[1] = (float)Get<byte>() / 255.0f;
				(*curColor)[2] = (float)Get<byte>() / 255.0f;
			}
		}
		if(info->id == ChunkInfo::ObjectBlock){
			objects.push_back(new Object());
			curObj = objects.back();
			GetCString(curObj->name);
		}
		if(info->id == ChunkInfo::TriangleMesh){

		}
		if(info->id == ChunkInfo::VertexList){
			if(!curObj){
				Message::Error("%s in wrong position", info->name.c_str());
			}
			else{
				uint16_t num = Get<uint16_t>();
				curObj->vertices.resize(num);
				for(int i = 0; i < num; i++){
					curObj->vertices[i] = Get<Vec3f>();
				}
			}
		}
		if(info->id == ChunkInfo::FaceDescription){
			if(!curObj){
				Message::Error("%s in wrong position", info->name.c_str());
			}
			else{
				uint16_t num = Get<uint16_t>();
				curObj->faces.resize(num);
				for(int i = 0; i < num; i++){
					Face& f = curObj->faces[i];
					f.vertices[0] = Get<uint16_t>();
					f.vertices[1] = Get<uint16_t>();
					f.vertices[2] = Get<uint16_t>();
					f.flag = Get<uint16_t>();
				}
			}
		}
		if(info->id == ChunkInfo::FaceMaterialList){
			if(!curObj){
				Message::Error("%s in wrong position", info->name.c_str());
			}
			else{
				curObj->materialList.push_back(FaceMaterial());
				FaceMaterial& faceMat = curObj->materialList.back();

				GetCString(faceMat.name);
				uint16_t num = Get<uint16_t>();
				faceMat.faces.resize(num);
				for(int i = 0; i < num; i++){
					faceMat.faces[i] = Get<uint16_t>();
				}
			}
		}

		// read sub-chunks
		chunkStack.push(info);
		while(ptr != ptr_end){
			ReadChunks();
		}
		chunkStack.pop();
	}
	
	ptr = ptr_end;
}

void Loader3DS::ParseBinary(){
	ptr = (byte*)&contents[0];
	ptr_end = ptr + contents.size();

	while(!IsEnd())
		ReadChunks();
}

void Loader3DS::Load(const string& filename){
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

	// マテリアル名からマテリアルインデックスを取得
	for(uint i = 0; i < objects.size(); i++){
		auto& obj = objects[i];

		for(uint j = 0; j < obj->materialList.size(); j++){
			FaceMaterial& fmat = obj->materialList[j];

			uint imat;
			for(imat = 0; imat < materials.size(); imat++){
				if(fmat.name == materials[imat].name)
					break;
			}
			if(imat == materials.size())
				 fmat.index = 0;
			else fmat.index = imat;
		}
	}

	contents.clear();
}

void Loader3DS::Convert(Model* model, const Affinef& aff){
	for(uint i = 0; i < objects.size(); i++){
		auto& obj = objects[i];

		for(uint j = 0; j < obj->materialList.size(); j++){
			FaceMaterial& fmat = obj->materialList[j];

			model->materialList.push_back(fmat.index);
			model->meshSolid   .push_back(Mesh());
			model->meshWire    .push_back(Mesh());
			Mesh* m[2] = {&model->meshSolid.back(), &model->meshWire.back()};
			for(int k = 0; k < 2; k++){
				m[k]->solid = (k == 0);
				m[k]->aff   = aff;
				m[k]->Begin(Mesh::Triangles);
				for(uint f = 0; f < fmat.faces.size(); f++){
					Face& face = obj->faces[f];
					m[k]->Vertex(obj->vertices[face.vertices[0]]);
					m[k]->Vertex(obj->vertices[face.vertices[1]]);
					m[k]->Vertex(obj->vertices[face.vertices[2]]);
				}
				m[k]->End();
			}
		}
	}

	model->materials.resize(materials.size());
	for(uint i = 0; i < materials.size(); i++){
		Scenebuilder::Material& md = model->materials[i];
		Loader3DS   ::Material& ms =        materials[i];
		md.ambient  = Vec4f(ms.ambient [0], ms.ambient [1], ms.ambient [2], 1.0f);
		md.diffuse  = Vec4f(ms.diffuse [0], ms.diffuse [1], ms.diffuse [2], 1.0f);
		md.specular = Vec4f(ms.specular[0], ms.specular[1], ms.specular[2], 1.0f);
	}
}

}
