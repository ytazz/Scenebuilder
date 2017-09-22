#include <sbmodelstl.h>

#include <boost/cstdint.hpp>
using namespace boost;

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

LoaderSTL::LoaderSTL(bool _useColor, bool _binary){
	useColor  = _useColor;
	binary    = _binary;
	facetOpen = false;
	vertexIdx = 0;
	Clear();
}

void LoaderSTL::Clear(){
	facets.clear();
}

void LoaderSTL::ParseBinary(){
	byte* ptr = (byte*)&contents[0];
	ptr += 80;
	
	int numTri = *(int*)ptr;
	ptr += sizeof(int);

	int szTri = 4 * (sizeof(float) * 3) + sizeof(uint16_t);
	Assert(80 + sizeof(int) + szTri * numTri <= contents.size());

	facets.resize(numTri);
	for(int i = 0; i < numTri; i++){
		Facet& f = facets[i];
		f.normal      = *(Vec3f   *)ptr; ptr += sizeof(Vec3f   );
		f.vertices[0] = *(Vec3f   *)ptr; ptr += sizeof(Vec3f   );
		f.vertices[1] = *(Vec3f   *)ptr; ptr += sizeof(Vec3f   );
		f.vertices[2] = *(Vec3f   *)ptr; ptr += sizeof(Vec3f   );
		f.attr        = *(uint16_t*)ptr; ptr += sizeof(uint16_t);
		if(useColor){
			float r = (float)((f.attr >>  0) & 0x1f) / 31.0f;
			float g = (float)((f.attr >>  5) & 0x1f) / 31.0f;
			float b = (float)((f.attr >> 10) & 0x1f) / 31.0f;
			f.color = Vec4f(r, g, b, 1.0f);
		}
	}
}

void LoaderSTL::Assert(bool eval){
	if(!eval)
		throw SyntaxError();
}

void LoaderSTL::Parse(){
	if(rowTok.IsEnd())
		return;
	colTok.Set(rowTok.GetToken(), " \t", true);

	Clear();
	string_iterator_pair str;
	
	// テキストSTLは一行目が"solid name". ただしnameはオプショナル
	// solidで始まらない場合はバイナリとして読む
	/*
	str = colTok.GetToken();	
	if(str != "solid"){
		ParseBinary();
		return;
	}
	*/
	// バイナリSTLでも80byteヘッダが"solid"で始まる場合があるので
	// テキストかバイナリかは手動で指定する
	if(binary){
		ParseBinary();
		return;
	}
		
	colTok.Next();
	if(!colTok.IsEnd()){
		str = colTok.GetToken();
		name.assign(str.begin(), str.end());
	}		

	rowTok.Next();
	
	while(!rowTok.IsEnd()){
		colTok.Set(rowTok.GetToken(), " \t", true);

		// 行頭の空トークンを読み飛ばす
		str = colTok.GetToken();
		while(str.empty() && !colTok.IsEnd()){
			colTok.Next();
			str = colTok.GetToken();
		}

		if(str == "facet"){
			Assert(!facetOpen);
			colTok.Next();
			Assert(colTok.GetToken() == "normal");
			
			facets.push_back(LoaderSTL::Facet());
			LoaderSTL::Facet& f = facets.back();
			f.attr  = 0;
			f.color = Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
			for(int i = 0; i < 3; i++){
				colTok.Next();
				f.normal[i] = (float)to_real(colTok.GetToken());
			}
			facetOpen = true;
		}
		else if(str == "outer"){
			Assert(facetOpen && vertexIdx == 0);
			colTok.Next();
			Assert(colTok.GetToken() == "loop");
			vertexIdx = 0;
		}
		else if(str == "vertex"){
			Assert(facetOpen);
			Assert(0 <= vertexIdx && vertexIdx < 3);
			LoaderSTL::Facet& f = facets.back();
			for(int i = 0; i < 3; i++){
				colTok.Next();
				f.vertices[vertexIdx][i] = (float)to_real(colTok.GetToken());
			}
			vertexIdx++;
		}
		else if(str == "endloop"){
			Assert(vertexIdx == 3);
			vertexIdx = 0;
		}
		else if(str == "endfacet"){
			Assert(facetOpen);
			facetOpen = false;
		}
		else if(str == "endsolid"){

		}
		
		rowTok.Next();
	}
}

void LoaderSTL::Load(const string& filename){
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
	
	// tokenize
	rowTok.Set(contents, "\n", true);
	
	Parse();
	contents.clear();
}

void LoaderSTL::Save(const string& filename){

}

void LoaderSTL::Convert(Model* model, const Affinef& aff){
	// facetのcolorからマテリアルを作成
	vector<uint16_t> attrs;
	vector<Vec4f>    colors;
	uint nmat = 1;
	
	if(useColor){
		for(uint i = 0; i < facets.size(); i++){
			Facet& f = facets[i];
			if(find(attrs.begin(), attrs.end(), f.attr) == attrs.end()){
				attrs .push_back(f.attr );
				colors.push_back(f.color);
			}
		}

		nmat = (uint)attrs.size();
		model->materials   .resize(nmat);
		model->materialList.resize(nmat);
		for(uint i = 0; i < nmat; i++){
			model->materials[i].diffuse = colors[i];
			model->materialList[i] = i;
		}
	}
	
	model->meshSolid.resize(nmat);
	model->meshWire .resize(nmat);
	for(uint i = 0; i < nmat; i++){
		model->meshSolid[i].solid = true;
		model->meshWire [i].solid = false;
		model->meshSolid[i].aff   = aff;
		model->meshWire [i].aff   = aff;
	}

	for(uint i = 0; i < nmat; i++){
		Mesh* m[2] = {&model->meshSolid[i], &model->meshWire[i]};
		for(int k = 0; k < 2; k++){
			m[k]->Begin(Mesh::Triangles);
			for(uint j = 0; j < facets.size(); j++){
				Facet& f = facets[j];
				if(useColor && f.attr != attrs[i])
					continue;
				m[k]->Normal(f.normal     );
				m[k]->Vertex(f.vertices[0]);
				m[k]->Vertex(f.vertices[1]);
				m[k]->Vertex(f.vertices[2]);
				if(useColor)
					m[k]->Color(f.color);
			}
			m[k]->End();
		}
	}
}

}
