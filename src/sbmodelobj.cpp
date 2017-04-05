#include <sbmodelobj.h>
#include <sbmessage.h>
#include <sbtokenizer.h>

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

LoaderOBJ::LoaderOBJ(){
	Clear();
}

void LoaderOBJ::Clear(){
}

void LoaderOBJ::Parse(){
	Clear();
	string_iterator_pair str;
	
	while(!rowTok.IsEnd()){
		colTok.Set(rowTok.GetToken(), " \t", true);

		str = colTok.GetToken();

		// 行頭の空トークンを読み飛ばす
		while(str.empty() && !colTok.IsEnd()){
			colTok.Next();
			str = colTok.GetToken();
		}

		if(str == "v"){
			vertices.push_back(Vec3f());
			Vec3f& v = vertices.back();

			for(int i = 0; i < 3; i++){
				colTok.Next();
				v[i] = (float)to_real(colTok.GetToken());
			}
		}
		if(str == "vt"){
			texCoords.push_back(Vec2f());
			Vec2f& vt = texCoords.back();

			for(int i = 0; i < 2; i++){
				colTok.Next();
				vt[i] = (float)to_real(colTok.GetToken());
			}
		}
		if(str == "vn"){
			normals.push_back(Vec3f());
			Vec3f& vn = normals.back();

			for(int i = 0; i < 3; i++){
				colTok.Next();
				vn[i] = (float)to_real(colTok.GetToken());
			}
		}
		if(str == "f"){
			faces.push_back(Face());
			Face& f = faces.back();

			// OBJの仕様では4点以上もあり得るが三角形のみを想定
			for(int i = 0; i < 3; i++){
				colTok.Next();
				str = colTok.GetToken();
				// '/'で分解
				Tokenizer tok(str, "/", false);
				if(!tok.IsEnd()){
					f.v[i] = to_int(tok.GetToken());
					tok.Next();
				}
				if(!tok.IsEnd()){
					f.t[i] = to_int(tok.GetToken());
					tok.Next();
				}
				if(!tok.IsEnd()){
					f.n[i] = to_int(tok.GetToken());
					tok.Next();
				}
			}
		}
		
		rowTok.Next();
	}
}

void LoaderOBJ::Load(const string& filename){
	ifstream ifs;
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

void LoaderOBJ::Convert(Model* model, const Affinef& aff){

}

}
