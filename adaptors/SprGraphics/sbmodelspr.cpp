#include "sbmodelspr.h"
#include <sbmessage.h>

namespace Scenebuilder{;

ModelSPR::ModelSPR(){

}

void ModelSPR::Load(string filename){
	/*
	// ロードするサブシーングラフのオーナとなるフレームを作成
	// ダミーフレームの下に作るのでこれ自体は描画されない
	GRFrameIf* meshFrame = grScene->CreateVisual(GRFrameDesc(), grDummyFrame)->Cast();
			
	ObjectIfs objs;
	objs.Push(grScene);
	objs.Push(meshFrame);

	FIFileIf* file = fiSdk->CreateFileFromExt((const char*)meshProp->filename);
	if(!file)
		file = fiSdk->CreateFileX();
	ok = file->Load(objs, path.c_str());

	grDummyFrame->DelChildObject(meshFrame);
	*/
}

void ModelSPR::Save(string filename){

}

void ModelSPR::Convert(vector<Mesh>& mesh, vector<int>& ml, bool solid, const Affinef& aff){

}

}
