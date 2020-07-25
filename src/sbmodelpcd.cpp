#include <sbmodelPCD.h>

#include <boost/cstdint.hpp>
using namespace boost;

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

LoaderPCD::LoaderPCD(){
}

bool LoaderPCD::ParseBinary(){
	int offset = -1;
	int num    =  0;

	while(!rowTok.IsEnd()){
		colTok.Set(rowTok.GetToken(), " \t", true);

		string_iterator_pair str;
		str = colTok.GetToken();

		if(str == "VERSION"  );
		if(str == "FIELDS"   );
		if(str == "SIZE"     );
		if(str == "TYPE"     );
		if(str == "COUNT"    );
		if(str == "WIDTH"    );
		if(str == "HEIGHT"   );
		if(str == "VIEWPOINT");
		if(str == "POINTS"   ){
			colTok.Next();
			Converter::FromString(colTok.GetToken(), num);
		}
		if(str == "DATA"     ){
			// DATA row comes last in the header, so the following are the main data
			offset = (&*(rowTok.GetToken().second) + 1) - &contents[0];
			break;
		}

		rowTok.Next();
	}

	if(offset == -1){
		return false;
	}
	if(contents.size() - offset != num*sizeof(Point)){
		return false;
	}

	points.resize(num);
	copy((Point*)&contents[offset], (Point*)&contents[offset + num*sizeof(Point)], &points[0]);

	return true;
}

void LoaderPCD::Load(const string& filename){
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
	
	ParseBinary();
	contents.clear();
}

void LoaderPCD::Convert(Model* model, const Affinef& aff){
	model->meshPoints.resize(1);
	model->meshPoints[0].type = Mesh::Type::Points;
	model->meshPoints[0].aff  = aff;
	
	Mesh* m = &model->meshPoints[0];
	m->Begin(Mesh::Points);
	for(Point& pt : points){
		m->Normal(pt.normal);
		m->Vertex(pt.pos   );
	}
	m->End();
}

}
