#include <sbmesh.h>
#include <sbmessage.h>
#include <sbscene.h>

#include <GL/glew.h>

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

Mesh::Mesh(){
	front   = true;
	back    = false;
	type    = Type::Solid;
	tile    = false;

	curNormal     = Vec3f(0.0f, 0.0f, 1.0f);
	curColor      = Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
	curColor2     = Vec4f(0.5f, 0.5f, 0.5f, 0.5f);
	colorSet      = false;
	curTexCoord   = Vec2f(0.0f, 0.0f);
	curBoneIndex  = Vec2i(-1, -1);
	curBoneWeight = 1.0f;

	texture_scale  = Vec2f(1.0f, 1.0f);
	texture_offset = Vec2f(0.0f, 0.0f); 
}

void Mesh::Clear(){
	positions  .clear();
	normals    .clear();
	colors     .clear();
	texcoords  .clear();
	boneIndices.clear();
	boneWeights.clear();

	vtx.clear();
}

void Mesh::Begin(int _prim){
	vtx.clear();
	curPrimitive = _prim;
}

void Mesh::End(){
	CreatePrimitive();
	vtx.clear();
}

void Mesh::CreatePoint(Mesh::Vtx& v){
	positions.push_back(v.pos   );
	normals  .push_back(v.normal);
	colors   .push_back(v.color );
}

void Mesh::CreateTriangle(Mesh::Vtx& v0, Mesh::Vtx& v1, Mesh::Vtx& v2, bool _2nd){
	Vtx* _v[3] = {&v0, &v1, &v2};
	
	if(type == Type::Solid){
		// front and back faces
		for(int i = 0; i < 2; i++){
			if((i == 0 && front) || (i == 1 && back)){
				for(int j = 0; j < 3; j++){
					positions.push_back(_v[j]->pos);
					// revert normal vector for back face
					normals  .push_back((i == 0 ? 1.0f : -1.0f) * _v[j]->normal);
					Vec4f c = (_2nd ? _v[j]->color2 : _v[j]->color);
					colors     .push_back(c);
					texcoords  .push_back(_v[j]->texcoord  );
					boneIndices.push_back(_v[j]->boneIndex );
					boneWeights.push_back(_v[j]->boneWeight);
				}
			}
			swap(_v[1], _v[2]);
		}
	}
	if(type == Type::Wireframe){
		for(int j = 0; j < 3; j++){
			positions  .push_back(_v[j]->pos);
			positions  .push_back(_v[(j+1)%3]->pos);
			colors     .push_back(_v[j]->color);
			colors     .push_back(_v[(j+1)%3]->color);
			boneIndices.push_back(_v[j]->boneIndex );
			boneWeights.push_back(_v[j]->boneWeight);
		}
	}
}

void Mesh::CreatePrimitive(){
	static uint flip = 0;
	flip = !flip;

	if(curPrimitive == Points){
		for(uint i = 0; i < vtx.size(); i++)
			CreatePoint(vtx[i]);
	}
	if(curPrimitive == Triangles){
		for(uint i = 0; i < vtx.size(); i+=3)
			CreateTriangle(vtx[i+0], vtx[i+1], vtx[i+2], (tile && ((i/3)%2) == flip));
	}
	if(curPrimitive == TriangleStrip){
		for(uint i = 0; i+2 < vtx.size(); i+=2){
			CreateTriangle(vtx[i+1], vtx[i+0], vtx[i+2], (tile && ((i/2)%2) == flip));
			CreateTriangle(vtx[i+1], vtx[i+2], vtx[i+3], (tile && ((i/2)%2) == flip));
		}
	}
	if(curPrimitive == TriangleFan){
		for(uint i = 1; i+1 < vtx.size(); i++)
			CreateTriangle(vtx[0], vtx[i], vtx[i+1], (tile && (i%2) == flip));
	}
}

void Mesh::Vertex(float x, float y, float z){
	Vertex(Vec3f(x,y,z));
}
void Mesh::Vertex(const Vec3f& _v){
	Vtx v;
	v.pos        = aff * _v;
	v.normal     = curNormal;
	v.texcoord   = curTexCoord;
	v.color      = curColor;
	v.color2     = curColor2;
	v.boneIndex  = curBoneIndex;
	v.boneWeight = curBoneWeight;
	vtx.push_back(v);
}

void Mesh::Normal(float x, float y, float z){
	Normal(Vec3f(x, y, z));
}
void Mesh::Normal(const Vec3f& n){
	curNormal = aff.Rot() * n;
}

void Mesh::TexCoord(float x, float y){
	TexCoord(Vec2f(x, y));
}
void Mesh::TexCoord(const Vec2f& t){
	curTexCoord.x = texture_scale.x * t.x + texture_offset.x;
	curTexCoord.y = texture_scale.y * t.y + texture_offset.y;
}

void Mesh::Color(float r, float g, float b, float a){
	Color(Vec4f(r, g, b, a));
}
void Mesh::Color(const Vec4f& c){
	curColor = c;
	colorSet = true;
}

void Mesh::Color2(float r, float g, float b, float a){
	Color2(Vec4f(r, g, b, a));
}
void Mesh::Color2(const Vec4f& c){
	curColor2 = c;
	colorSet  = true;
}

void Mesh::BoneIndex(int i0, int i1){
	curBoneIndex = Vec2i(i0, i1);
}
void Mesh::BoneWeight(float w){
	curBoneWeight = w;
}

void Mesh::GenerateNormals(){
	normals.resize(positions.size());
	for(uint i = 0; i < positions.size(); i+=3){
		Vec3f n = (positions[i+1] - positions[i+0]) % (positions[i+2] % positions[i+1]);
		normals[i+0] = n;
		normals[i+1] = n;
		normals[i+2] = n;
	}
}

void Mesh::GenerateTexCoords(int style){
	CalcBBox();
	if(normals.empty())
		GenerateNormals();

	texcoords.resize(positions.size());
	for(uint i = 0; i < positions.size(); i++)
		texcoords[i] = MapTexCoord(positions[i], normals[i], style);
}

Vec2f Mesh::MapTexCoord(const Vec3f& p, const Vec3f& n, int style){
	Vec2f uv;
	
	if(style == ShapeProp::Box){
		// pから法線n方向に延長して交差するbboxの面を求める
		float inf = numeric_limits<float>::max();
		float smin = inf;
		int   idx = -1;
		bool  dir;
		for(int k = 0; k < 3; k++){
			if(n[k] == 0.0f)
				continue;
			float s0 = (bbmin[k] - p[k])/n[k];
			float s1 = (bbmax[k] - p[k])/n[k];
			float s;
			bool  d;
			if(s1 > s0){
				s = s1;
				d = true;
			}
			else{
				s = s0;
				d = false;
			}
			if(s < smin){
				smin = s;
				idx  = k;
				dir  = d;
			}
		}

		if(idx == -1)
			return Vec2f();

		float w = bbmax[0] - bbmin[0];
		float h = bbmax[1] - bbmin[1];
		float d = bbmax[2] - bbmin[2];

		// bboxをテクスチャ座標に展開したときのu,vを求める
		Vec3f pb = p + smin * n;
		if(idx == 0){
			if(dir)
				 uv = Vec2f(w+d + pb.z, d + pb.y);
			else uv = Vec2f(  d - pb.z, d + pb.y);
		}
		if(idx == 1){
			if(dir)
				 uv = Vec2f(d + pb.x, h+d + pb.z);
			else uv = Vec2f(d + pb.x,   d - pb.z);
		}
		if(idx == 2){
			if(dir)
				 uv = Vec2f(     d  + pb.x, d + pb.y);
			else uv = Vec2f(2*(w+d) - pb.x, d + pb.y);
		}
	}
	if(style == ShapeProp::Sphere){
		uv[0] = atan2(p.y, p.x) / (float)M_PI;
		uv[1] = atan2(p.z, sqrtf(p.x*p.x + p.y*p.y)) / (float)M_PI;
	}

	uv.x = texture_scale.x * uv.x + texture_offset.x;
	uv.y = texture_scale.y * uv.y + texture_offset.y;
	
	return uv;
}

void Mesh::CalcBBox(){
	float inf = numeric_limits<float>::max();
	for(uint k = 0; k < 3; k++){
		bbmin[k] =  inf;
		bbmax[k] = -inf;
	}
	for(uint i = 0; i < positions.size(); i++){
		for(uint k = 0; k < 3; k++){
			bbmin[k] = std::min(bbmin[k], positions[i][k]);
			bbmax[k] = std::max(bbmax[k], positions[i][k]);
		}
	}
}

void Mesh::Box(float sx, float sy, float sz, uint divx, uint divy, uint divz){
	float dx = sx / (float)divx;
	float dy = sy / (float)divy;
	float dz = sz / (float)divz;
	float rx = 0.5f * sx;
	float ry = 0.5f * sy;
	float rz = 0.5f * sz;
	float x, y, z;
	uint i, j;

	Normal(0.0f, 0.0f, -1.0f);
	for(j = 0, y = ry; j < divy; j++, y -= dy){
		Begin(TriangleStrip);
		for(i = 0, x = -rx; i <= divx; i++, x += dx){
			TexCoord(2*(sx+sz) - x, sz + y   ); Vertex(x, y   , -rz);
			TexCoord(2*(sx+sz) - x, sz + y-dy); Vertex(x, y-dy, -rz);
		}
		End();
	}

	Normal(0.0f, 0.0f, 1.0f);
	for(j = 0, y = -ry; j < divy; j++, y += dy){
		Begin(TriangleStrip);
		for(i = 0, x = -rx; i <= divx; i++, x += dx){
			TexCoord(sz + x, sz + y   ); Vertex(x, y   , rz);
			TexCoord(sz + x, sz + y+dy); Vertex(x, y+dy, rz);
		}
		End();
	}

	Normal(0.0f, -1.0f, 0.0f);
	for(j = 0, x = rx; j < divx; j++, x -= dx){
		Begin(TriangleStrip);
		for(i = 0, z = -rz; i <= divz; i++, z += dz){
			TexCoord(sz + x   , sz - z); Vertex(x   , -ry, z);
			TexCoord(sz + x-dx, sz - z); Vertex(x-dx, -ry, z);
		}
		End();
	}
	
	Normal(0.0f, 1.0f, 0.0f);
	for(j = 0, x = -rx; j < divx; j++, x += dx){
		Begin(TriangleStrip);
		for(i = 0, z = -rz; i <= divz; i++, z += dz){
			TexCoord(sz + x   , sy+sz + z); Vertex(x   , ry, z);
			TexCoord(sz + x+dx, sy+sz + z); Vertex(x+dx, ry, z);
		}
		End();
	}
	
	Normal(-1.0f, 0.0f, 0.0f);
	for(j = 0, z = rz; j < divz; j++, z -= dz){
		Begin(TriangleStrip);
		for(i = 0, y = -ry; i <= divy; i++, y += dy){
			TexCoord(sz -  z    , sz + y); Vertex(-rx, y, z   );
			TexCoord(sz - (z-dz), sz + y); Vertex(-rx, y, z-dz);
		}
		End();
	}
	
	Normal(1.0f, 0.0f, 0.0f);
	for(j = 0, z = -rz; j < divz; j++, z += dz){
		Begin(TriangleStrip);
		for(i = 0, y = -ry; i <= divy; i++, y += dy){
			TexCoord(sx+sz + z   , sz + y); Vertex(rx, y, z   );
			TexCoord(sx+sz + z+dz, sz + y); Vertex(rx, y, z+dz);
		}
		End();
	}
}

void Mesh::CreateCircle(vector<Vec2f>& points, uint slice){
	points.resize(slice+1);
	float theta = 0.0f;
	float dtheta = (float)(2.0 * M_PI)/(float)slice;
	for(uint s = 0; s <= slice; s++){
		points[s].x = cosf(theta);
		points[s].y = sinf(theta);
		theta += dtheta;
	}
}

void Mesh::Sphere(float radius, uint slice, uint stack, bool upper, bool lower, float offset){
	vector<Vec2f> circle;
	CreateCircle(circle, slice);
	
	int i = 0;
	float dz = radius/(float)stack;
	float rinv = 1.0f / radius;
	float rsqr = radius*radius;
	float z0, z1, r0, r1;
	Vec3f v;
		
	for(int t = -(int)stack; t < (int)stack; t++){
		if(!lower && t <  0) continue;
		if(!upper && t >= 0) continue;
		z0 = dz*(t+0); r0 = sqrtf(rsqr - z0*z0);
		z1 = dz*(t+1); r1 = sqrtf(rsqr - z1*z1);

		if(t == -(int)stack){
			Begin(Triangles);
			for(int s = (int)slice; s > 0; s--){
				Normal(0.0f, 0.0f, -1.0f); TexCoord((float)(s) / (float)slice, 0.0f); Vertex(0.0f, 0.0f, -radius - offset);

				v = Vec3f(r1 * circle[s-0].x, r1 * circle[s-0].y, z1 - offset); Normal(rinv * v); TexCoord((float)(s-0) / (float)slice, ((float)(t+1) / (float)stack + 1.0f) / 4.0f); Vertex(v);
				v = Vec3f(r1 * circle[s-1].x, r1 * circle[s-1].y, z1 - offset); Normal(rinv * v); TexCoord((float)(s-1) / (float)slice, ((float)(t+1) / (float)stack + 1.0f) / 4.0f); Vertex(v);
			}
			End();
		}
		else if(t == stack-1){
			Begin(Triangles);
			for(uint s = 0; s < slice; s++){
				Normal(0.0f, 0.0f, 1.0f); TexCoord((float)(s) / (float)slice, 0.5f); Vertex(0.0f, 0.0f, radius + offset);
				
				v = Vec3f(r0 * circle[s+0].x, r0 * circle[s+0].y, z0 + offset); Normal(rinv * v); TexCoord((float)(s+0) / (float)slice, ((float)t / (float)stack + 1.0f) / 4.0f); Vertex(v);
				v = Vec3f(r0 * circle[s+1].x, r0 * circle[s+1].y, z0 + offset); Normal(rinv * v); TexCoord((float)(s+1) / (float)slice, ((float)t / (float)stack + 1.0f) / 4.0f); Vertex(v);
			}
			End();
		}
		else{
			float o = (t < 0 ? -1.0f : 1.0f) * offset;
			Begin(TriangleStrip);
			for(uint s = 0; s <= slice; s++){
				v = Vec3f(r0 * circle[s].x, r0 * circle[s].y, z0 + o); Normal(rinv * v); TexCoord((float)s / (float)slice, ((float)(t+0) / (float)stack + 1.0f) / 4.0f); Vertex(v);
				v = Vec3f(r1 * circle[s].x, r1 * circle[s].y, z1 + o); Normal(rinv * v); TexCoord((float)s / (float)slice, ((float)(t+1) / (float)stack + 1.0f) / 4.0f); Vertex(v);
			}
			End();
		}
	}
}

void Mesh::Cylinder(float radius, float length, uint slice, bool cap){
	vector<Vec2f> circle;
	CreateCircle(circle, slice);
	
	float rinv = 1.0f / radius;
	float z = 0.5f * length;
	float L = 2.0f * (float)M_PI * radius;
	Vec3f v;
	Begin(TriangleStrip);
	for(uint s = 0; s <= slice; s++){
		v = Vec3f(radius * circle[s].x, radius * circle[s].y, 0.0f);
		Normal(rinv * v);
		TexCoord(L * (float)s / (float)slice, 2*radius);
		Vertex(v.x, v.y, -z);
		TexCoord(L * (float)s / (float)slice, 2*radius+length);
		Vertex(v.x, v.y,  z);
	}
	End();
	
	if(cap){
		Begin(TriangleFan);
		Normal(0.0f, 0.0f, 1.0f);
		TexCoord(radius, 3.0f*radius+length);
		Vertex(0.0f, 0.0f, z);
		for(uint s = 0; s <= slice; s++){
			TexCoord(radius + radius*circle[s].x, 3.0f*radius+length + radius*circle[s].y);
			Vertex(radius * circle[s].x, radius * circle[s].y,  z);
		}
		End();

		Begin(TriangleFan);
		Normal(0.0f, 0.0f, -1.0f);
		TexCoord(radius, radius);
		Vertex(0.0f, 0.0f, -z);
		for(int s = slice; s >= 0; s--){
			TexCoord(radius + radius*circle[s].x, radius - radius*circle[s].y);
			Vertex(radius * circle[s].x, radius * circle[s].y, -z);
		}
		End();
	}
}

void Mesh::Capsule(float radius, float length, uint slice, uint stack){
	Sphere  (radius, slice, stack, true, true, length/2.0f);
	Cylinder(radius, length, slice, false);
}

//-------------------------------------------------------------------------------------------------

VertexArray::VertexArray(){
	mesh        = 0;
	usePos      = false;
	useNormal   = false;
	useColor    = false;
	useTexcoord = false;
	useBone     = false;
	useVao      = false;
	vaoId       = 0;
	for(int i = 0; i < 6; i++)
		vboId[i] = 0;

	pointSize = 1.0f;
	lineWidth = 1.0f;
}

VertexArray::~VertexArray(){
	Delete();
}

void VertexArray::Delete(){
	for(int i = 0; i < 6; i++){
		if(vboId[i])
			glDeleteBuffers(1, &vboId[i]);
	}
	if(useVao && vaoId)
		glDeleteVertexArrays(1, &vaoId);
}

void VertexArray::Create(uint _count, bool p, bool n, bool c, bool t, bool b, bool vao, bool dynamic){
	Delete();

	usePos      = p;
	useNormal   = n;
	useColor    = c;
	useTexcoord = t;
	useBone     = b;
	useVao      = vao;
	count       = _count;

	int usage = (dynamic ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
	
	glGenBuffers(6, vboId);
	
	if(useVao){
		glGenVertexArrays(1, &vaoId);
		glBindVertexArray(vaoId);
	}

	if(usePos){
		glEnableClientState(GL_VERTEX_ARRAY);
		glBindBuffer(GL_ARRAY_BUFFER, vboId[0]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3f)*count, 0, usage);
		glVertexPointer(3, GL_FLOAT, 0, 0);
	}
	else{
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glVertexPointer(3, GL_FLOAT, 0, 0);
	}

	if(useNormal){
		glEnableClientState(GL_NORMAL_ARRAY);
		glBindBuffer(GL_ARRAY_BUFFER, vboId[1]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3f)*count, 0, usage);
		glNormalPointer(GL_FLOAT, 0, 0);
	}
	else{
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glNormalPointer(GL_FLOAT, 0, 0);
	}

	if(useColor){
		glEnableClientState(GL_COLOR_ARRAY);
		glBindBuffer(GL_ARRAY_BUFFER, vboId[2]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Vec4f)*count, 0, usage);
		glColorPointer(4, GL_FLOAT, 0, 0);
	}
	else{
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glColorPointer(4, GL_FLOAT, 0, 0);
	}

	if(useTexcoord){
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		glBindBuffer(GL_ARRAY_BUFFER, vboId[3]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Vec2f)*count, 0, usage);
		glTexCoordPointer(2, GL_FLOAT, 0, 0);
	}
	else{
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glTexCoordPointer(2, GL_FLOAT, 0, 0);
	}

	// int型のvertex attributeを渡すためのglVertexAttribIPointerはOpenGL3.0+なので
	// スキニングは3.0+でのみサポート
	if(useVao && useBone){
		glBindBuffer(GL_ARRAY_BUFFER, vboId[4]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Vec2i)*count, 0, usage);
		glVertexAttribIPointer   (Attribute::BlendIndex, 2, GL_INT, 0, 0);
		glEnableVertexAttribArray(Attribute::BlendIndex);

		glBindBuffer(GL_ARRAY_BUFFER, vboId[5]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float)*count, 0, usage);
		glVertexAttribPointer    (Attribute::BlendWeight, 1, GL_FLOAT, false, 0, 0);
		glEnableVertexAttribArray(Attribute::BlendWeight);
	}

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	if(useVao)
		glBindVertexArray(0);

}

void VertexArray::Create(Mesh* _mesh, bool vao, bool dynamic){
	mesh   = _mesh;
	useVao = vao;
	count  = (uint)mesh->positions.size();
	
	if(count == 0)
		return;

	Create(count,
		mesh->positions  .size() == count,
		mesh->normals    .size() == count,
		mesh->colors     .size() == count && mesh->colorSet,
		mesh->texcoords  .size() == count,
		mesh->boneIndices.size() == count && mesh->boneIndices.size() == count && useVao,
		useVao,
		dynamic);

	if(usePos){
		Vec3f* ptr = MapPositionArray();
		copy(mesh->positions.begin(), mesh->positions.end(), ptr);
		UnmapPositionArray();
	}

	if(useNormal){
		Vec3f* ptr = MapNormalArray();
		copy(mesh->normals.begin(), mesh->normals.end(), ptr);
		UnmapNormalArray();
	}

	if(useColor){
		Vec4f* ptr = MapColorArray();
		copy(mesh->colors.begin(), mesh->colors.end(), ptr);
		UnmapColorArray();
	}

	if(useTexcoord){
		Vec2f* ptr = MapTexCoordArray();
		copy(mesh->texcoords.begin(), mesh->texcoords.end(), ptr);
		UnmapTexCoordArray();
	}

	if(useBone){
		Vec2i* ptri = MapBoneIndexArray();
		copy(mesh->boneIndices.begin(), mesh->boneIndices.end(), ptri);
		UnmapBoneIndexArray();

		float* ptrw = MapBoneWeightArray();
		copy(mesh->boneWeights.begin(), mesh->boneWeights.end(), ptrw);
		UnmapBoneWeightArray();
	}
}

Vec3f* VertexArray::MapPositionArray(){
	if(!usePos)
		return 0;
	glBindBuffer(GL_ARRAY_BUFFER, vboId[0]);
	Vec3f* p = (Vec3f*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_WRITE);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	return p;
}

Vec3f* VertexArray::MapNormalArray(){
	if(!useNormal)
		return 0;
	glBindBuffer(GL_ARRAY_BUFFER, vboId[1]);
	Vec3f* p = (Vec3f*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_WRITE);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	return p;
}

Vec4f* VertexArray::MapColorArray(){
	if(!useColor)
		return 0;
	glBindBuffer(GL_ARRAY_BUFFER, vboId[2]);
	Vec4f* p = (Vec4f*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_WRITE);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	return p;
}

Vec2f* VertexArray::MapTexCoordArray(){
	if(!useTexcoord)
		return 0;
	glBindBuffer(GL_ARRAY_BUFFER, vboId[3]);
	Vec2f* p = (Vec2f*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_WRITE);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	return p;
}

Vec2i* VertexArray::MapBoneIndexArray(){
	if(!useBone)
		return 0;
	glBindBuffer(GL_ARRAY_BUFFER, vboId[4]);
	Vec2i* p = (Vec2i*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_WRITE);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	return p;
}

float* VertexArray::MapBoneWeightArray(){
	if(!useBone)
		return 0;
	glBindBuffer(GL_ARRAY_BUFFER, vboId[5]);
	float* p = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_WRITE);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	return p;
}

void VertexArray::UnmapPositionArray(){
	if(!usePos)
		return;
	glBindBuffer (GL_ARRAY_BUFFER, vboId[0]);
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer (GL_ARRAY_BUFFER, 0);
}

void VertexArray::UnmapNormalArray(){
	if(!useNormal)
		return;
	glBindBuffer (GL_ARRAY_BUFFER, vboId[1]);
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer (GL_ARRAY_BUFFER, 0);
}

void VertexArray::UnmapColorArray(){
	if(!useColor)
		return;
	glBindBuffer (GL_ARRAY_BUFFER, vboId[2]);
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer (GL_ARRAY_BUFFER, 0);
}

void VertexArray::UnmapTexCoordArray(){
	if(!useTexcoord)
		return;
	glBindBuffer (GL_ARRAY_BUFFER, vboId[3]);
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer (GL_ARRAY_BUFFER, 0);
}

void VertexArray::UnmapBoneIndexArray(){
	if(!useBone)
		return;
	glBindBuffer (GL_ARRAY_BUFFER, vboId[4]);
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer (GL_ARRAY_BUFFER, 0);
}

void VertexArray::UnmapBoneWeightArray(){
	if(!useBone)
		return;
	glBindBuffer (GL_ARRAY_BUFFER, vboId[5]);
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer (GL_ARRAY_BUFFER, 0);
}

void VertexArray::Draw(int prim){
	if(useVao){
		glBindVertexArray(vaoId);
	}
	else{
		if(usePos){
			glEnableClientState(GL_VERTEX_ARRAY);
			glBindBuffer(GL_ARRAY_BUFFER, vboId[0]);
			glVertexPointer(3, GL_FLOAT, 0, 0);
		}
		else glDisableClientState(GL_VERTEX_ARRAY);

		if(useNormal){
			glEnableClientState(GL_NORMAL_ARRAY);
			glBindBuffer(GL_ARRAY_BUFFER, vboId[1]);
			glNormalPointer(GL_FLOAT, 0, 0);
		}
		else glDisableClientState(GL_NORMAL_ARRAY);

		if(useColor){
			glEnableClientState(GL_COLOR_ARRAY);
			glBindBuffer(GL_ARRAY_BUFFER, vboId[2]);
			glColorPointer(4, GL_FLOAT, 0, 0);
		}
		else glDisableClientState(GL_COLOR_ARRAY);

		if(useTexcoord){
			glEnableClientState(GL_TEXTURE_COORD_ARRAY);
			glBindBuffer(GL_ARRAY_BUFFER, vboId[3]);
			glTexCoordPointer(2, GL_FLOAT, 0, 0);
		}
		else glDisableClientState(GL_TEXTURE_COORD_ARRAY);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	int t = GL_TRIANGLES;
	switch(prim){
	case Primitive::Points:
		t = GL_POINTS;
		glPointSize(pointSize);
		break;
	case Primitive::Lines:
		t = GL_LINES;
		glLineWidth(lineWidth);
		break;
	case Primitive::Triangles:
		t = GL_TRIANGLES;
		break;
	}
	glDrawArrays(t, 0, count);

	glPointSize(1.0f);
	glLineWidth(1.0f);
	
	if(useVao){
		glBindVertexArray(0);
	}
	else{

	}
}

}
