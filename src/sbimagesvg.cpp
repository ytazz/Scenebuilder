#include <sbimagesvg.h>
#include <sbtokenizer.h>

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

Vec2f SVG::Segment::GetPoint(float t)const{
	float tc = 1.0f - t;
	float tc2 = tc * tc;
	float tc3 = tc * tc2;
	float t2 = t * t;
	float t3 = t * t2;
	return (tc3) * p0 + (3.0f * tc2 * t) * c0 + (3.0f * tc * t2) * c1 + (t3) * p1;
}

Vec2f SVG::Segment::GetTangent(float t)const{
	float tc = 1.0f - t;
	float tc2 = tc * tc;
	float t2 = t * t;
	Vec2f v = (-3.0f * tc2) * p0 + (3.0f * tc * (1.0f - 3.0f * t)) * c0 + (3.0f * t * (2.0f - 3.0f * t)) * c1 + (3.0f * t2) * p1;
	if(v.square() < 1.0e-10f)
		return Vec2f(1.0f, 0.0f);
	return v.unit();
}

Vec2f SVG::Segment::GetTangentDiff(float t)const{
	const float dt = 0.01f;
	Vec2f p0 = GetPoint((t + dt > 1.0f ? t - dt : t));
	Vec2f p1 = GetPoint((t + dt > 1.0f ? t : t + dt));
	return (p1 - p0).unit();
}

void SVG::Segment::GetPoints(vector<Vec2f>& points, bool last){
	const uint div = 10;
	float t = 0.0f;
	float dt = 1.0f / (float)div;
	for(uint i = 0; i < div; i++){
		points.push_back(GetPoint(t));
		t += dt;
	}
	if(last)
		points.push_back(GetPoint(1.0f));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void SVG::Path::GetPoints(vector<Vec2f>& points){
	points.clear();
	uint n = (uint)segments.size();
	for(uint i = 0; i < n; i++){
		segments[i].GetPoints(points, i == n-1);
	}
}

void SVG::Path::CalcBBox(){
	const float inf = numeric_limits<float>::max();

	if(segments.empty()){
		bbmin.clear();
		bbmax.clear();
		return;
	}

	bbmin = Vec2f( inf,  inf);
	bbmax = Vec2f(-inf, -inf);
	for(uint i = 0; i < segments.size(); i++){
		bbmin.x = std::min(bbmin.x, segments[i].p0.x);
		bbmin.y = std::min(bbmin.y, segments[i].p0.y);
		bbmax.x = std::max(bbmax.x, segments[i].p0.x);
		bbmax.y = std::max(bbmax.y, segments[i].p0.y);
	}
	bbmin.x = std::min(bbmin.x, segments.back().p1.x);
	bbmin.y = std::min(bbmin.y, segments.back().p1.y);
	bbmax.x = std::max(bbmax.x, segments.back().p1.x);
	bbmax.y = std::max(bbmax.y, segments.back().p1.y);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

SVG::SVG(){
	strokeWidth = 1.0f;
	strokeColor = "black";
	fillColor   = "none";
	scale       = Vec2f(1.0f, 1.0f);
	offset      = Vec2f(0.0f, 0.0f);
	aspect      = 1.0f;

	affModel.push(Affinef());
}

void SVG::LoadPath(Path* path, XMLNode* node){
	// IDから名前を取得
	path->name = node->GetAttr("id", false);

	// d属性を解析
	string d = node->GetAttr("d", false);

	// 仕様ではラベル(mなど）と座標値の間にデリミタがなくても良いことになっているが、面倒なので考慮しない
	Tokenizer tok(d, ", \t", true);
	string_iterator_pair str;

	try{
		while(!tok.IsEnd()){
			str = tok.GetToken();
			tok.Next();
			// move_to, line_to
			bool moveto     = false;
			bool lineto     = false;
			bool curveto    = false;
			bool close      = false;
			bool vertical   = false;
			bool horizontal = false;
			bool rel        = false;
			if(str == "m"){
				moveto = true;
				rel    = true;
			}
			if(str == "M"){
				moveto = true;
				rel    = false;
			}
			if(str == "l"){
				lineto = true;
				rel    = true;
			}
			if(str == "L"){
				lineto = true;
				rel    = false;
			}
			if(str == "h"){
				lineto     = true;
				rel        = true;
				horizontal = true;
			}
			if(str == "H"){
				lineto     = true;
				rel        = false;
				horizontal = true;
			}
			if(str == "v"){
				lineto     = true;
				rel        = true;
				vertical   = true;
			}
			if(str == "V"){
				lineto     = true;
				rel        = false;
				vertical   = true;
			}
			if(str == "c"){
				curveto = true;
				rel = true;
			}
			if(str == "C"){
				curveto = true;
				rel = false;
			}
			if(str == "Z" || str == "z"){
				close = true;
			}
			if(moveto || lineto){
				try{
					// x, yが任意個つづく．文字列終端か非数値まで読む
					Vec2f p;
					while(true){
						if(horizontal){
							p.x = (float)to_real(tok.GetToken());
							tok.Next();
							p.y = 0.0f;
						}
						else if(vertical){
							p.x = 0.0f;
							p.y = (float)to_real(tok.GetToken());
							tok.Next();
						}
						else{
							p.x = (float)to_real(tok.GetToken());
							tok.Next();
							p.y = (float)to_real(tok.GetToken());
							tok.Next();
						}

						if(!path->segments.empty()){
							Segment& s = path->segments.back();
							if(rel){
								s.p1 = s.p0 + p;
								s.c1 = s.p0 + p;
							}
							else{
								if(horizontal){
									s.p1 = Vec2f(p.x, s.p0.y);
									s.c1 = Vec2f(p.x, s.p0.y);
								}
								else if(vertical){
									s.p1 = Vec2f(s.p0.x, p.y);
									s.c1 = Vec2f(s.p0.x, p.y);
								}
								else{
									s.p1 = p;
									s.c1 = p;
								}
							}

							path->segments.push_back(Segment(s.p1, s.c1));
						}
						else{
							path->segments.push_back(Segment(p, p));
						}
					}
				}
				catch(SyntaxError&){}
			}
			else if(curveto){
				// パスがcで始まることはない
				if(path->segments.empty())
					throw SyntaxError();

				try{
					Vec2f c0, c1, p;
					// (x1, y1, x2, y2, x, y)が任意個つづく．ただし
					// [x1, y1] : 第1制御点
					// [x2, y2] : 第2制御点
					// [x, y] : 次の端点
					// いずれも現在点から相対座標
					// 文字列終端か非数値まで読む
					while(true){
						c0.x = (float)to_real(tok.GetToken()); tok.Next();
						c0.y = (float)to_real(tok.GetToken()); tok.Next();
						c1.x = (float)to_real(tok.GetToken()); tok.Next();
						c1.y = (float)to_real(tok.GetToken()); tok.Next();
						p .x = (float)to_real(tok.GetToken()); tok.Next();
						p .y = (float)to_real(tok.GetToken()); tok.Next();
						
						Segment& s = path->segments.back();
						if(rel){
							s.c0 = s.p0 + c0;
							s.c1 = s.p0 + c1;
							s.p1 = s.p0 + p;
						}
						else{
							s.c0 = c0;
							s.c1 = c1;
							s.p1 = p;
						}
						path->segments.push_back(Segment(s.p1, s.p1));
					}
				}
				catch(SyntaxError&){}
			}
			else if(close){
				if(!path->segments.empty()){
					Segment& s = path->segments.back();
					s.p1 = path->segments[0].p0;
					s.c1 = path->segments[0].c0;
					path->segments.push_back(Segment(s.p1, s.c1));
				}
			}
		}
	}
	catch(...){

	}

	// segmentがひとつ余計にできているので削除
	if(!path->segments.empty())
		path->segments.pop_back();

	path->CalcBBox();
}

void SVG::Load(XMLNode* node){
	if(node->name == "g"){
		Layer* l = new Layer();
		l->name         = node->GetAttr("id", false);
		l->inkscapeName = node->GetAttr("inkscape:label", false);
		l->shown        = !(node->GetAttr("style", false) == "display:none");
		layers.push_back(l);
		layerStack.push_back(l);
	}

	if(node->name == "path"){
		Path* p = new Path();
		p->layer = layerStack.back();
		paths.push_back(p);
		LoadPath(p, node);
	}

	for(uint i = 0; i < node->children.size(); i++){
		XMLNode* child = node->xml->GetNode(node->children[i]);
		Load(child);
	}

	if(node->name == "g")
		layerStack.pop_back();
}

void SVG::Load(XML& xml){
	layerStack.clear();
	layerStack.push_back(0);
	Load(xml.GetRootNode());
}

void SVG::SaveStart(string filename, int w, int h){
	width  = w;
	height = h;
	file.open(filename);
	file << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>" << endl;
	file << "<svg" << endl;
	// Inkscape名前空間（これを書かないとレイヤーが認識されない）
	file << " xmlns:inkscape=\"http://www.inkscape.org/namespaces/inkscape\"" << endl;
	file << " width=\""  << width  << "\"" << endl;
	file << " height=\"" << height << "\"" << endl;
	file << ">" << endl;
}

void SVG::SaveEnd(){
	file << "</svg>" << endl;
	file.close();
}

void SVG::SetScale(float sx, float sy){
	scale.x = sx;
	scale.y = sy;
}

void SVG::SetOffset(float ox, float oy){
	offset.x = ox;
	offset.y = oy;
}

void SVG::SetStrokeColor(string c){
	strokeColor = c;
}

void SVG::SetStrokeWidth(float  w){
	strokeWidth = w;
}

void SVG::SetFillColor(string c){
	fillColor = c;
}

void SVG::PushModelMatrix(){
	affModel.push(affModel.top());
}
void SVG::PopModelMatrix(){
	affModel.pop();
	affTrans = affProj * affViewInv * affModel.top();
}
void SVG::MultModelMatrix(const Affinef& a){
	affModel.top() = affModel.top() * a;
	affTrans = affProj * affViewInv * affModel.top();
}
void SVG::SetModelMatrix(const Affinef& a){
	affModel.top() = a;
	affTrans = affProj * affViewInv * affModel.top();
}

void SVG::SetViewMatrix (const Affinef& a){
	affView    = a;
	affViewInv = affView.inv();
	affTrans   = affProj * affViewInv * affModel.top();
}

void SVG::SetProjMatrix (const Affinef& a){
	affProj  = a;
	affTrans = affProj * affViewInv * affModel.top();
}

void SVG::SetAspect(float a){
	aspect = a;
}

void SVG::WriteStyle(){
	file << " style=\"";
	file << "fill:"         << fillColor   << ";";
	file << "stroke:"       << strokeColor << ";";
	file << "stroke-width:" << strokeWidth << "\"" << endl;
}

Vec2f SVG::Transform(Vec3f p){
	Vec4f q = affTrans * Vec4f(p.x, p.y, p.z, 1.0f);
	float winv = 1.0f / q.w;
	
	// xが[0,1]，yが[0,aspect]の範囲になる
	return Vec2f(
		(q.x * winv + 1.0f) * 0.5f,
		(q.y * winv + 1.0f) * 0.5f * aspect);
}

void SVG::BeginLayer(string name, bool shown){
	// inkscapeがレイヤーとして認識するようにする
	file << "<g";
	file << " inkscape:groupmode=\"layer\"";
	file << " id=\"" << name << "\"";
	file << " inkscape:label=\"" << name << "\"";
	if(shown)
		 file << " style=\"display:inline\"";
	else file << " style=\"display:none\"";
	file << ">" << endl;
}

void SVG::EndLayer(){
	file << "</g>" << endl;
}

void SVG::BeginPath(){
	file << "<path" << endl;
	WriteStyle();
	file << " d=\"";
}

void SVG::EndPath(bool loop){
	if(loop)
		file << "Z";
	file << "\"/>" << endl;
}

void SVG::MoveTo(Vec2f p){
	file << "M "
	     << offset.x + scale.x * p.x << ", "
		 << offset.y + scale.y * p.y << " ";
}

void SVG::LineTo(Vec2f p){
	file << "L "
	     << offset.x + scale.x * p.x << ", "
		 << offset.y + scale.y * p.y << " ";
}

void SVG::MoveTo(Vec3f p){
	MoveTo(Transform(p));
}

void SVG::LineTo(Vec3f p){
	LineTo(Transform(p));
}

void SVG::DrawLine(Vec2f p0, Vec2f p1){
	BeginPath();
	MoveTo(p0);
	LineTo(p1);
	EndPath(false);
}

void SVG::DrawLine(Vec3f p0, Vec3f p1){
	BeginPath();
	MoveTo(p0);
	LineTo(p1);
	EndPath(false);
}

void SVG::DrawCircle(Vec2f c, float r){
	file << "<circle" << endl;
	WriteStyle();
	file << " cx=\"" << offset.x + scale.x * c.x << "\"";
	file << " cy=\"" << offset.y + scale.y * c.y << "\"";
	file << " r=\""  << std::abs(scale.x)  * r   << "\"";
	file << "/>" << endl;
}

void SVG::DrawCircle(Vec3f c, float r, int ndiv){
	BeginPath();
	float ds = (2.0f * (float)M_PI) / (float)ndiv;
	float s  = 0.0f;
	for(int i = 0; i < ndiv; i++){
		float _cos = cos(s);
		float _sin = sin(s);
		Vec3f p = c + Vec3f(r*_cos, r*_sin, 0.0f);
		if(i == 0)
			 MoveTo(p);
		else LineTo(p);
		s += ds;
	}
	EndPath(true);
}

void SVG::DrawRect(Vec2f p0, Vec2f p1){
	file << "<rect" << endl;
	WriteStyle();
	file << "x=\""      << p0.x      << "\"";
	file << "y=\""      << p0.y      << "\"";
	file << "width=\""  << p1.x-p0.x << "\"";
	file << "height=\"" << p1.y-p0.y << "\"";
	file << "/>" << endl;
}

void SVG::DrawBox(Vec3f rmin, Vec3f rmax){
	DrawLine(Vec3f(rmin[0], rmin[1], rmin[2]), Vec3f(rmin[0], rmax[1], rmin[2]));
	DrawLine(Vec3f(rmin[0], rmax[1], rmin[2]), Vec3f(rmin[0], rmax[1], rmax[2]));
	DrawLine(Vec3f(rmin[0], rmax[1], rmax[2]), Vec3f(rmin[0], rmin[1], rmax[2]));
	DrawLine(Vec3f(rmin[0], rmin[1], rmax[2]), Vec3f(rmin[0], rmin[1], rmin[2]));

	DrawLine(Vec3f(rmax[0], rmin[1], rmin[2]), Vec3f(rmax[0], rmax[1], rmin[2]));
	DrawLine(Vec3f(rmax[0], rmax[1], rmin[2]), Vec3f(rmax[0], rmax[1], rmax[2]));
	DrawLine(Vec3f(rmax[0], rmax[1], rmax[2]), Vec3f(rmax[0], rmin[1], rmax[2]));
	DrawLine(Vec3f(rmax[0], rmin[1], rmax[2]), Vec3f(rmax[0], rmin[1], rmin[2]));
	
	DrawLine(Vec3f(rmin[0], rmin[1], rmin[2]), Vec3f(rmax[0], rmin[1], rmin[2]));
	DrawLine(Vec3f(rmin[0], rmax[1], rmin[2]), Vec3f(rmax[0], rmax[1], rmin[2]));
	DrawLine(Vec3f(rmin[0], rmax[1], rmax[2]), Vec3f(rmax[0], rmax[1], rmax[2]));
	DrawLine(Vec3f(rmin[0], rmin[1], rmax[2]), Vec3f(rmax[0], rmin[1], rmax[2]));
}

void SVG::DrawSphere(Vec3f p, float r){
	/// T.B.D.
}

}
