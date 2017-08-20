#include <sbconverter.h>
#include <sbtokenizer.h>

#include <windows.h>

namespace Scenebuilder{;

//-------------------------------------------------------------------------------------------------

void Converter::ToString(ostream& os, const char* val){
	os << val;
}
void Converter::ToString(ostream& os, bool val){
	os << (val ? "true" : "false");
}
void Converter::ToString(ostream& os, int val){
	os << val;
}
void Converter::ToString(ostream& os, float val){
	os << val;
}
void Converter::ToString(ostream& os, double val){
	os << val;
}
void Converter::ToString(ostream& os, const VVecf& val){
	for(uint i = 0; i < val.size(); i++){
		os << val[i];
		if(i != val.size()-1)
			os << ' ';
	}
}
void Converter::ToString(ostream& os, const VVecd& val){
	for(uint i = 0; i < val.size(); i++){
		os << val[i];
		if(i != val.size()-1)
			os << ' ';
	}
}
void Converter::ToString(ostream& os, const Vec2f& val){
	os << val[0] << ' ' << val[1];
}
void Converter::ToString(ostream& os, const Vec2d& val){
	os << val[0] << ' ' << val[1];
}
void Converter::ToString(ostream& os, const Vec3f& val){
	os << val[0] << ' ' << val[1] << ' ' << val[2];
}
void Converter::ToString(ostream& os, const Vec3d& val){
	os << val[0] << ' ' << val[1] << ' ' << val[2];
}
void Converter::ToString(ostream& os, const Vec4f& val){
	os << val[0] << ' ' << val[1] << ' ' << val[2] << ' ' << val[3];
}
void Converter::ToString(ostream& os, const Vec4d& val){
	os << val[0] << ' ' << val[1] << ' ' << val[2] << ' ' << val[3];
}
void Converter::ToString(ostream& os, const Quaternionf& val){
	os << val[0] << ' ' << val[1] << ' ' << val[2] << ' ' << val[3];
}
void Converter::ToString(ostream& os, const Quaterniond& val){
	os << val[0] << ' ' << val[1] << ' ' << val[2] << ' ' << val[3];
}
void Converter::ToString(ostream& os, const Matrix3f& val){
	for(uint i = 0; i < 3; i++)for(uint j = 0; j < 3; j++){
		os << val[i][j];
		if(!(i == 2 && j == 2))
			os << ' ';
	}
}
void Converter::ToString(ostream& os, const Matrix3d& val){
	for(uint i = 0; i < 3; i++)for(uint j = 0; j < 3; j++){
		os << val[i][j];
		if(!(i == 2 && j == 2))
			os << ' ';
	}
}

//-------------------------------------------------------------------------------------------------

void Converter::ConvertString(string& str, const wstring& wstr, bool utf8){
	if(wstr.empty()){
		str.clear();
		return;
	}
	uint cp = (utf8 ? CP_UTF8 : CP_ACP);
	int sz = (int)WideCharToMultiByte(cp, 0, &wstr[0], (int)wstr.size(), 0, 0, 0, 0);
	str.resize(sz);
	WideCharToMultiByte(cp, 0, &wstr[0], (int)wstr.size(), &str[0], sz, 0, 0);
}

void Converter::ConvertString(wstring& wstr, const string& str, bool utf8){
	if(str.empty()){
		wstr.clear();
		return;
	}
	uint cp = (utf8 ? CP_UTF8 : CP_ACP);
	int sz = (int)MultiByteToWideChar(cp, 0, &str[0], (int)str.size(), 0, 0);
	wstr.resize(sz);
	MultiByteToWideChar(cp, 0, &str[0], (int)str.size(), &wstr[0], sz);
	return;
}

//-------------------------------------------------------------------------------------------------

bool Converter::FromString(string_iterator_pair str, char* val, size_t len){
	size_t sz = str.size();
	if(sz >= len)
		sz = len-1;
	if(sz >= 0){
		copy(str.first, str.first + sz, val);
		val[sz] = '\0';
	}
	return true;
}

bool Converter::FromString(string_iterator_pair str, string& val){
	val = string(str.begin(), str.end());
	return true;
}

/* bool値
	- true : 1, true, True, TRUE
	- false: 0, false, False, FALSE
 */
bool Converter::FromString(string_iterator_pair str, bool& val){
	str = eat_white(str);
	if(str == "1" || str == "true" || str == "True" || str == "TRUE"){
		val = true;
		return true;
	}
	if(str == "0" || str == "false" || str == "False" || str == "FALSE"){
		val = false;
		return true;
	}
	return false;
}

bool Converter::FromString(string_iterator_pair str, int& val){
	try{
		val = to_int(str);
		return true;
	}
	catch(SyntaxError&){
		return false;
	}
}

bool Converter::FromString(string_iterator_pair str, uint& val){
	return FromString(str, (int&)val);
}

/* 実数値
	- 数値部 + [単位部]
 */
bool Converter::FromString(string_iterator_pair str, float& val, int dim){
	try{
		val = (float)Calc()(eat_white(str), dim);
		return true;
	}
	catch(CalcException&){
		return false;
	}
}
bool Converter::FromString(string_iterator_pair str, double& val, int dim){
	try{
		val = Calc()(eat_white(str), dim);
		return true;
	}
	catch(CalcException&){
		return false;
	}
}

bool Converter::FromString(string_iterator_pair str, VVecf& val, int dim){
	VVecd tmp;
	if(FromString(str, tmp, dim)){
		for(uint i = 0; i < val.size(); i++)
			val[i] = (float)tmp[i];
		return true;
	}
	return false;
}
bool Converter::FromString(string_iterator_pair str, VVecd& val, int dim){
	str = eat_white(str);
	// vvec_t(PTM::VVector)::resizeは内容を保持してくれないので，一度目のパースで要素数を調べ，二度目のパースで数値を読む
	size_t n;
	double elem;
	
	// 括弧をスキップしつつトークン化
	Tokenizer tok;
	for(n = 0, tok.Set(str, " \t", true, true); !tok.IsEnd(); n++, tok.Next());

	val.resize(n);

	for(n = 0, tok.Set(str, " \t", true, true); !tok.IsEnd(); n++, tok.Next()){
		if(!FromString(tok.GetToken(), elem, dim))
			return false;
		val[n] = elem;
	}
	return true;
}

bool Converter::FromString(string_iterator_pair str, Vec2f& val, int dim){
	Vec2d tmp;
	if(FromString(str, tmp, dim)){
		val = tmp;
		return true;
	}
	return false;
}
bool Converter::FromString(string_iterator_pair str, Vec2d& val, int dim){
	VVecd v;
	if(FromString(str, v, dim)){
		if(v.size() == 1){
			val[0] = val[1] = v[0];
			return true;
		}
		if(v.size() == 2){
			val = (const vec2_t&)v[0];
			return true;
		}
		return false;
	}
	return false;
}

bool Converter::FromString(string_iterator_pair str, Vec3f& val, int dim){
	Vec3d tmp;
	if(FromString(str, tmp, dim)){
		val = tmp;
		return true;
	}
	return false;
}
bool Converter::FromString(string_iterator_pair str, Vec3d& val, int dim){
	VVecd v;
	if(FromString(str, v, dim)){
		if(v.size() == 1){
			val[0] = val[1] = val[2] = v[0];
			return true;
		}
		if(v.size() == 3){
			val = (const vec3_t&)v[0];
			return true;
		}
		return false;
	}
	return false;
}

bool Converter::FromString(string_iterator_pair str, Vec4f& val, int dim){
	Vec4d tmp;
	if(FromString(str, tmp, dim)){
		val = tmp;
		return true;
	}
	return false;
}
bool Converter::FromString(string_iterator_pair str, Vec4d& val, int dim){
	VVecd v;
	if(FromString(str, v, dim)){
		if(v.size() == 1){
			val[0] = val[1] = val[2] = val[3] = v[0];
			return true;
		}
		if(v.size() == 4){
			val = (const vec4_t&)v[0];
			return true;
		}
		return false;
	}
	return false;
}

/* クォータニオン
	- 合成クォータニオン
		- *区切りで基本クォータニオン型が並ぶ
	- 基本クォータニオン
		- 9次元ベクトル			-> 回転行列			1 0 0; 0 1 0; 0 0 1
		- 4次元ベクトル			-> 成分表記			1 0 0 0
		- 実数値@x|y|z			-> 回転角+回転軸		90deg@x
		- 実数値@3次元ベクトル	-> 回転角+回転軸		0.5@0 1 0
 */
bool Converter::FromString(string_iterator_pair str, Quaternionf& val, int dim){
	Quaterniond tmp;
	if(FromString(str, tmp, dim)){
		val = tmp;
		return true;
	}
	return false;
}
bool Converter::FromString(string_iterator_pair str, Quaterniond& val, int dim){
	str = eat_white(str);
	// 括弧考慮しつつ*で区切る（回転合成）
	Tokenizer tok0(str, "*", true, true);
	
	val = Quaterniond();
	Quaterniond q;
	Matrix3d    R;
	
	while(!tok0.IsEnd()){
		// 4次元ベクトル表記
		if(FromString(tok0.GetToken(), (vec4_t&)q, Dimension::None)){
			val = val * q;
		}
		// 3x3回転行列表記
		else if(FromString(tok0.GetToken(), R, Dimension::None)){
			q.FromMatrix(R);
			val = val * q;
		}
		// @区切り
		else{
			double angle;
			Vec3d  axis;
			
			Tokenizer tok1(tok0.GetToken(), "@", false, true);
			if(tok1.IsEnd())
				return false;
			if(!FromString(tok1.GetToken(), angle, dim))
				return false;
			tok1.Next();
			if(tok1.IsEnd())
				return false;

			string_iterator_pair straxis = tok1.GetToken();
			if(FromString(straxis, axis, Dimension::None)){
				val = val * quat_t::Rot(angle, axis);
			}
			else if(straxis == "x" || straxis == "X")
				val = val * quat_t::Rot(angle, 'x');
			else if(straxis == "y" || straxis == "Y")
				val = val * quat_t::Rot(angle, 'y');
			else if(straxis == "z" || straxis == "Z")
				val = val * quat_t::Rot(angle, 'z');
			else
				return false;
		}
		tok0.Next();
	}
	return true;
}

bool Converter::FromString(string_iterator_pair str, Matrix3f& val, int dim){
	Matrix3d tmp;
	if(FromString(str, tmp, dim)){
		val = tmp;
		return true;
	}
	return false;
}
bool Converter::FromString(string_iterator_pair str, Matrix3d& val, int dim){
	VVecd v;
	if(FromString(str, v, dim)){
		// 3次元ベクトルは対角行列へ，スカラは単位行列のスカラ倍はそれぞれ変換する
		if(v.size() == 9){
			val = (const mat3_t&)v[0];
			return true;
		}
		if(v.size() == 3){
			val = mat3_t::Diag(v[0], v[1], v[2]);
			return true;
		}
		if(v.size() == 1){
			val = mat3_t::Unit() * v[0];
			return true;
		}
		return false;
	}
	return false;
}

//-------------------------------------------------------------------------------------------------

void Converter::ToBinary(byte*& buf, const char* val, size_t len){
	strncpy((char*)buf, val, len);
	if(len != 0)
		buf[len-1] = 0;
	buf += len;
}
void Converter::ToBinary(byte*& buf, const string& val){
	strcpy((char*)buf, val.c_str());
	buf += val.size()+1;
}
void Converter::ToBinary(byte*& buf, const VVecf& val){
	uint n = (uint)val.size();
	*(uint*)buf = n;
	buf += sizeof(uint);
	for(uint i = 0; i < n; i++){
		*(float*)buf = val[i];
		buf += sizeof(float);
	}
}
void Converter::ToBinary(byte*& buf, const VVecd& val){
	uint n = (uint)val.size();
	*(uint*)buf = n;
	buf += sizeof(uint);
	for(uint i = 0; i < n; i++){
		*(double*)buf = val[i];
		buf += sizeof(double);
	}
}
void Converter::ToBinary(ostream& os, const char* val, size_t len){
	int n = (int)strlen(val);
	for(int i = 0; i < (int)len-1; i++)
		ToBinary(os, i < n ? val[i] : '\0');
	if(len != 0)
		ToBinary(os, '\0');
}
void Converter::ToBinary(ostream& os, const string& val){
	os << val << '\0';
}
void Converter::ToBinary(ostream& os, const VVecf& val){
	uint n = (uint)val.size();
	ToBinary(os, n);
	for(uint i = 0; i < n; i++)
		ToBinary(os, val[i]);
}
void Converter::ToBinary(ostream& os, const VVecd& val){
	uint n = (uint)val.size();
	ToBinary(os, n);
	for(uint i = 0; i < n; i++)
		ToBinary(os, val[i]);
}
	
//-------------------------------------------------------------------------------------------------

void Converter::FromBinary(const byte*& buf, char* val, size_t len){
	size_t n = strlen((const char*)buf);
	size_t sz = (n < len ? n : len-1);
	if(sz >= 0){
		copy(buf, buf + sz, val);
		val[sz] = '\0';
	}
	buf += n+1;
}
void Converter::FromBinary(const byte*& buf, string& val){
	size_t n = strlen((const char*)buf);
	val.assign(buf, buf + n);
	buf += n+1;
}
void Converter::FromBinary(const byte*& buf, VVecf& val){
	uint n = *(uint*)buf;
	val.resize(n);
	for(uint i = 0; i < n; i++){
		val[i] = *(float*)buf;
		buf += sizeof(float);
	}
}
void Converter::FromBinary(const byte*& buf, VVecd& val){
	uint n = *(uint*)buf;
	val.resize(n);
	for(uint i = 0; i < n; i++){
		val[i] = *(double*)buf;
		buf += sizeof(double);
	}
}
void Converter::FromBinary(istream& is, char* val, size_t len){
	string tmp;
	is >> tmp;
	strncpy(val, tmp.c_str(), len);
}
void Converter::FromBinary(istream& is, string& val){
	val.clear();
	char c;
	while(true){
		FromBinary(is, c);
		if(c == '\0')
			return;
		val.push_back(c);
	}
}
void Converter::FromBinary(istream& is, VVecf& val){
	uint n;
	FromBinary(is, n);
	val.resize(n);
	for(uint i = 0; i < n; i++)
		FromBinary(is, val[i]);
}
void Converter::FromBinary(istream& is, VVecd& val){
	uint n;
	FromBinary(is, n);
	val.resize(n);
	for(uint i = 0; i < n; i++)
		FromBinary(is, val[i]);
}

//-------------------------------------------------------------------------------------------------

bool Converter::ColorFromName(string_iterator_pair name, Vec4f& c){
	#undef  RGB
	#define RGB(x, y, z) Vec4f((float)x/255.0f, (float)y/255.0f, (float)z/255.0f, 1.0f)
	
	// 無色 
	if(name == "none"                ){ c = Vec4f();            return true; }

	//
	if(name == "indianred"           ){ c = RGB(205,  92,  92); return true; }
	if(name == "lightcoral"          ){ c = RGB(240, 128, 128); return true; }
	if(name == "salmon"              ){ c = RGB(250, 128, 114); return true; }
	if(name == "darksalmon"          ){ c = RGB(233, 150, 122); return true; }
	if(name == "lightsalmon"         ){ c = RGB(255, 160, 122); return true; }
	if(name == "red"                 ){ c = RGB(255,   0,   0); return true; }
	if(name == "crimson"             ){ c = RGB(220,  20,  60); return true; }
	if(name == "firebrick"           ){ c = RGB(178,  34,  34); return true; }
	if(name == "darkred"             ){ c = RGB(139,   0,   0); return true; }
	if(name == "pink"                ){ c = RGB(255, 192, 203); return true; }
	if(name == "lightpink"           ){ c = RGB(255, 182, 193); return true; }
	if(name == "hotpink"             ){ c = RGB(255, 105, 180); return true; }
	if(name == "deeppink"            ){ c = RGB(255,  20, 147); return true; }
	if(name == "mudiumvioletred"     ){ c = RGB(199,  21, 133); return true; }
	if(name == "palevioletred"       ){ c = RGB(219, 112, 147); return true; }
	if(name == "coral"               ){ c = RGB(255, 127,  80); return true; }
	if(name == "tomato"              ){ c = RGB(255,  99,  71); return true; }
	if(name == "orangered"           ){ c = RGB(255,  69,   0); return true; }
	if(name == "darkorange"          ){ c = RGB(255, 140,   0); return true; }
	if(name == "orange"              ){ c = RGB(255, 165,   0); return true; }
	if(name == "gold"                ){ c = RGB(255, 215,   0); return true; }
	if(name == "yellow"              ){ c = RGB(255, 255,   0); return true; }
	if(name == "lightyellow"         ){ c = RGB(255, 255, 224); return true; }
	if(name == "lemonchiffon"        ){ c = RGB(255, 250, 205); return true; }
	if(name == "lightgoldenrodyellow"){ c = RGB(250, 250, 210); return true; }
	if(name == "papayawhip"          ){ c = RGB(255, 239, 213); return true; }
	if(name == "moccasin"            ){ c = RGB(255, 228, 181); return true; }
	if(name == "peachpuff"           ){ c = RGB(255, 218, 185); return true; }
	if(name == "palegoldenrod"       ){ c = RGB(238, 232, 170); return true; }
	if(name == "khaki"               ){ c = RGB(240, 230, 140); return true; }
	if(name == "darkkhaki"           ){ c = RGB(189, 183, 107); return true; }
	if(name == "lavender"            ){ c = RGB(230, 230, 250); return true; }
	if(name == "thistle"             ){ c = RGB(216, 191, 216); return true; }
	if(name == "plum"                ){ c = RGB(221, 160, 221); return true; }
	if(name == "violet"              ){ c = RGB(238, 130, 238); return true; }
	if(name == "orchild"             ){ c = RGB(218, 112, 214); return true; }
	if(name == "fuchsia"             ){ c = RGB(255,   0, 255); return true; }
	if(name == "magenta"             ){ c = RGB(255,   0, 255); return true; }
	if(name == "mediumorchild"       ){ c = RGB(186,  85, 211); return true; }
	if(name == "mediumpurple"        ){ c = RGB(147, 112, 219); return true; }
	if(name == "blueviolet"          ){ c = RGB(138,  43, 226); return true; }
	if(name == "darkviolet"          ){ c = RGB(148,   0, 211); return true; }
	if(name == "darkorchild"         ){ c = RGB(153,  50, 204); return true; }
	if(name == "darkmagenta"         ){ c = RGB(139,   0, 139); return true; }
	if(name == "purple"              ){ c = RGB(128,   0, 128); return true; }
	if(name == "indigo"              ){ c = RGB( 75,   0, 130); return true; }
	if(name == "darkslateblue"       ){ c = RGB( 72,  61, 139); return true; }
	if(name == "slateblue"           ){ c = RGB(106,  90, 205); return true; }
	if(name == "mediumslateblue"     ){ c = RGB(123, 104, 238); return true; }
	if(name == "greenyellow"         ){ c = RGB(173, 255,  47); return true; }
	if(name == "chartreuse"          ){ c = RGB(127, 255,   0); return true; }
	if(name == "lawngreen"           ){ c = RGB(124, 252,   0); return true; }
	if(name == "lime"                ){ c = RGB(  0, 252,   0); return true; }
	if(name == "limegreen"           ){ c = RGB( 50, 205,  50); return true; }
	if(name == "palegreen"           ){ c = RGB(152, 251, 152); return true; }
	if(name == "lightgreen"          ){ c = RGB(144, 238, 144); return true; }
	if(name == "mediumspringgreen"   ){ c = RGB(  0, 250, 154); return true; }
	if(name == "springgreen"         ){ c = RGB(  0, 255, 127); return true; }
	if(name == "mediumseagreen"      ){ c = RGB( 60, 179, 113); return true; }
	if(name == "seagreen"            ){ c = RGB( 46, 139,  87); return true; }
	if(name == "forestgreen"         ){ c = RGB( 34, 139,  34); return true; }
	if(name == "green"               ){ c = RGB(  0, 128,   0); return true; }
	if(name == "darkgreen"           ){ c = RGB(  0, 100,   0); return true; }
	if(name == "yellowgreen"         ){ c = RGB(154, 205,  50); return true; }
	if(name == "olivedrab"           ){ c = RGB(107, 142,  35); return true; }
	if(name == "olive"               ){ c = RGB(128, 128,   0); return true; }
	if(name == "darkolivegreen"      ){ c = RGB( 85, 107,  47); return true; }
	if(name == "mediumaquamarine"    ){ c = RGB(102, 205, 170); return true; }
	if(name == "darkseagreen"        ){ c = RGB(143, 188, 143); return true; }
	if(name == "lightseagreen"       ){ c = RGB( 32, 178, 170); return true; }
	if(name == "darkcyan"            ){ c = RGB(  0, 139, 139); return true; }
	if(name == "teal"                ){ c = RGB(  0, 128, 128); return true; }
	if(name == "aqua"                ){ c = RGB(  0, 255, 255); return true; }
	if(name == "cyan"                ){ c = RGB(  0, 255, 255); return true; }
	if(name == "lightcyan"           ){ c = RGB(224, 255, 255); return true; }
	if(name == "paleturquoise"       ){ c = RGB(175, 238, 238); return true; }
	if(name == "aquamarine"          ){ c = RGB(127, 255, 212); return true; }
	if(name == "turquoise"           ){ c = RGB( 64, 224, 208); return true; }
	if(name == "mediumturquoise"     ){ c = RGB( 72, 209, 204); return true; }
	if(name == "darkturquoise"       ){ c = RGB(  0, 206, 209); return true; }
	if(name == "cadetblue"           ){ c = RGB( 95, 158, 160); return true; }
	if(name == "steelblue"           ){ c = RGB( 70, 130, 180); return true; }
	if(name == "lightsteelblue"      ){ c = RGB(176, 196, 222); return true; }
	if(name == "powderblue"          ){ c = RGB(176, 224, 230); return true; }
	if(name == "lightblue"           ){ c = RGB(173, 216, 230); return true; }
	if(name == "skyblue"             ){ c = RGB(135, 206, 235); return true; }
	if(name == "lightskyblue"        ){ c = RGB(135, 206, 250); return true; }
	if(name == "deepskyblue"         ){ c = RGB(  0, 191, 255); return true; }
	if(name == "dodgerblue"          ){ c = RGB( 30, 144, 255); return true; }
	if(name == "cornflowerblue"      ){ c = RGB(100, 149, 237); return true; }
	if(name == "royalblue"           ){ c = RGB( 65, 105, 225); return true; }
	if(name == "blue"                ){ c = RGB(  0,   0, 255); return true; }
	if(name == "mediumblue"          ){ c = RGB(  0,   0, 205); return true; }
	if(name == "darkblue"            ){ c = RGB(  0,   0, 139); return true; }
	if(name == "navy"                ){ c = RGB(  0,   0, 128); return true; }
	if(name == "midnightblue"        ){ c = RGB( 25,  25, 112); return true; }
	if(name == "cornsilk"            ){ c = RGB(255, 248, 220); return true; }
	if(name == "blanchedalmond"      ){ c = RGB(255, 235, 205); return true; }
	if(name == "bisque"              ){ c = RGB(255, 228, 196); return true; }
	if(name == "navajowhite"         ){ c = RGB(255, 222, 173); return true; }
	if(name == "wheat"               ){ c = RGB(245, 222, 179); return true; }
	if(name == "burlywood"           ){ c = RGB(222, 184, 135); return true; }
	if(name == "tan"                 ){ c = RGB(210, 180, 140); return true; }
	if(name == "rosybrown"           ){ c = RGB(188, 143, 143); return true; }
	if(name == "sandybrown"          ){ c = RGB(244, 164,  96); return true; }
	if(name == "goldenrod"           ){ c = RGB(218, 165,  32); return true; }
	if(name == "darkgoldenrod"       ){ c = RGB(184, 134,  11); return true; }
	if(name == "peru"                ){ c = RGB(205, 133,  63); return true; }
	if(name == "chocolate"           ){ c = RGB(210, 105,  30); return true; }
	if(name == "saddlebrown"         ){ c = RGB(139,  69,  19); return true; }
	if(name == "sienna"              ){ c = RGB(160,  82,  45); return true; }
	if(name == "brown"               ){ c = RGB(165,  42,  42); return true; }
	if(name == "maroon"              ){ c = RGB(128,   0,   0); return true; }
	if(name == "white"               ){ c = RGB(255, 255, 255); return true; }
	if(name == "snow"                ){ c = RGB(255, 250, 250); return true; }
	if(name == "honeydew"            ){ c = RGB(240, 255, 240); return true; }
	if(name == "mintcream"           ){ c = RGB(245, 255, 250); return true; }
	if(name == "azure"               ){ c = RGB(240, 255, 255); return true; }
	if(name == "aliceblue"           ){ c = RGB(240, 248, 255); return true; }
	if(name == "ghostwhite"          ){ c = RGB(248, 248, 255); return true; }
	if(name == "whitesmoke"          ){ c = RGB(245, 245, 245); return true; }
	if(name == "seashell"            ){ c = RGB(255, 245, 238); return true; }
	if(name == "beige"               ){ c = RGB(245, 245, 220); return true; }
	if(name == "oldlace"             ){ c = RGB(253, 245, 230); return true; }
	if(name == "floralwhite"         ){ c = RGB(255, 250, 240); return true; }
	if(name == "ivory"               ){ c = RGB(255, 255, 240); return true; }
	if(name == "antiquewhite"        ){ c = RGB(250, 235, 215); return true; }
	if(name == "linen"               ){ c = RGB(250, 240, 230); return true; }
	if(name == "lavenderblush"       ){ c = RGB(255, 240, 245); return true; }
	if(name == "mistyrose"           ){ c = RGB(255, 228, 225); return true; }
	if(name == "gainsboro"           ){ c = RGB(220, 220, 220); return true; }
	if(name == "lightgray"           ){ c = RGB(211, 211, 211); return true; }
	if(name == "silver"              ){ c = RGB(192, 192, 192); return true; }
	if(name == "darkgray"            ){ c = RGB(169, 169, 169); return true; }
	if(name == "gray"                ){ c = RGB(128, 128, 128); return true; }
	if(name == "dimgray"             ){ c = RGB(105, 105, 105); return true; }
	if(name == "lightslategray"      ){ c = RGB(119, 136, 153); return true; }
	if(name == "slategray"           ){ c = RGB(112, 128, 144); return true; }
	if(name == "darkslategray"       ){ c = RGB( 47,  79,  79); return true; }
	if(name == "black"               ){ c = RGB(  0,   0,   0); return true; }
	#undef RGB
	return false;
}

}
