#pragma once

#include <sbtypes.h>
#include <sbcalc.h>

namespace Scenebuilder{;

/** Color

 */
class Color{
public:
	string name;
	Vec4f  rgba;

public:
	void Init();

	Color(string n = "white"){
		name = n;
		Init();
	}
};

inline Color operator*(const Color& c0, const Color& c1){
	Color c;
	c.rgba[0] = c0.rgba[0] * c1.rgba[0];
	c.rgba[1] = c0.rgba[1] * c1.rgba[1];
	c.rgba[2] = c0.rgba[2] * c1.rgba[2];
	c.rgba[3] = c0.rgba[3] * c1.rgba[3];
	return c;
}

/** Converter

	各種変換機能
	- 数値　->　文字列
	- 文字列 -> 数値
	- 文字列間のエンコード変換

 */
class Converter{
public:
	/// conversion from UNICODE (UTF-16) to multi-byte
	static void ConvertString(string& str, const wstring& wstr, bool utf8 = true);
	/// conversion from multi-byte to UNICODE (UTF-16)
	static void ConvertString(wstring& wstr, const string& str, bool utf8 = true);
	
	/** conversion between primitive types and string **/
	static void ToString( ostream& os, const char*        val);
	static void ToString( ostream& os, bool               val);
	static void ToString( ostream& os, int                val);
	static void ToString( ostream& os, float              val);
	static void ToString( ostream& os, double             val);
	static void ToString( ostream& os, const VVecf      & val);
	static void ToString( ostream& os, const VVecd      & val);
	static void ToString( ostream& os, const Vec2f      & val);
	static void ToString( ostream& os, const Vec2d      & val);
	static void ToString( ostream& os, const Vec3f      & val);
	static void ToString( ostream& os, const Vec3d      & val);
	static void ToString( ostream& os, const Vec4f      & val);
	static void ToString( ostream& os, const Vec4d      & val);
	static void ToString( ostream& os, const Quaternionf& val);
	static void ToString( ostream& os, const Quaterniond& val);
	static void ToString( ostream& os, const Matrix3f   & val);
	static void ToString( ostream& os, const Matrix3d   & val);
	
	/** @brief	decodes a given string into a given type
		@param	value	value in string
		**/
	static bool FromString(string_iterator_pair str, char*  val, size_t len);
	static bool FromString(string_iterator_pair str, string        &  val);
	static bool FromString(string_iterator_pair str, Color         &  val);
	static bool FromString(string_iterator_pair str, bool          &  val);
	static bool FromString(string_iterator_pair str, int           &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, uint          &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, float         &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, double        &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, VVecf         &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, VVecd         &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, vector<int>   &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, vector<float> &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, vector<double>&  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, Vec2f         &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, Vec2d         &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, Vec3f         &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, Vec3d         &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, Vec4f         &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, Vec4d         &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, Quaternionf   &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, Quaterniond   &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, Matrix3f      &  val, int dim = Dimension::None);
	static bool FromString(string_iterator_pair str, Matrix3d      &  val, int dim = Dimension::None);
	
	template<typename T>
	static bool FromString(string_iterator_pair str, vector<T>&  val, int dim = Dimension::None){

	}
	
	/// ウェブカラー名からRGBAを取得
	static bool ColorFromName(string_iterator_pair name, Vec4f& c);
	/// インデックスからRGBAを取得
	static bool ColorFromIndex(int index, Vec4f& c);
	
	template<typename T>
	static void ToBinary(byte*& buf, T val){
		*(T*)buf = val;
		buf += sizeof(T);
	}
	static void ToBinary(byte*& buf, const char*   val, size_t len);
	static void ToBinary(byte*& buf, const string& val);
	static void ToBinary(byte*& buf, const VVecf & val);
	static void ToBinary(byte*& buf, const VVecd & val);
	
	template<typename T>
	static void ToBinary(ostream& os, T val){
		os.write((const char*)&val, sizeof(T));
	}
	static void ToBinary(ostream& os, const char*   val, size_t len);
	static void ToBinary(ostream& os, const string& val);
	static void ToBinary(ostream& os, const VVecf & val);
	static void ToBinary(ostream& os, const VVecd & val);
	
	template<typename T>
	static void FromBinary(const byte*& buf, T& val){
		val = *(const T*)buf;
		buf += sizeof(T);
	}
	static void FromBinary(const byte*& buf, char*   val, size_t len);
	static void FromBinary(const byte*& buf, string& val);
	static void FromBinary(const byte*& buf, VVecf & val);
	static void FromBinary(const byte*& buf, VVecd & val);
	
	template<typename T>
	static void FromBinary(istream& is, T& val){
		is.read((char*)&val, sizeof(T));
	}
	static void FromBinary(istream& is, char*   val, size_t len);
	static void FromBinary(istream& is, string& val);
	static void FromBinary(istream& is, VVecf & val);
	static void FromBinary(istream& is, VVecd & val);
	
};

}
