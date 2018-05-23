#pragma once

// disable Visual C++ decprecation warnings
#define	_SCL_SECURE_NO_WARNINGS
#define	_CRT_SECURE_NO_WARNINGS

#include <Base/TQuaternion.h>
#include <Base/TCurve.h>
#include <Base/BaseUtility.h>
using namespace Spr;

#include <memory>
#include <vector>
#include <locale>
#include <functional>
using namespace std;

namespace Scenebuilder{;

// window.hをインクルードするとrpcndr.h内でbyteがtypedefされるので干渉回避が必要
#ifndef USE_WINDOWS_H
typedef unsigned char	byte;
#endif

#ifndef SB_UINT_DEFINED
typedef unsigned short  ushort;
typedef unsigned int	uint;
#endif

typedef PTM::VVector<float>     VVecf;
typedef PTM::VVector<double>    VVecd;
typedef PTM::VMatrixCol<float>  VMatrixf;
typedef PTM::VMatrixCol<double> VMatrixd;

#ifdef SB_USE_SINGLE_PRECISION
/// use single-precision
typedef	float			real_t;
typedef Vec2f			vec2_t;
typedef Vec3f			vec3_t;
typedef Vec4f			vec4_t;
typedef Vec6f			vec6_t;
typedef VVecf		    vvec_t;
typedef Quaternionf		quat_t;
typedef Posef			pose_t;
typedef Matrix2f		mat2_t;
typedef Matrix3f		mat3_t;
typedef VMatrixf        vmat_t;
typedef Curve3f			curve3_t;
#else
/// use doulbe-precision
typedef	double			real_t;
typedef Vec2d			vec2_t;
typedef Vec3d			vec3_t;
typedef Vec4d			vec4_t;
typedef Vec6d			vec6_t;
typedef VVecd           vvec_t;
typedef Quaterniond		quat_t;
typedef Posed			pose_t;
typedef Matrix2d		mat2_t;
typedef Matrix3d		mat3_t;
typedef VMatrixd        vmat_t;
typedef Curve3d			curve3_t;
#endif

/* string_iterator_pair
 */
template<class T>
struct basic_string_iterator_pair : pair< typename basic_string<T>::const_iterator, typename basic_string<T>::const_iterator >{
	typedef typename basic_string<T>::const_iterator iterator_type;
	typedef pair<iterator_type, iterator_type>       base_type;
	
	iterator_type begin()const{ return first;  }
	iterator_type end  ()const{ return second; }
	bool          empty()const{ return second == first; }
	size_t        size ()const{ return second - first;  }

	basic_string_iterator_pair(){}
	basic_string_iterator_pair(iterator_type f, iterator_type s):base_type(f, s){}
	basic_string_iterator_pair(const basic_string<T>& c):base_type(c.begin(), c.end()){}
};

//typedef iterator_pair< string::const_iterator>	 string_iterator_pair;
//typedef iterator_pair<wstring::const_iterator>	wstring_iterator_pair;
typedef basic_string_iterator_pair<char   >  string_iterator_pair;
typedef basic_string_iterator_pair<wchar_t> wstring_iterator_pair;

template<class T>
basic_string_iterator_pair<T> eat_white(basic_string_iterator_pair<T> str){
	if(str.empty())
		return str;

	std::locale loc;

	basic_string_iterator_pair<T>::iterator_type i0 = str.begin();
	basic_string_iterator_pair<T>::iterator_type i1 = str.end();
	while(i0 != i1 && std::isspace(*(i1-1), loc))
		i1--;
	while(i0 != i1 && std::isspace(*i0, loc))
		i0++;
	return basic_string_iterator_pair<T>(i0, i1);
}

int     to_int   ( string_iterator_pair str);
real_t  to_real  ( string_iterator_pair str);
string  to_string( string_iterator_pair str);
wstring to_string(wstring_iterator_pair str);

inline bool operator==(string_iterator_pair str, const char* comp){
	return (str.end() - str.begin() == strlen(comp)) && equal(str.begin(), str.end(), comp);
}
inline bool operator!=(string_iterator_pair str, const char* comp){
	return !(str == comp);
}
inline bool operator==(wstring_iterator_pair str, const wchar_t* comp){
	return (str.end() - str.begin() == wcslen(comp)) && equal(str.begin(), str.end(), comp);
}
inline bool operator!=(wstring_iterator_pair str, const wchar_t* comp){
	return !(str == comp);
}

/* for_each コンテナ版
 */
template<class T, class F>
void for_each(const T& c, F f){
	std::for_each(c.begin(), c.end(), f);
}

/** name (fixed-length, null-terminated)
	ネットワーク通信でオブジェクトの名前を指定する際に使用
 **/
template<class T, size_t n>
struct FixedStr{
	static const int length = n;
	T buf[n];

	FixedStr(){
		fill(buf, buf+n, T(0));
	}
	FixedStr(const T* str){
		int i;
		for(i = 0; i < n-1 && str[i]; i++){
			buf[i] = str[i];
		}
		buf[i] = T('\0');
	}
	FixedStr(const basic_string<T>& str){
		if(str.size() < n){
			std::copy(str.begin(), str.end(), buf);
			buf[str.size()] = T('\0');
		}
		else{
			std::copy(str.begin(), str.begin() + n-1, buf);
			buf[n-1] = T('\0');
		}
	}
	
	size_t  capacity()const{
		return FixedStr<n>::length;
	}
	size_t	size()const{
		size_t i = 0;
		while(i < n && buf[i] != '\0')
			i++;
		return i;
	}
	operator const T*()const{
		return buf;
	}
};

template<size_t n>
inline bool operator==(const FixedStr<char, n>& str, const char* comp){
	return strlen(comp) < n && strcmp(str.buf, comp) == 0;
}
template<size_t n>
inline bool operator==(const FixedStr<wchar_t, n>& str, const wchar_t* comp){
	return wcslen(comp) < n && wcscmp(str.buf, comp) == 0;
}

typedef FixedStr<char, 32>  str32_t;
typedef FixedStr<char, 256>	str256_t;

// remove elements from array
template<class T, class E>
void RemoveFromArray(vector< shared_ptr<T> >& arr, E elem){
	for(vector< shared_ptr<T> >::iterator it = arr.begin(); it != arr.end(); ){
		if(it->get() == elem)
			 it = arr.erase(it);
		else it++;
	}
}
template<class T, class E>
void RemoveFromArray(vector<T>& arr, E elem){
	for(vector<T>::iterator it = arr.begin(); it != arr.end(); ){
		if(*it == elem)
			it = arr.erase(it);
		else it++;
	}
}

/** exceptions **/
struct Exception{
};
struct Failure          : virtual Exception{};	//< 操作は正しいが結果がエラー
struct InvalidOperation : virtual Exception{};	//< 引数，パスなどが不正
struct FileError        : virtual Exception{};  //< ファイル操作まわりのエラー
struct NetworkError     : virtual Exception{};	//< ネットワーク操作まわりのエラー
struct SyntaxError      : virtual Exception{};  //< 構文解析のエラー

struct XMLException   : virtual Exception{};
struct XMLFailure     : XMLException, Failure    {};
struct XMLFileError   : XMLException, FileError  {};
struct XMLSyntaxError : XMLException, SyntaxError{};

struct SceneException : virtual Exception{};
struct SceneFailure         : SceneException, Failure         {};
struct SceneInvalidOperation: SceneException, InvalidOperation{};
struct SceneFileError       : SceneException, FileError       {};
struct SceneNetworkError    : SceneException, NetworkError    {};

struct CalcException  : virtual Exception{};
struct CalcSyntaxError : CalcException, SyntaxError{};

struct AdaptorException : virtual Exception{};
struct AdaptorFailure : AdaptorException, Failure{};

}
