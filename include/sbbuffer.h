#pragma once

#include <sbtypes.h>

#include <vector>
#include <algorithm>
using namespace std;

namespace Scenebuilder{;

/** シリアライズ用のバッファ
 **/
struct Buffer : vector<byte>{
	struct OverflowException{};

	uint idx;

	/// constructor
	Buffer(){ idx = 0; }

	/// reset
	void Reset(uint sz = 0){
		resize(sz);
		idx = 0;
	}

	/// returns head address
	byte* Head(){ return &(*this)[idx]; }
	const byte*	Head()const{ return &(*this)[idx]; }

	/// moves head
	void Seek(uint offset){
		if(idx + offset > size())
			throw InvalidOperation();
		idx += offset;
	}
	void SeekFromHead(uint offset){
		if(offset > (int)size())
			throw InvalidOperation();
		idx = offset;
	}
	void SeekFromTail(uint offset){
		if(size() - offset < 0)
			throw InvalidOperation();
		idx = (uint)size() - offset;
	}

	template<typename T>
	void Read(T* var, uint sz){
		Read((byte*)var, sizeof(T)*sz);
	}

	template<typename T>
	void Write(const T* var, uint sz){
		Write((const byte*)var, sizeof(T)*sz);
	}

	void Read(byte* var, uint sz){
		//if(idx + sz > size())
		//	throw InvalidOperation();
		uint _sz = std::min(sz, (uint)(size()-idx));
		copy(Head(), Head() + _sz, var); 
		idx += _sz;
	}

	void Write(const byte* var, uint sz){
		if(idx + sz > size())
			resize(idx + sz);
		copy(var, var + sz, Head());
		idx += sz;
	}
	
	/// 
	template<typename T>
	void Read(T& var){
		Read((byte*)&var, sizeof(T));
	}

	///
	template<typename T>
	void Write(const T& var){
		Write((const byte*)&var, sizeof(T));
	}

};

template<typename T>
inline Buffer& operator >>(Buffer& buf, T& var){
	buf.Read(var);
	return buf;
}

inline Buffer& operator >>(Buffer& buf, string& var){
	// store null-terminated string to var
	var.clear();
	char ch;
	while(true){
		try{
			buf >> ch;
		}
		catch(Exception&){ break; }
		if(ch == 0)
			break;
		var.push_back(ch);
	}
	return buf;
}

template<typename T>
inline Buffer& operator <<(Buffer& buf, const T& var){
	buf.Write(var);
	return buf;
}

inline Buffer& operator <<(Buffer& buf, const string& var){
	// store var as null-terminated string
	for(string::const_iterator it = var.begin(); it != var.end(); it++){
		buf << *it;
	}
	buf << '\0';
	return buf;
}

}
