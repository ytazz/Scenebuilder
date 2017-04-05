﻿#include <sbpath.h>
#include <windows.h>

namespace Scenebuilder{;

Path::Path(const string& s):string(s){
	ReplaceDelim();
}

Path& Path::operator=(const char* p){
	string::operator=(p);
	ReplaceDelim();
	return *this;
}

Path& Path::operator=(const string& p){
	string::operator=(p);
	ReplaceDelim();
	return *this;
}

void Path::ReplaceDelim(){
	// windowsに合わせる
	int n = (int)strlen(Delims());
	for(int i = 0; i < n; i++)
		::replace(begin(), end(), Delims()[i], Delim());
	
	// 末尾の区切り文字を除く
	if(!empty() && at(size()-1) == Delim())
		erase(size()-1, 1);
}

string Path::Ext() const{
	size_t extPos = find_last_of('.');
	if(extPos == string::npos)
		return string();
	extPos++;
	if(extPos == size())
		return string();
	return substr(extPos, size() - extPos);
}

string Path::File() const{
	if(IsDir())
		return string();

	size_t delimPos = find_last_of(Delim());

	size_t extPos = find_last_of('.');
	// パス区切りが無い場合は先頭から
	if(delimPos == string::npos)
		 delimPos = 0;
	else delimPos++;
	// 拡張子が無い場合は末尾まで
	if(extPos == string::npos)
		extPos = size();

	return substr(delimPos, extPos - delimPos);
}

string Path::Dir() const{
	// ディレクトリパスならそのまま
	if(IsDir())
		return *this;
	
	size_t delimPos = find_last_of(Delim());
	// パス区切りが無いなら空文字列
	if(delimPos == string::npos)
		return string();
	
	return substr(0, delimPos);
}

string Path::Drive() const{
	if(size() >= 2 && at(1) == ':')
		return substr(0,1);
	return string();
}

bool Path::IsFile() const{
	return !IsDir();
}

bool Path::IsDir() const{
	int att = GetFileAttributes(c_str());
	return (att != -1 && (att & FILE_ATTRIBUTE_DIRECTORY) != 0);
}

bool Path::IsAbsolute() const{
	return !Drive().empty();
}

bool Path::IsRelative() const{
	return !IsAbsolute();
}

Path Path::Current(){
	Path ret;
	size_t sz = GetCurrentDirectory(0, 0);
	ret.resize(sz+1);
	GetCurrentDirectory((DWORD)sz, &ret[0]);
	return ret;
}

Path operator+(const Path& p0, const Path& p1){
	string ret = p0;
	if(!ret.empty())
		ret += Path::Delim();
	ret.append(p1);
	return ret;
}

}
