﻿#include <sbshared.h>
#include <windows.h>

namespace Scenebuilder{;

SharedMemory::SharedMemory(){
	hFile = NULL;
	pView = NULL;
	name  = NULL;
}

SharedMemory::~SharedMemory(){
	Close();
}

void SharedMemory::Create(const char* _name, size_t _sz){
	hFile = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, (DWORD)_sz, _name);

	if(!hFile || GetLastError() == ERROR_ALREADY_EXISTS)
		throw FileError();
	
	name = _name;
}

void SharedMemory::Open(const char* _name){
	if(hFile)
		Close();

	hFile = OpenFileMappingA(FILE_MAP_WRITE, FALSE, _name);
	if(!hFile)
		throw FileError();
	
	name = _name;
}

bool SharedMemory::IsOpen(){
	return !!hFile;
}

void SharedMemory::Close(){
	if(pView)
		UnmapViewOfFile(pView);

	if(hFile)
		CloseHandle(hFile);

	hFile = NULL;
	pView = NULL;
	name  = NULL;
}

byte* SharedMemory::Get(size_t offset, size_t sz){
	if(!hFile)
		return NULL;

	if(pView)
		UnmapViewOfFile(pView);

	pView = (byte*)MapViewOfFile(hFile, FILE_MAP_WRITE, 0, (DWORD)offset, (DWORD)sz);

	return pView;
}

}