#include <sbshared.h>

#ifdef _WIN32
# include <windows.h>
#else
# include <sys/mman.h>
# include <sys/stat.h>
# include <fcntl.h>
# include <unistd.h>
#endif

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
#ifdef _WIN32
	hFile = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, (DWORD)_sz, _name);

	if(!hFile || GetLastError() == ERROR_ALREADY_EXISTS)
		throw FileError();
#else
	// append '/' in front of shared memory name
	str n[256];
	sprintf(n, "/%s", _name);
	hFile = shm_open(n, O_RDWR | O_CREAT, 0644); 

	ftruncate(hFile, sz); 
#endif
	name = _name;
}

void SharedMemory::Open(const char* _name){
	if(hFile)
		Close();

#ifdef _WIN32
	hFile = OpenFileMappingA(FILE_MAP_WRITE, FALSE, _name);
#else
	// append '/' in front of shared memory name
	str n[256];
	sprintf(n, "/%s", _name);
	hFile = shm_open(n, O_RDWR, 0644);
#endif
	if(!hFile)
		throw FileError();
	
	name = _name;
}

bool SharedMemory::IsOpen(){
	return !!hFile;
}

void SharedMemory::Close(){
#ifdef _WIN32
	if(pView)
		UnmapViewOfFile(pView);

	if(hFile)
		CloseHandle(hFile);
#else
	close(hFile);
#endif

	hFile = NULL;
	pView = NULL;
	name  = NULL;
}

byte* SharedMemory::Get(size_t offset, size_t sz){
	if(!hFile)
		return NULL;

#ifdef _WIN32
	if(pView)
		UnmapViewOfFile(pView);

	pView = (byte*)MapViewOfFile(hFile, FILE_MAP_WRITE, 0, (DWORD)offset, (DWORD)sz);
#else
	pView = (byte*) mmap(NULL, sz, PROT_READ|PROT_WRITE, MAP_SHARED, hFile, 0); 
#endif

	return pView;
}

}
