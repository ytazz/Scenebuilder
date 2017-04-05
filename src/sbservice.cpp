#include <sbservice.h>
using namespace std;

#include <boost/algorithm/string/replace.hpp>
using namespace boost;
using namespace boost::algorithm;

#include <windows.h>

namespace Scenebuilder{;

string Service::binPath(".\\");
vector< UTRef<Service> >	Service::services;
	
void Service::SetServicePath(string path){
	binPath = path;
	replace_all(binPath, "/", "\\");
	if(binPath.back() != '\\')
		binPath += '\\';
}

bool Service::StartProcess(string exePath){
	// サービスを子プロセスとして起動する
	// - 標準入出力をパイプでリダイレクト
	// - 子プロセスのコンソールウィンドウは出さない

	HANDLE hOutputReadTmp = 0, hOutputRead = 0, hOutputWrite = 0;
	HANDLE hInputWriteTmp = 0, hInputRead = 0, hInputWrite = 0;

	try{
		// Set up the security attributes struct.
		SECURITY_ATTRIBUTES sa;
		sa.nLength= sizeof(SECURITY_ATTRIBUTES);
		sa.lpSecurityDescriptor = NULL;
		sa.bInheritHandle = TRUE;

		// Create the child output pipe.
		if (!CreatePipe(&hOutputReadTmp, &hOutputWrite, &sa, 0)) throw;
		// Create the child input pipe.
		if (!CreatePipe(&hInputRead, &hInputWriteTmp, &sa, 0)) throw;

		// Create new output read handle and the input write handles. Set
		// the Properties to FALSE. Otherwise, the child inherits the
		// properties and, as a result, non-closeable handles to the pipes
		// are created.
		if (!DuplicateHandle(GetCurrentProcess(), hOutputReadTmp, GetCurrentProcess(), &hOutputRead, 0, FALSE, DUPLICATE_SAME_ACCESS))
			throw;
		if (!DuplicateHandle(GetCurrentProcess(),hInputWriteTmp, GetCurrentProcess(), &hInputWrite, 0, FALSE, DUPLICATE_SAME_ACCESS))
			throw;

		// Close inheritable copies of the handles you do not want to be inherited.
		if (!CloseHandle(hOutputReadTmp)) throw;
		hOutputReadTmp = 0;
		if (!CloseHandle(hInputWriteTmp)) throw;
		hInputWriteTmp = 0;

		STARTUPINFO	startInfo;
		ZeroMemory(&startInfo, sizeof(STARTUPINFO));
		startInfo.cb = sizeof(STARTUPINFO);
		//startInfo.dwFlags = STARTF_USESTDHANDLES;
		//startInfo.hStdOutput = hOutputWrite;
		//startInfo.hStdInput  = hInputRead;
		//startInfo.hStdError  = hOutputWrite;

		PROCESS_INFORMATION	procInfo;

		BOOL ok = CreateProcess(
			exePath.c_str(),	//< path to executable
			NULL,				//< command line
			NULL,				//< lpProcessAttributes
			NULL,				//< lpThreadAttributes
			FALSE,				//< bInheritHandles
			CREATE_NO_WINDOW | NORMAL_PRIORITY_CLASS,	//< dwCreationFlags
			NULL,				//< lpEnvironment
			NULL,				//< lpCurrentDirectory
			&startInfo,			//< lpStartupInfo
			&procInfo			//< lpProcessInfo
			);

		if(!ok)
			throw 0;

		HANDLE h = procInfo.hProcess;
		CloseHandle(procInfo.hThread);
	
		// Close pipe handles (do not continue to modify the parent).
		// You need to make sure that no handles to the write end of the
		// output pipe are maintained in this process or else the pipe will
		// not close when the child process exits and the ReadFile will hang.
		if (!CloseHandle(hOutputWrite)) throw 0;
		hOutputWrite = 0;
		if (!CloseHandle(hInputRead )) throw 0;
		hInputRead = 0;
		
	}
	catch(...){
		if(hOutputReadTmp)	CloseHandle(hOutputReadTmp);
		if(hOutputRead)		CloseHandle(hOutputRead);
		if(hOutputWrite)	CloseHandle(hOutputWrite);
		if(hInputWriteTmp)	CloseHandle(hInputWriteTmp);
		if(hInputRead)		CloseHandle(hInputRead);
		if(hInputWrite)		CloseHandle(hInputWrite);
		return false;
	}

	hOutputPipe = hOutputRead;
	hInputPipe = hInputWrite;
	return true;

}

void Service::Send(string command){
	DWORD nWritten;
	if(!WriteFile(hInputPipe, command.c_str(), (DWORD)command.size(), &nWritten, 0)){
		int hoge = 0;	
	}
}

void Service::Open(string name){
	ss.str("");
	ss << "open " << name << endl;
	Send(ss.str());
}

void Service::Pause(){
	Send("pause\n");
}

void Service::Resume(){
	Send("resume\n");
}

//-------------------------------------------------------------------------------------------------

SimulatorService* SimulatorService::Start(){
	UTRef<SimulatorService> s = new SimulatorService;
	if(s->StartProcess(binPath + "sbsim.exe")){
		services.push_back(s);
		return s;
	}
	return 0;
}

//-------------------------------------------------------------------------------------------------

ViewerService* ViewerService::Start(){
	UTRef<ViewerService> s = new ViewerService;
	if(s->StartProcess(binPath + "sbview.exe")){
		services.push_back(s);
		return s;
	}
	return 0;
}

//-------------------------------------------------------------------------------------------------

ServerService* ServerService::Start(){
	UTRef<ServerService> s = new ServerService;
	if(s->StartProcess(binPath + "sbserve.exe")){
		services.push_back(s);
		return s;
	}
	return 0;
}

void ServerService::Create(string name){
	ss.str("");
	ss << "create " << name << endl;
	Send(ss.str());
}

}
