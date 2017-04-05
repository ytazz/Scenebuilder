#ifndef SB_SERVICE_H
#define SB_SERVICE_H

#include <sbtypes.h>

#include <vector>
#include <sstream>

/** Scenebuilderのサービスを子プロセスとして起動し，やりとりするための支援機能

 **/

namespace Scenebuilder{;

class Service : public UTRefCount{
protected:
	static string binPath;
	static std::vector< UTRef<Service> >	services;
	
	void*	hProcess;
	void*	hInputPipe;
	void*	hOutputPipe;

	std::stringstream	ss;

	void	Send(string command);

public:
	/// set path to service executable files
	static void	SetServicePath(string path);

	bool StartProcess(string exeName);

	void	Open(string name);
	void	Pause();
	void	Resume();

};

class SimulatorService : public Service{
public:
	static SimulatorService* Start();

};

class ViewerService : public Service{
public:
	static ViewerService* Start();

};

class ServerService : public Service{
public:
	static ServerService* Start();

	void Create(string name);

};


}

#endif
