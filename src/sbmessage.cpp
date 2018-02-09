#include <sbmessage.h>

#include <sstream>
#include <fstream>
using namespace std;

#include <boost/date_time.hpp>
using namespace boost;
using namespace boost::gregorian;

namespace Scenebuilder{;

int					Message::verboseLevel = Message::Level::Normal;
ostream*			Message::os           = &cout;
MessageCallback*	Message::callback     = 0;
CriticalSection		Message::cs;

void Message::SetStream(ostream* _os){
	cs.Enter();
	os = _os;
	cs.Leave();
}
void Message::SetCallback(MessageCallback* cb){
	cs.Enter();
	callback = cb;
	cs.Leave();
}

void Message::SetVerboseLevel(int level){
	cs.Enter();
	verboseLevel = level;
	cs.Leave();
}

int Message::GetVerboseLevel(){
	return verboseLevel;
}

void Message::Out(const char* str, ...){
	if(verboseLevel < Level::Normal)
		return;

	cs.Enter();
	char buf[1024];

	va_list	args;	
	va_start(args, str);
	vsprintf(buf, str, args);
	sprintf(buf + strlen(buf), "\n");
	va_end(args);

	if(os)
		(*os) << buf;
	if(callback)
		callback->OnMessage(Level::Normal, buf);
	cs.Leave();
}

void Message::Error(const char* str, ...){
	if(verboseLevel < Level::Error)
		return;

	cs.Enter();
	char buf[1024];

	va_list	args;	
	va_start(args, str);
	vsprintf(buf, str, args);
	sprintf(buf + strlen(buf), "\n");
	va_end(args);

	if(os)
		(*os) << buf;
	if(callback)
		callback->OnMessage(Level::Error, buf);
	cs.Leave();
}

void Message::Extra(const char* str, ...){
	if(verboseLevel < Level::Extra)
		return;

	cs.Enter();
	char buf[1024];

	va_list	args;	
	va_start(args, str);
	vsprintf(buf, str, args);
	sprintf(buf + strlen(buf), "\n");
	va_end(args);

	if(os)
		(*os) << buf;
	if(callback)
		callback->OnMessage(Level::Extra, buf);
	cs.Leave();
}

}
