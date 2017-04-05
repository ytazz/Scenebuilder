#include "sbopenal.h"
#include <sbmessage.h>

#include <alut.h>

#define AUTO(T, X, EXP) T X = static_cast<T>(EXP)

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorOpenAL::ALObject::ALObject(){
	id = -1;
}

bool AdaptorOpenAL::ALObject::CheckError(int err){
	if(err == AL_NO_ERROR)
		return true;
	if(err == AL_INVALID_VALUE)
		Message::Error("AL value out of range");
	if(err == AL_INVALID_ENUM)
		Message::Error("AL invalid parameter");
	if(err == AL_INVALID_NAME)
		Message::Error("AL invalie name");
	if(err == AL_INVALID_OPERATION)
		Message::Error("AL no current context");
	return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorOpenAL::Buffer::Buffer(){

}

AdaptorOpenAL::Buffer::~Buffer(){
	if(alIsBuffer(id)){
		alGetError();
		alDeleteBuffers(1, &id);
		int err = alGetError();
		if(err == AL_INVALID_OPERATION)
			Message::Error("buffer is still in use");
		if(err == AL_INVALID_NAME)
			Message::Error("buffer name invalid");
	}
}

bool AdaptorOpenAL::Buffer::Load(string _filename){
	filename = _filename;
	alutGetError();
	id = alutCreateBufferFromFile(filename.c_str());
	if(id == AL_NONE){
		int err = alutGetError();
		if(err == ALUT_ERROR_OUT_OF_MEMORY)
			Message::Error("ALUT out of memory");
		if(err == ALUT_ERROR_INVALID_OPERATION)
			Message::Error("ALUT not initialized");
		if(err == ALUT_ERROR_NO_CURRENT_CONTEXT)
			Message::Error("ALUT no current context");
		if(err == ALUT_ERROR_AL_ERROR_ON_ENTRY)
			Message::Error("ALUT detected AL error on entry");
		if(err == ALUT_ERROR_ALC_ERROR_ON_ENTRY)
			Message::Error("ALUT detected ALC error on entry");
		if(err == ALUT_ERROR_GEN_BUFFERS)
			Message::Error("ALUT error generating buffer");
		if(err == ALUT_ERROR_BUFFER_DATA)
			Message::Error("ALUT error passing data to buffer");
		if(err == ALUT_ERROR_IO_ERROR)
			Message::Error("ALUT I/O error");
		if(err == ALUT_ERROR_UNSUPPORTED_FILE_TYPE)
			Message::Error("ALUT unsupported file type");
		if(err == ALUT_ERROR_UNSUPPORTED_FILE_SUBTYPE)
			Message::Error("ALUT unsupported file subtype");
		if(err == ALUT_ERROR_CORRUPT_OR_TRUNCATED_DATA)
			Message::Error("ALUT sound data corrupt");

		return false;
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorOpenAL::Source::Source(){
	buffer = 0;
	loop   = false;
	gain   = 1.0f;

	alGetError();
	alGenSources(1, &id);
	int err = alGetError();
	if(err == AL_OUT_OF_MEMORY)
		Message::Error("AL out of memory");
	if(err == AL_INVALID_VALUE)
		Message::Error("AL pointer invalid");
	if(err == AL_INVALID_OPERATION)
		Message::Error("AL context invalid");
}

AdaptorOpenAL::Source::~Source(){
	if(alIsSource(id)){
		alGetError();
		alDeleteSources(1, &id);
		int err = alGetError();
		if(err == AL_INVALID_NAME)
			Message::Error("AL source name invalid");
		if(err == AL_INVALID_OPERATION)
			Message::Error("AL no current context");
	}
}

bool AdaptorOpenAL::Source::Attach(Buffer* buf){
	buffer = buf;
	alGetError();
	alSourcei(id, AL_BUFFER, buf->id);
	return CheckError(alGetError());
}

bool AdaptorOpenAL::Source::EnableLoop(bool on){
	loop = on;
	alGetError();
	alSourcei(id, AL_LOOPING, (loop ? AL_TRUE : AL_FALSE));
	return CheckError(alGetError());
}

bool AdaptorOpenAL::Source::SetPosition(const Vec3f& p){
	pos = p;
	alGetError();
	alSourcefv(id, AL_POSITION, pos);
	return CheckError(alGetError());
}

bool AdaptorOpenAL::Source::SetVelocity(const Vec3f& v){
	vel = v;
	alGetError();
	alSourcefv(id, AL_VELOCITY, vel);
	return CheckError(alGetError());
}

bool AdaptorOpenAL::Source::SetDirection(const Vec3f& d){
	dir = d;
	alGetError();
	alSourcefv(id, AL_DIRECTION, dir);
	return CheckError(alGetError());
}

bool AdaptorOpenAL::Source::SetGain(float g){
	gain = g;
	alGetError();
	alSourcef(id, AL_GAIN, gain);
	return CheckError(alGetError());
}

bool AdaptorOpenAL::Source::Play(){
	alGetError();
	alSourcePlay(id);
	return CheckError(alGetError());
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorOpenAL::Listener::Listener(){
	active = false;
	up     = Vec3f(0.0f, 1.0f,  0.0f);
	front  = Vec3f(0.0f, 0.0f, -1.0f);
	gain   = 1.0f;
}

AdaptorOpenAL::Listener::~Listener(){

}

bool AdaptorOpenAL::Listener::SetPosition(const Vec3f& p){
	pos = p;
	if(active){
		alGetError();
		alListenerfv(AL_POSITION, pos);
		return CheckError(alGetError());
	}
	return true;
}

bool AdaptorOpenAL::Listener::SetVelocity(const Vec3f& v){
	vel = v;
	if(active){
		alGetError();
		alListenerfv(AL_VELOCITY, vel);
		return CheckError(alGetError());
	}
	return true;
}

bool AdaptorOpenAL::Listener::SetOrientation(const Vec3f& f, const Vec3f& u){
	front = f;
	up    = u;
	Matrix3f R;
	R.col(1) =  up;
	R.col(2) = -front;
	R.col(0) = R.col(1) % R.col(2);
	ori.FromMatrix(R.trans());
	if(active){
		alGetError();
		alListenerfv(AL_ORIENTATION, front);
		return CheckError(alGetError());		
	}
	return true;
}

bool AdaptorOpenAL::Listener::SetOrientation(const Quaternionf& q){
	ori   = q;
	front = ori * Vec3f(0.0f, 0.0f, -1.0f);
	up    = ori * Vec3f(0.0f, 1.0f,  0.0f);
	if(active){
		alGetError();
		alListenerfv(AL_ORIENTATION, front);
		return CheckError(alGetError());
	}
	return true;
}

bool AdaptorOpenAL::Listener::SetGain(float g){
	gain = g;
	if(active){
		alGetError();
		alListenerf(AL_GAIN, gain);
		return CheckError(alGetError());
	}
	return true;
}

bool AdaptorOpenAL::Listener::Activate(){
	active = true;
	bool ok = true;
	ok &= SetPosition(pos);
	ok &= SetVelocity(vel);
	ok &= SetOrientation(ori);
	ok &= SetGain(gain);
	return ok;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

AdaptorOpenAL::AdaptorOpenAL(){
	ready = false;
}

AdaptorOpenAL::~AdaptorOpenAL(){
	Close();
}

bool AdaptorOpenAL::Init(){
	alGetError();
	if(alutInit(0, 0) == AL_FALSE){
		int err = alGetError();
		if(err == ALUT_ERROR_INVALID_VALUE)
			Message::Error("ALUT invalid argument");
		if(err == ALUT_ERROR_INVALID_OPERATION)
			Message::Error("ALUT already initialized");
		if(err == ALUT_ERROR_OPEN_DEVICE)
			Message::Error("ALUT error opening device");
		if(err == ALUT_ERROR_CREATE_CONTEXT)
			Message::Error("ALUT error creating ALC context");
		if(err == ALUT_ERROR_MAKE_CONTEXT_CURRENT)
			Message::Error("ALUT could not change current ALC context");
		return false;
	}

	// デフォルトのサウンドファイルをロード T.B.D.

	ready = true;
	return true;
}

bool AdaptorOpenAL::Close(){
	if(!ready)
		return false;

	buffers  .clear();
	sources  .clear();
	listeners.clear();
	
	alGetError();
	if(alutExit() == AL_FALSE){
		int err = alGetError();
		if(err == ALUT_ERROR_INVALID_OPERATION)
			Message::Error("ALUT has not been initialized");
		if(err == ALUT_ERROR_NO_CURRENT_CONTEXT)
			Message::Error("ALUT no current context");
		if(err == ALUT_ERROR_AL_ERROR_ON_ENTRY)
			Message::Error("ALUT error on entry");
		if(err == ALUT_ERROR_ALC_ERROR_ON_ENTRY)
			Message::Error("ALUT error on entry");
		if(err == ALUT_ERROR_CLOSE_DEVICE)
			Message::Error("ALUT error closing ALC device");
		if(err == ALUT_ERROR_MAKE_CONTEXT_CURRENT)
			Message::Error("ALUT could not release current ALC context");
		if(err == ALUT_ERROR_DESTROY_CONTEXT)
			Message::Error("ALUT error destroying context");
		return false;
	}
	return true;
}

AdaptorOpenAL::Buffer* AdaptorOpenAL::CreateBuffer(){
	if(!ready)
		return 0;
	Buffer* buf = new Buffer();
	buf->adaptor = this;
	buffers.push_back(buf);
	return buf;
}

AdaptorOpenAL::Source* AdaptorOpenAL::CreateSource(){
	if(!ready)
		return 0;
	Source* src = new Source();
	src->adaptor = this;
	sources.push_back(src);
	return src;
}

AdaptorOpenAL::Listener* AdaptorOpenAL::CreateListener(){
	if(!ready)
		return 0;
	Listener* lis = new Listener();
	lis->adaptor = this;
	listeners.push_back(lis);
	return lis;
}

int	AdaptorOpenAL::CreateObject(int id){
	return SupportState::Ignored;
}

void AdaptorOpenAL::DeleteObject(int id){

}

void AdaptorOpenAL::SyncObjectProperty(int id, bool download, int cat){
	if(!ready)
		return;
	if(!IsSupported(id))
		return;
	
}

}
