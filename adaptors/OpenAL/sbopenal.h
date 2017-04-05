#pragma once

#include <sbadaptor.h>

/** OpenAL adaptor
	- 物理イベントに応じてサウンド再生
 **/

namespace Scenebuilder{;

class AdaptorOpenAL : public Adaptor{
public:
	class ALObject : public UTRefCount{
	public:
		AdaptorOpenAL*  adaptor;
		uint            id;			///< handle to OpenAL object

		bool CheckError(int err);

		ALObject();
	};

	class Buffer : public ALObject{
	public:
		string filename;	///< path to sound file
	
	public:
		/// load sound from file
		bool Load(string filename);

		 Buffer();
		~Buffer();
	};

	class Source : public ALObject{
	public:
		Buffer*		buffer;	///< buffer attached to this source
		bool		loop;
		Vec3f		pos;
		Quaternionf	dir;
		Vec3f		vel;
		float		gain;

	public:
		/// attach buffer
		bool Attach(Buffer* buf);

		/// enable or disable looping
		bool EnableLoop(bool on = true);

		/// set position
		bool SetPosition(const Vec3f& p);

		/// set velocity
		bool SetVelocity(const Vec3f& v);

		/// set direction
		bool SetDirection(const Vec3f& d);

		/** set gain
			@param g gain
			1.0 default, 0.0 silence
		 */
		bool SetGain(float g);

		/// play source
		bool Play();

		 Source();
		~Source();
	};

	class Listener : public ALObject{
	public:
		bool		active;
		Vec3f		pos;
		Vec3f		front;
		Vec3f		up;
		Quaternionf	ori;
		Vec3f		vel;
		float		gain;

	public:
		/// set position
		bool SetPosition(const Vec3f& p);

		/// set velocity
		bool SetVelocity(const Vec3f& v);

		/// set direction
		bool SetOrientation(const Vec3f& front, const Vec3f& up);
		bool SetOrientation(const Quaternionf& ori);

		/// set gain
		bool SetGain(float g);

		/// activate this listener
		bool Activate();

		 Listener();
		~Listener();
	};

protected:
	typedef vector< UTRef<Buffer  > >	Buffers;
	typedef vector< UTRef<Source  > >	Sources;
	typedef vector< UTRef<Listener> >	Listeners;

	bool		ready;
	Buffers		buffers;
	Sources		sources;
	Listeners	listeners;
	
public:
	
public:
	bool	Init();
	bool	Close();
	
	Buffer*			CreateBuffer  ();
	Source*			CreateSource  ();
	Listener*		CreateListener();
	
	virtual int		CreateObject(int id);
	virtual void	DeleteObject(int id);
	virtual void	SyncObjectProperty(int id, bool download, int cat);

	AdaptorOpenAL();
	~AdaptorOpenAL();
};

}
