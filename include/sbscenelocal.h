#pragma once

#include <sbtypes.h>
#include <sbtree.h>
#include <sbscene.h>
#include <sbpath.h>

#include <map>

/**	SceneLocal
	ローカルメモリ空間に置かれるシーン

**/

namespace Scenebuilder{;

class SceneLocal : public SceneBase{
public:
	struct Link{
		int     id;
		string  name;

		Link(int _id, const string& _name):id(_id), name(_name){}
	};
	struct Object : UTRefCount{
		bool        alive;
		bool		original;
		int			type;
		string		name;

		int				parId;
		vector<int>		children;
		vector<Link>	links;
		vector<byte>	prop;

		Object(){
			alive    = true;
			original = true;
			type     = SceneObjectProp::id;
			parId    = -1;
		}
	};

	struct Location{
		int		id;
		Path	path;

		Location(int _id, string _path):id(_id), path(_path){}
	};
	
	vector< UTRef<Object> >	objArray;
	vector< Location >		locArray;

public:
	void	Create();
	void    GetLivenessRecursively(int id, vector<byte>& alive);

public:
	/// TreeBase virtual functions
	virtual void		Clear      ();
	virtual int			GetRoot    ();
	virtual int			GetParent  (int id);
	virtual void		GetChildren(int id, vector<int>& children);
	virtual string		GetName    (int id);
	virtual void        SetName    (int id, string name);
	virtual void		AddChild   (int parId, int childId);
	virtual void		RemoveChild(int parId, int childId);

	/// SceneBase virtual functions
	virtual int			CreateObject    (int type, const string& name, TypeDB* typedb);
	virtual void		DeleteObject    (int id);
	virtual int			GetObjectType   (int id);
	virtual void		AddLink         (int srcId, int destId, const string& name);
	virtual void		RemoveLink      (int srcId, int destId);
	virtual int			GetLink         (int id, const string& name);
	virtual void		GetLinks        (int id, vector<int>& links, vector<string>& names);
	virtual Property*	GetProperty     (int id);
	virtual void		GetLivenessArray(Address& addr, vector<byte>& alive);
	virtual void		SetLocation     (int id, const string& path);
	virtual string		GetLocation     (int id);

	SceneLocal();
};

}
