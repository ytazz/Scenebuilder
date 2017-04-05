#pragma once

#include <sbtree.h>
#include <sbconverter.h>

#include <map>
using namespace std;

namespace Scenebuilder{;

class XML;

class XMLNode : public TreeNode{
public:
	string	name;	///< タグ名

	typedef vector<pair<string, string>>	Attrs;
	Attrs	attrs;

	string  contents;

	XML*    xml;
	
public:
	/// TreeNode functions
	/// XMLのタグ名ではなくname属性値を返す
	virtual  string GetName();
	virtual  void   SetName(string name);

	/// idx番目の子ノード
	XMLNode* GetNode    (int idx = 0, bool _throw = true);
	/// 指定したタグ名のidx番目の子ノード
	XMLNode* GetNode    (const string& n, int idx = 0, bool _throw = true);
	string   GetAttr    (const string& n, bool _throw = true);
	void     SetAttr    (const string& n, const string& v);
	string   GetContents();
	void     SetContents(const string& str);

	void	Print(ostream& os, bool full, int depth);

	bool    Parse(string& valstr, string_iterator_pair path);

	template<class T>
	bool Get(T& val, const string& path){
		string valstr;
		if(!Parse(valstr, path))
			return false;
		if(valstr.empty())
			return false;
		return Converter::FromString(valstr, val);
	}

	XMLNode(XML* _xml, const string& tn);
};

class XML : public Tree{
public:
	int       curId;
	Converter converter;

public:
	/// create new node
	int CreateNode(const string& name, int parId = -1);
	
	/// get xml node specified by id
	XMLNode* GetNode(int id){ return (XMLNode*)Tree::GetNode(id); }
	/// get root node
	XMLNode* GetRootNode(){ return GetNode(GetRoot()); }

	/// load from file
	void Load(const string& filename);

	/// save to file
	void Save(ostream& os, int id, int depth);
	void Save(const string& filename);

	/** 指定したパスの属性あるいはコンテンツ文字列を指定した型に変換して取得
		- ノードは/区切り，属性は.区切り  e.g. node0/node1.att
	 */
	template<class T>
	bool Get(T& val, const string& path){
		return GetNode(GetRoot())->Get(val, path);
	}

	/// ルートノード直下のnameノードのi番目のtypestrノードの文字列をval[i]にlexical_castして取得
	template<class T>
	void GetArray(T* val, uint sz, string name, string typestr){
		try{
			for(uint i = 0; i < sz; i++){
				Converter::FromString(GetNode(GetRoot())->GetNode(name)->GetNode(typestr, i)->GetContents(), val[i]);
			}
		}
		catch(...){}
	}

};

}
