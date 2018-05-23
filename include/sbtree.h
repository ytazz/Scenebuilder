#pragma once

#include <sbtypes.h>
#include <sbbuffer.h>

#include <deque>
using namespace std;

namespace Scenebuilder{;

/** ツリー構造のロケーションを指し示すアドレス
	- 先頭文字列が""の場合は絶対パス
 **/
struct Address : std::deque<string>{
	typedef std::deque<string> base_type;

	///
	char Delim(){ return '/'; }
	/// 
	void FromString(const string& str);
	///
	string ToString();

	Address(){}
	Address(const string& str){ FromString(str); }
};

/** ツリー構造のインタフェース
	- パスを指定してノードを検索する機能
 **/
struct TreeBase{
protected:
	int FindRecurse(Address& addr, int id);
	
public:
	/// クリア
	virtual void Clear() = 0;
	/// ルートノードのIDを返す
	virtual int	 GetRoot() = 0;
	/// 親ノードのIDを返す
	virtual int	 GetParent(int id) = 0;
	/// 子ノードのIDを配列に格納する
	virtual void GetChildren(int id, vector<int>& children) = 0;
	/// 子ノードを追加する
	virtual void AddChild(int parId, int childId) = 0;
	/// 子ノードを削除する
	virtual void RemoveChild(int parId, int childId) = 0;
	/// 名前を取得する
	virtual string GetName(int id) = 0;
	/// 名前を設定する
	virtual void   SetName(int id, string name) = 0;
	/// ノードのテキスト出力
	virtual void PrintNode(int id, ostream& os);

	/** @brief パスを検索する
		@param	addr	アドレス
		@param	id		addrが相対パスの場合に起点となるオブジェクトのID．省略するとルートオブジェクトとみなされる
	 **/
	int	Find(const Address& addr, int id = -1);

	/** @brief パスを取得する
	    @param addr		アドレス
		@param id0		起点となるオブジェクト
		@param id1		アドレスを取得するオブジェクト
	 **/
	void GetAddress(Address& addr, int id0, int id1);

	/** @brief テキスト出力
	 **/
	void PrintRecursive(int id, ostream& os, int depth = 0);
	void Print(ostream& os);

};

/** ツリー構造の実体 **/

class Tree;

/** ツリーノード **/
struct TreeNode : UTRefCount{
	int				parId;
	vector<int>		children;

	/// name
	virtual string GetName() = 0;
	virtual void   SetName(string name) = 0;
	
	TreeNode();
	virtual ~TreeNode(){}
};

class Tree : public TreeBase{
protected:
	typedef std::vector< UTRef<TreeNode> >		Nodes;
	Nodes	nodes;

public:
	int	            AddNode(TreeNode* n);
	TreeNode*       GetNode(int id);
	
public:	/// TreeBase functions

	virtual void    Clear      ();
	virtual int	    GetRoot    ();
	virtual int	    GetParent  (int id);
	virtual void    GetChildren(int id, vector<int>& children);
	virtual string  GetName    (int id);
	virtual void    SetName    (int id, string name);
	virtual void    AddChild   (int parId, int childId);
	virtual void    RemoveChild(int parId, int childId);
};

}
