#include <sbtree.h>
#include <sbconverter.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
using namespace boost::algorithm;
using namespace boost;

namespace Scenebuilder{;

//-------------------------------------------------------------------------------------------------

void Address::FromString(const string& path){
	clear();
	split((base_type&)*this, path, is_any_of("/"), token_compress_on);

	// '/'や'hoge/moge/'の場合にできる余計な空トークンを除外
	if(size() > 1 && back() == "")
		pop_back();
}
	
string Address::ToString(){
	string path;
	const char delim = '/';
	for(iterator it = begin(); it != end(); ){
		path += *it++;
		if(it != end())
			path += delim;
	}
	return path;
}

//-------------------------------------------------------------------------------------------------

int TreeBase::Find(const Address& addr, int id){
	Address tmp = addr;
	return FindRecurse(tmp, id);
}

void TreeBase::GetAddress(Address& addr, int id0, int id1){
	if(id0 == id1)
		return;

	int par = GetParent(id1);
	if(par == -1)
		throw InvalidOperation();
	
	addr.push_front(GetName(id1));
	GetAddress(addr, id0, par);
}

int	TreeBase::FindRecurse(Address& addr, int id){
	if(id == -1)
		id = GetRoot();

	if(addr.empty())
		return id;

	// if first token is "", this means path starts with delimiter, thus absolute path
	if(addr[0] == ""){
		addr.pop_front();
		return FindRecurse(addr, GetRoot());
	}
		
	string token = addr.front();
	addr.pop_front();

	if(token == ".")
		return Find(addr, id);
	if(token == ".."){
		int parId = GetParent(id);
		if(parId == -1)
			throw InvalidOperation();

		return FindRecurse(addr, parId);
	}

	// find in child namespaces
	vector<int> children;
	GetChildren(id, children);
	for(uint i = 0; i < children.size(); i++){
		int childId = children[i];
		if(GetName(childId) == token){
			return FindRecurse(addr, childId);
		}
	}

	// not found
	throw InvalidOperation();
}

void TreeBase::PrintNode(int id, ostream& os){
	os << GetName(id) << endl;
}

void TreeBase::PrintRecursive(int id, ostream& os, int depth){
	const int tab=2;
	for(int i = 0; i < tab*depth; i++)
		os << ' ';

	PrintNode(id, os);

	vector<int> children;
	GetChildren(id, children);
	for(uint i = 0; i < children.size(); i++){
		PrintRecursive(children[i], os, depth+1);
	}
}

void TreeBase::Print(ostream& os){
	PrintRecursive(GetRoot(), os, 0);
}

//-------------------------------------------------------------------------------------------------

TreeNode::TreeNode(){
	parId = -1;
}

//-------------------------------------------------------------------------------------------------

int	Tree::AddNode(TreeNode* n){
	int id = (int)nodes.size();
	nodes.push_back(n);
	return id;
}

TreeNode* Tree::GetNode(int id){
	if(0 <= id && id < (int)nodes.size())
		return nodes[id];
	throw InvalidOperation();
}

void Tree::Clear(){
	nodes.clear();
}

int	Tree::GetRoot(){
	return 0;
}

int	Tree::GetParent(int id){
	return GetNode(id)->parId;
}

void Tree::GetChildren(int id, vector<int>& children){
	children = GetNode(id)->children;
}

string Tree::GetName(int id){
	return GetNode(id)->GetName();
}

void Tree::SetName(int id, string name){
	GetNode(id)->SetName(name);
}

void Tree::AddChild(int parId, int childId){
	TreeNode* par = GetNode(parId);
	TreeNode* child = GetNode(childId);
	// 重複やサイクルチェックはしない
	par->children.push_back(childId);
	child->parId = parId;
}

void Tree::RemoveChild(int parId, int childId){
	TreeNode* par = GetNode(parId);
	TreeNode* child = GetNode(childId);
	par->children.erase(find(par->children.begin(), par->children.end(), childId));
	child->parId = -1;
}

}
