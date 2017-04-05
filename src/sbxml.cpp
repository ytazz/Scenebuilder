#include <sbxml.h>
#include <sbtokenizer.h>

// expatを静的リンクで使用
#define XML_STATIC
#include <expat.h>

namespace Scenebuilder{;

//-------------------------------------------------------------------------------------------------

XMLNode::XMLNode(XML* _xml, const string& tn){
	xml  = _xml;
	name = tn;
}

string XMLNode::GetName(){
	return GetAttr("name", false);
}

void XMLNode::SetName(string name){
	SetAttr("name", name);
}

XMLNode* XMLNode::GetNode(int idx, bool _throw){
	if(0 <= idx && idx < (int)children.size())
		return xml->GetNode(children[idx]);
	if(_throw)
		throw XMLFailure();
	return 0;
}

XMLNode* XMLNode::GetNode(const string& n, int idx, bool _throw){
	int cnt = 0;
	for(uint i = 0; i < children.size(); i++){
		XMLNode* child = xml->GetNode(children[i]);
		if(child->name == n){
			if(cnt == idx)
				return child;
			cnt++;
		}
	}
	if(_throw)
		throw XMLFailure();
	return 0;
}

string XMLNode::GetAttr(const string& n, bool _throw){
	Attrs::iterator it;
	for(it = attrs.begin(); it != attrs.end(); it++){
		if(it->first == n)
			break;
	}

	if(it == attrs.end()){
		if(_throw)
			throw XMLFailure();
		return string();
	}
	return it->second;
}

void XMLNode::SetAttr(const string& n, const string& v){
	Attrs::iterator it;
	for(it = attrs.begin(); it != attrs.end(); it++){
		if(it->first == n)
			break;
	}
	if(it == attrs.end()){
		attrs.push_back(make_pair(n, v));	
	}
	else it->second = v;
}

string XMLNode::GetContents(){
	return contents;
}

void XMLNode::SetContents(const string& str){
	contents = str;
}

void XMLNode::Print(ostream& os, bool full, int depth){
	os << string(depth, ' ');
	os << '<' << name;

	if(full){
		for(XMLNode::Attrs::iterator it = attrs.begin(); it != attrs.end(); it++){
			os << ' ' << it->first << '=' << '\"' << it->second << '\"';
		}
	}

	if(children.empty()){
		os << "/>" << endl;
	}
	else{
		os << '>' << endl;
		for(uint i = 0; i < children.size(); i++)
			xml->GetNode(children[i])->Print(os, full, depth+1);
		os << string(depth, ' ');
		os << "</" << name << '>' << endl;
	}
}

bool XMLNode::Parse(string& valstr, string_iterator_pair path){
	string_iterator_pair token, name, attr;
		
	// パスを'/'で分解
	Tokenizer tok(path, "/", false);
	token = tok.GetToken();
	tok.Next();
		
	// つづきがある場合
	if(!tok.IsEnd()){
		XMLNode* node = GetNode(to_string(token), 0, false);
		if(!node)
			return false;
		return node->Parse(valstr, tok.GetToken());
	}

	// '/'がなかった場合：'.'で分解
	tok.Set(token, ".", false);
	name = tok.GetToken();
	tok.Next();
	if(!tok.IsEnd())
		attr = tok.GetToken();

	XMLNode* node = 0;
	if(!name.empty()){
		node = GetNode(to_string(name), 0, false);
		if(!node)
			return false;
	}
	else node = this;

	if(tok.IsEnd())
		 valstr = node->GetContents();
	else valstr = node->GetAttr(to_string(attr), false);
	return true;
}

//-------------------------------------------------------------------------------------------------

void StartHandler(void *userData, const XML_Char *name, const XML_Char **atts){
	XML* xml = (XML*)userData;
	
	// create XML node
	xml->curId = xml->CreateNode(name, xml->curId);
	XMLNode* node = xml->GetNode(xml->curId);
	node->xml = xml;

	// store attributes
	while(*atts){
		node->attrs.push_back(make_pair(atts[0], atts[1]));
		atts += 2;
	}
}

void EndHandler(void *userData, const XML_Char *name){
	XML* xml = (XML*)userData;
	xml->curId = xml->GetParent(xml->curId);
}

void TextHandler(void *userData, const XML_Char* s, int len){
	XML* xml = (XML*)userData;
	XMLNode* node = xml->GetNode(xml->curId);

	node->contents += string(s, s+len);;
}

//-------------------------------------------------------------------------------------------------

int XML::CreateNode(const string& name, int parId){
	XMLNode* node = new XMLNode(this, name);
	int id = AddNode(node);
	if(parId != -1)
		AddChild(parId, id);
	return id;
}

void XML::Load(const string& filename){
	// load file content to buffer
	string contents;
	ifstream ifs;
	ifs.open(filename, ifstream::in | ifstream::binary);

	if(!ifs.is_open())
		throw XMLFileError();

	ifs.seekg(0, ifs.end);
	int len = (int)ifs.tellg();
	ifs.seekg(0, ifs.beg);

	contents.resize(len);
	ifs.read(&contents[0], len);
	
	ifs.close();
	
	Clear();
	curId = -1;
	
	// parse xml
	XML_Parser parser = XML_ParserCreate(0);
	XML_SetElementHandler(parser, StartHandler, EndHandler);
	XML_SetCharacterDataHandler(parser, TextHandler);
	XML_SetUserData(parser, this);
	int ret = XML_Parse(parser, contents.c_str(), (int)contents.size(), 1);
	XML_ParserFree(parser);

	if(ret == XML_STATUS_ERROR)
		throw XMLSyntaxError();
}

void XML::Save(ostream& os, int id, int depth){
	XMLNode* node = GetNode(id);

	os << string(depth, ' ');
	os << '<' << node->name;
	for(XMLNode::Attrs::iterator it = node->attrs.begin(); it != node->attrs.end(); it++){
		os << ' ' << it->first << '=' << '\"' << it->second << '\"';
	}
	
	if(node->contents.empty() && node->children.empty()){
		os << "/>" << endl;
	}
	else{
		os << '>' << endl;
		// タグ以外の文字列は字下げしない
		if(!node->contents.empty())
			os << node->contents << endl;

		for(uint i = 0; i < node->children.size(); i++){
			Save(os, node->children[i], depth+1);
		}

		os << string(depth, ' ');
		os << "</" << node->name << '>' << endl;
	}
}

void XML::Save(const string& filename){
	ofstream ofs;
	ofs.open(filename);
	if(!ofs.is_open())
		throw XMLFileError();

	// ヘッダ書き出し
	ofs << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl;

	int depth = 0;
	Save(ofs, GetRoot(), depth);
}

}
