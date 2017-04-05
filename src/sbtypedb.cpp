#include <sbtypedb.h>
#include <sbscene.h>
#include <sbbuilder.h>

namespace Scenebuilder{;

//-------------------------------------------------------------------------------------------------

AttrInfo::AttrInfo(int _id, const string& _name, int _primId, uint _len, uint _offset, MultiType _def, int _cat, int _dim)
	:id(_id), name(_name), primId(_primId), len(_len), offset(_offset), def(_def), cat(_cat), dim(_dim){

}
void AttrInfo::ToXML(XML* xml, int typeNodeId){
	int nodeId = xml->CreateNode(name, typeNodeId);
	XMLNode* node = xml->GetNode(nodeId);

	stringstream ss;
	ss << id;
	node->SetAttr("id", ss.str());
}

void AttrInfo::Set(MultiType& val, Property* prop){
	byte* pos = (byte*)prop + offset;
	switch(primId){
	case Primitive::String : strncpy((char*)pos, val.str.c_str(), len-1);	break;
	case Primitive::Bool   : *(bool*)  pos = val.boolean; break;
	case Primitive::Int    : *(int*)   pos = val.integer; break;
	case Primitive::Real   : *(real_t*)pos = val.real;    break;
	case Primitive::Vec2   : *(vec2_t*)pos = val.vec2;    break;
	case Primitive::Vec3   : *(vec3_t*)pos = val.vec3;    break;
	case Primitive::Vec4   : *(vec4_t*)pos = val.vec4;    break;
	case Primitive::Quat   : *(quat_t*)pos = val.quat;    break;
	case Primitive::Mat3   : *(mat3_t*)pos = val.mat3;    break;
	}
}

bool AttrInfo::FromString(string_iterator_pair str, Property* prop){
	byte* pos = (byte*)prop + offset;
	switch(primId){
	case Primitive::String : return Converter::FromString(str,  (char   *)pos, len);
	case Primitive::Bool   : return Converter::FromString(str, *(bool   *)pos     );
	case Primitive::Int    : return Converter::FromString(str, *(int    *)pos     );
	case Primitive::Real   : return Converter::FromString(str, *(real_t *)pos, dim);
	case Primitive::Vec2   : return Converter::FromString(str, *(vec2_t *)pos, dim);
	case Primitive::Vec3   : return Converter::FromString(str, *(vec3_t *)pos, dim);
	case Primitive::Vec4   : return Converter::FromString(str, *(vec4_t *)pos, dim);
	case Primitive::Quat   : return Converter::FromString(str, *(quat_t *)pos, dim);
	case Primitive::Mat3   : return Converter::FromString(str, *(mat3_t *)pos, dim);
	}
	return false;
}

void AttrInfo::ToString(ostream& os, Property* prop){
	byte* pos = (byte*)prop + offset;
	switch(primId){
	case Primitive::String : Converter::ToString(os,  (char  *)pos); break;
	case Primitive::Bool   : Converter::ToString(os, *(bool  *)pos); break;
	case Primitive::Int    : Converter::ToString(os, *(int   *)pos); break;
	case Primitive::Real   : Converter::ToString(os, *(real_t*)pos); break;
	case Primitive::Vec2   : Converter::ToString(os, *(vec2_t*)pos); break;
	case Primitive::Vec3   : Converter::ToString(os, *(vec3_t*)pos); break;
	case Primitive::Vec4   : Converter::ToString(os, *(vec4_t*)pos); break;
	case Primitive::Quat   : Converter::ToString(os, *(quat_t*)pos); break;
	case Primitive::Mat3   : Converter::ToString(os, *(mat3_t*)pos); break;
	}
}

void AttrInfo::FromBinary(const byte*& buf, Property* prop){
	byte* pos = (byte*)prop + offset;
	switch(primId){
	case Primitive::String : strncpy((char*)pos, (const char*)buf, len-1); break;
	case Primitive::Bool   : *(bool  *)pos = *(bool  *)buf; break;
	case Primitive::Int    : *(int   *)pos = *(int   *)buf; break;
	case Primitive::Real   : *(real_t*)pos = *(real_t*)buf; break;
	case Primitive::Vec2   : *(vec2_t*)pos = *(vec2_t*)buf; break;
	case Primitive::Vec3   : *(vec3_t*)pos = *(vec3_t*)buf; break;
	case Primitive::Vec4   : *(vec4_t*)pos = *(vec4_t*)buf; break;
	case Primitive::Quat   : *(quat_t*)pos = *(quat_t*)buf; break;
	case Primitive::Mat3   : *(mat3_t*)pos = *(mat3_t*)buf; break;
	}
}

void AttrInfo::ToBinary(byte*& buf, Property* prop){
	byte* pos = (byte*)prop + offset;
	switch(primId){
	case Primitive::String : Converter::ToBinary(buf, (const char*)pos, len); break;
	case Primitive::Bool   : Converter::ToBinary(buf, *(bool  *)pos); break;
	case Primitive::Int    : Converter::ToBinary(buf, *(int   *)pos); break;
	case Primitive::Real   : Converter::ToBinary(buf, *(real_t*)pos); break;
	case Primitive::Vec2   : Converter::ToBinary(buf, *(vec2_t*)pos); break;
	case Primitive::Vec3   : Converter::ToBinary(buf, *(vec3_t*)pos); break;
	case Primitive::Vec4   : Converter::ToBinary(buf, *(vec4_t*)pos); break;
	case Primitive::Quat   : Converter::ToBinary(buf, *(quat_t*)pos); break;
	case Primitive::Mat3   : Converter::ToBinary(buf, *(mat3_t*)pos); break;
	}
}

uint AttrInfo::CalcBytes(){
	switch(primId){
	case Primitive::String : return len;
	case Primitive::Bool   : return sizeof(bool);
	case Primitive::Int    : return sizeof(int);
	case Primitive::Real   : return sizeof(real_t);
	case Primitive::Vec2   : return sizeof(vec2_t);
	case Primitive::Vec3   : return sizeof(vec3_t);
	case Primitive::Vec4   : return sizeof(vec4_t);
	case Primitive::Quat   : return sizeof(quat_t);
	case Primitive::Mat3   : return sizeof(mat3_t);
	}
	return 0;
}

//-------------------------------------------------------------------------------------------------

AttrInfo* TypeInfo::GetAttr(int attrId){
	for(uint i = 0; i < attrs.size(); i++){
		AttrInfo* attr = attrs[i];
		if(attr->id == attrId)
			return attr;
	}
	return 0;
}

AttrInfo* TypeInfo::GetAttr(const string& attrName){
	for(uint i = 0; i < attrs.size(); i++){
		AttrInfo* attr = attrs[i];
		if(attr->name == attrName)
			return attr;
	}
	return 0;
}

TypeInfo* TypeInfo::AddAttr(const string& name, uint prim, uint len, uint offset, MultiType def, int cat, int dim){
	int id = (int)attrs.size();
	attrs.push_back(new AttrInfo(id, name, prim, len, offset, def, cat, dim));
	return this;
}

void TypeInfo::ToXML(XML* xml){
	int nodeId = xml->CreateNode(name, xml->GetRoot());
	
	XMLNode* node = xml->GetNode(nodeId);

	stringstream ss;
	node->SetAttr("id",     (ss.str(""), ss << id    , ss.str()) );
	node->SetAttr("baseId", (ss.str(""), ss << baseId, ss.str()) );
	node->SetAttr("szProp", (ss.str(""), ss << szProp, ss.str()) );

	for(uint i = 0; i < attrs.size(); i++)
		attrs[i]->ToXML(xml, nodeId);
}

void TypeInfo::SetDefault(Property* prop, int cat){
	if(baseId != -1)
		db->GetType(baseId)->SetDefault(prop, cat);

	for(uint i = 0; i < attrs.size(); i++){
		AttrInfo* attr = attrs[i];

		if(attr->cat & cat){
			attr->Set(attr->def, prop);
		}
	}
}

void TypeInfo::FromString(Tokenizer& tok, Property* prop, int cat){
	if(baseId != -1)
		db->GetType(baseId)->FromString(tok, prop, cat);

	for(uint i = 0; i < attrs.size(); i++){
		AttrInfo* attr = attrs[i];

		if(tok.IsEnd())
			break;
		if(attr->cat & cat){
			attr->FromString(tok.GetToken(), prop);
			tok.Next();
		}
	}
}

void TypeInfo::FromString(string_iterator_pair str, Property* prop, int cat){
	Tokenizer tok(str, ";", false);
	FromString(tok, prop, cat);
}

void TypeInfo::ToString(ostream& os, Property* prop, int cat){
	if(baseId != -1)
		db->GetType(baseId)->ToString(os, prop, cat);

	for(uint i = 0; i < attrs.size(); i++){
		AttrInfo* attr = attrs[i];

		if(attr->cat & cat){
			attr->ToString(os, prop);
			os << ";";
		}
	}
}

void TypeInfo::FromBinary(const byte*& buf, Property* prop, int cat){
	if(baseId != -1)
		db->GetType(baseId)->FromBinary(buf, prop, cat);

	for(uint i = 0; i < attrs.size(); i++){
		AttrInfo* attr = attrs[i];

		if(attr->cat & cat){
			attr->FromBinary(buf, prop);
			buf += attr->CalcBytes();
		}
	}
}

void TypeInfo::ToBinary(byte*& buf, Property* prop, int cat){
	if(baseId != -1)
		db->GetType(baseId)->ToBinary(buf, prop, cat);

	for(uint i = 0; i < attrs.size(); i++){
		AttrInfo* attr = attrs[i];

		if(attr->cat & cat)
			attr->ToBinary(buf, prop);
	}
}

uint TypeInfo::CalcBytes(int cat){
	uint sz = 0;
	if(baseId != -1)
		sz = db->GetType(baseId)->CalcBytes(cat);

	for(uint i = 0; i < attrs.size(); i++){
		AttrInfo* attr = attrs[i];
		if(attr->cat & cat)
			sz += attr->CalcBytes();
	}
	return sz;
}

TypeInfo::TypeInfo(TypeDB* _db, int _id, const string& _name, size_t _sz, PropertyConstructor _constructor, int _baseId):
 db(_db), id(_id), name(_name), szProp(_sz), constructor(_constructor), baseId(_baseId){

}

//-------------------------------------------------------------------------------------------------

TypeDB::TypeDB(){
}

void TypeDB::Register(){
	/*
	// 基本型の登録
	StringProp::Register(this);
	BoolProp::Register(this);
	IntProp::Register(this);
	RealProp::Register(this);
	Vec2Prop::Register(this);
	Vec3Prop::Register(this);
	Vec4Prop::Register(this);
	QuatProp::Register(this);
	Mat3Prop::Register(this);
	*/
	// SceneObject
	SceneBase::Register(this);

	// Builder
	Builder::Register(this);

}

int TypeDB::AddType(const string& name, size_t sz, PropertyConstructor constructor, int baseId){
	int id = (int)types.size();
	types.push_back(new TypeInfo(this, id, name, sz, constructor, baseId));
	return id;
}

TypeInfo* TypeDB::GetType(int typeId){
	for(uint i = 0; i < types.size(); i++){
		TypeInfo* type = types[i];
		if(type->id == typeId)
			return type;
	}
	return 0;
}

TypeInfo* TypeDB::GetType(const string& typeName){
	for(uint i = 0; i < types.size(); i++){
		TypeInfo* type = types[i];
		if(type->name == typeName)
			return type;
	}
	return 0;
}

bool TypeDB::KindOf(int typeId, int baseId){
	TypeInfo* type = GetType(typeId);
	TypeInfo* base = GetType(baseId);
	// must be valid type
	if(!type || !base)
		return false;
	// if two types are the same
	if(type == base)
		return true;
	// if [base] is the direct parent of [type]
	if(type->baseId == baseId)
		return true;
	// test recursively
	return KindOf(type->baseId, baseId);
}

void TypeDB::EnumDerivedTypes(int baseId, vector<int>& derived){
	derived.clear();
	for(uint i = 0; i < types.size(); i++){
		TypeInfo* type = types[i];
		if(KindOf(type->id, baseId))
			derived.push_back(type->id);
	}
}

void TypeDB::FromXML(XML* xml){
	int rootId = xml->GetRoot();
	vector<int> children;
	xml->GetChildren(rootId, children);

	for(uint i = 0; i < children.size(); i++){
		XMLNode* node = xml->GetNode(children[i]);
		string name = node->GetName();
		//if(name != "type")
		//	continue;

	}
}

void TypeDB::ToXML(XML* xml){
	for(uint i = 0; i < types.size(); i++)
		types[i]->ToXML(xml);
}

void TypeDB::Save(string filename){
	XML xml;
	ToXML(&xml);
	xml.Save(filename);
}

void TypeDB::Load(string filename){
	XML xml;
	xml.Load(filename);
	FromXML(&xml);
}

}
