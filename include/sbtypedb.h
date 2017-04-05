#pragma once

#include <sbtypes.h>
#include <sbxml.h>
#include <sbcalc.h>
#include <sbtokenizer.h>

namespace Scenebuilder{;

class  TypeDB;
struct Property;

/** 複合型 **/
struct MultiType{
	string	str;
	bool	boolean;
	int		integer;
	real_t	real;
	vvec_t	vvec;
	vec2_t	vec2;
	vec3_t	vec3;
	vec4_t	vec4;
	quat_t	quat;
	mat3_t	mat3;
public:
	void Reset(){
		boolean = false;
		integer = 0;
		real = 0.0;
		vvec.clear();
		vec2.clear();
		vec3.clear();
		vec4.clear();
		quat = quat_t();
		mat3 = mat3_t();
	}

	MultiType(){ Reset(); }
	MultiType(string val)		{ Reset(); str     = val; }
	MultiType(bool val)			{ Reset(); boolean = val; }
	MultiType(int val)			{ Reset(); integer = val; }
	MultiType(real_t val)		{ Reset(); real    = val; }
	MultiType(const vvec_t& val){ Reset(); vvec    = val; }
	MultiType(vec2_t val)		{ Reset(); vec2    = val; }
	MultiType(vec3_t val)		{ Reset(); vec3    = val; }
	MultiType(vec4_t val)		{ Reset(); vec4    = val; }
	MultiType(quat_t val)		{ Reset(); quat    = val; }
	MultiType(const mat3_t& val){ Reset(); mat3    = val; }
};

/** 属性の変更頻度レベル
 */
struct AttrCategory{
	enum{
		Param  = 0x01,		///< 一度設定したら変更なし
		State  = 0x02,		///< 動的に変わる
		All    = Param | State,
	};
};

/** 属性情報

 */
struct AttrInfo : UTRefCount{
	int			id;			///< id
	string		name;		///< name
	int			primId;		///< primitive type
	uint		len;		///< string length, only if StringProp::id
	uint		offset;		///< byte offset to this attribute
	MultiType	def;		///< default value
	int			cat;		///< category
	int			dim;		///< physical dimension

public:
	void	ToXML     (XML* xml, int typeNodeId);
	void	Set       (MultiType& val, Property* prop);
	bool	FromString(string_iterator_pair  str, Property* prop);
	void	ToString  (ostream&              os , Property* prop);
	void	FromBinary(const byte*&          buf, Property* prop);
	void	ToBinary  (byte*&                buf, Property* prop);
	uint	CalcBytes ();

	AttrInfo(int _id, const string& _name, int _primId, uint _len, uint _
		, MultiType def, int _cat, int _dim);
};

/** prototype of functions that creates a new instance of Property **/
typedef void (*PropertyConstructor)(Property*);

/** Composite type information

 **/
struct TypeInfo : UTRefCount{
	TypeDB*		db;					///< TypeDB
	int			id;					///< ID
	int			baseId;				///< ID of base type (if any)
	size_t		szProp;				///< size of propety
	string		name;				///< name

	PropertyConstructor	constructor;	///< function that news a property instance of this type

	vector< UTRef<AttrInfo> >	attrs;	///< attributes

public:
	TypeInfo*	AddAttr(const string& name, uint prim, uint len, uint offset, MultiType def = MultiType(), int cat = AttrCategory::Param, int dim = Dimension::None);
	AttrInfo*	GetAttr(int   attrId);
	AttrInfo*	GetAttr(const string& attrName);

	/// convert typeinfo into xml
	void	ToXML(XML* xml);

	/// set default value
	void    SetDefault(Property* prop, int cat);
	/// serialize into text stream
	void    FromString(Tokenizer& tok,           Property* prop, int cat);
	void	FromString(string_iterator_pair str, Property* prop, int cat);
	/// deserialize from text stream
	void	ToString(ostream& os,        Property* prop, int cat);
	/// serialize into binary array
	void	FromBinary(const byte*& buf, Property* prop, int cat);
	/// deserialize from binary array
	void	ToBinary(byte*& buf,         Property* prop, int cat);
	/// calculate number of bytes in binary format
	uint	CalcBytes(int cat);

	TypeInfo(TypeDB* _db, int _id, const string& _name, size_t _sz, PropertyConstructor _constructor, int _baseId);
};

class XML;

/** 型情報データベース

 **/
class TypeDB : public UTRefCount{
protected:
	vector< UTRef<TypeInfo> >	types;
	
public:
	/// register primitive types
	void		Register();

	int			AddType(const string& name, size_t sz = 0, PropertyConstructor constructor = 0, int baseId = -1);
	TypeInfo*	GetType(int typeId);
	TypeInfo*	GetType(const string& typeName);

	/// returns true if (typeId == baseId) holds or [typeId] derives from [baseId]
	bool		KindOf(int typeId, int baseId);

	/// stores ids of derived types of type [baseId] into array [derived]
	void		EnumDerivedTypes(int baseId, vector<int>& derived);

	/// convert from/to XML tree
	void		FromXML(XML* xml);
	void		ToXML  (XML* xml);

	/// save/load in XML format
	void		Save(string filename);
	void		Load(string filename);

	TypeDB();
};


/** collection of macros **/

/** byte offset of a member variable **/
#define OFFSET(CLS, MEM) ( offsetof(CLS, MEM) )

/** property **/
struct Property{

};

/** 基本型情報 **/
struct Primitive{
	enum{
		String,
		Bool,
		Int,
		Real,
		Vec2,
		Vec3,
		Vec4,
		Quat,
		Mat3,
		Path,
	};
};

}
