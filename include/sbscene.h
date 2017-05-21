#pragma once

#include <sbtypes.h>
#include <sbtree.h>
#include <sbtypedb.h>

#include <vector>
#include <set>

using namespace std;
using namespace Spr;

/** Common Scene Structure

	- Scenebuilderにおけるシーン表現の仕様

	- 形式を規定するものであって機能は規定しない．
	- 種々の機能はアダプタによって提供される．

	- 構造はオブジェクト間の親子関係から成るツリー構造に，任意のオブジェクト間の横断的リンクを加えたもの．

	- オブジェクトは名前を持つ．名前は同一階層内で重複してはならない．
	- オブジェクトには型があり，型に応じたプロパティを持つ．

	- シーンは初期状態で唯一のルートオブジェクトを持つ．
	- ルートオブジェクトの型はNamespaceである．

	- オブジェクトの作成直後，その親オブジェクトは未定義であり，この状態ではまだシーンに加えられたことにはならない．
	- 一度親子関係を定義すると，それを後から変更することはできない．

	- あるオブジェクトを削除すると，その下に連なるすべてのオブジェクト（子孫オブジェクト）が削除され，
	　同時にそれらのオブジェクトを端点とするリンクもすべて削除される．

 **/

namespace Scenebuilder{;

/** affine transformation
	T = <r, R, M, s>
	r : translation
	R : rotation		
	M : mirror
	s : scaling

	given p,
	p' = s R M p + r

	T1 = <r1, R1, M1, s1>
	T2 = <r2, R2, M2, s2>
	T1 T2 p
	   = s1 R1 M1 (s2 R2 M2 p + r2) + r1
	   = (s1 s2) R1 M1 R2 M2 p + (s1 R1 M1 r2 + r1)

 **/
struct Transform{
	vec3_t	trn;		///< relative translation w.r.t. parent spatial object
	quat_t	rot;		///< relative rotation w.r.t. parent spatial object
	bool	mirrorx;	///< mirror transform in x-direction
	bool	mirrory;	///< mirror transform in y-direction
	bool	mirrorz;	///< mirror transform in z-direction
	real_t	scale;		///< scaling factor

	Transform Inv();

	Transform(){
		mirrorx = false;
		mirrory = false;
		mirrorz = false;
		scale   = 1.0;
	}
};

Transform operator*(const Transform& t0, const Transform& t1);

/** SceneObject
	- シーンの構成要素
 **/
struct SceneObjectProp : Property{
	static int id;
	bool	alive;	///< liveness
	
	static string GetName(){ return "sceneobject"; }
	static void Construct(Property* p){ new(p) SceneObjectProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(SceneObjectProp), &Construct);
	}

	SceneObjectProp(){
		alive = true;
	}
};

/** SpatialObject
 **/
struct SpatialObjectProp : SceneObjectProp, Transform{
	static int id;

	static string GetName(){ return "spatialobject"; }
	static void Construct(Property* p){ new(p) SpatialObjectProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(SpatialObjectProp), &Construct, SceneObjectProp::id);
		db->GetType(id)
			->AddAttr("trn",     Primitive::Vec3, 1, OFFSET(SpatialObjectProp, trn),	 vec3_t(), AttrCategory::State, Dimension::L)
			->AddAttr("rot",     Primitive::Quat, 1, OFFSET(SpatialObjectProp, rot),	 quat_t(), AttrCategory::State, Dimension::R)
			->AddAttr("mirrorx", Primitive::Bool, 1, OFFSET(SpatialObjectProp, mirrorx), false)
			->AddAttr("mirrory", Primitive::Bool, 1, OFFSET(SpatialObjectProp, mirrory), false)
			->AddAttr("mirrorz", Primitive::Bool, 1, OFFSET(SpatialObjectProp, mirrorz), false)
			->AddAttr("scale",   Primitive::Real, 1, OFFSET(SpatialObjectProp, scale),	 1.0);
	}

	SpatialObjectProp(){
	}
};

/** Namespace
	- グルーピングのためのオブジェクト
 **/
struct NamespaceProp : SpatialObjectProp{
	static int id;

	static string GetName(){ return "namespace"; }
	static void Construct(Property* p){ new(p) NamespaceProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(NamespaceProp), &Construct, SpatialObjectProp::id);
	}
};

/** Body
	- 剛体
	- parent
	  - Namespace
 **/
struct BodyProp : SpatialObjectProp{
	static int id;
	real_t	mass;		///< mass
	mat3_t	inertia;	///< inertia matrix
	vec3_t	center;		///< center of mass
	bool	dynamical;	///< obeys law of physics
	bool	stationary; ///< stationary object
	bool	auto_mass;	///< compute mass, inertia and CoM from geometries and physical materials
	bool    auto_tree;  ///< create joint coordinate tree using this body as a root
	vec3_t	vel;		///< velocity in world coordinate
	vec3_t	angvel;		///< ang-velocity in world coordinate
	vec3_t  acc;
	vec3_t  angacc;
	
	static string GetName(){ return "body"; }
	static void Construct(Property* p){ new(p) BodyProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(BodyProp), &Construct, SpatialObjectProp::id);
		db->GetType(id)
			->AddAttr("mass"      ,	Primitive::Real, 1, OFFSET(BodyProp, mass      ), 1.0           , AttrCategory::Param, Dimension::M)
			->AddAttr("inertia"   ,	Primitive::Mat3, 1, OFFSET(BodyProp, inertia   ), mat3_t::Unit(), AttrCategory::Param, Dimension::MLL)
			->AddAttr("center"    ,	Primitive::Vec3, 1, OFFSET(BodyProp, center    ), vec3_t()      , AttrCategory::Param, Dimension::L)
			->AddAttr("dynamical" ,	Primitive::Bool, 1, OFFSET(BodyProp, dynamical ), true          , AttrCategory::Param)
			->AddAttr("stationary", Primitive::Bool, 1, OFFSET(BodyProp, stationary), false         , AttrCategory::Param)
			->AddAttr("auto_mass" ,	Primitive::Bool, 1, OFFSET(BodyProp, auto_mass ), false         , AttrCategory::Param)
			->AddAttr("auto_tree" ,	Primitive::Bool, 1, OFFSET(BodyProp, auto_tree ), false         , AttrCategory::Param)
			->AddAttr("vel"       , Primitive::Vec3, 1, OFFSET(BodyProp, vel       ), vec3_t()      , AttrCategory::State, Dimension::L_T)
			->AddAttr("angvel"    , Primitive::Vec3, 1, OFFSET(BodyProp, angvel    ), vec3_t()      , AttrCategory::State, Dimension::R_T)
			->AddAttr("acc"       , Primitive::Vec3, 1, OFFSET(BodyProp, acc       ), vec3_t()      , AttrCategory::State, Dimension::L_TT)
			->AddAttr("angacc"    , Primitive::Vec3, 1, OFFSET(BodyProp, angacc    ), vec3_t()      , AttrCategory::State, Dimension::R_TT);
	}
};

/** Connector
	- コネクタ．剛体同士を関節でつなぐために使用する
	- parent
	  - Body
	- children
	  - none
	- links
	  - none
 **/
struct ConnectorProp : SpatialObjectProp{
	static int id;
	
	static string GetName(){ return "connector"; }
	static void Construct(Property* p){ new(p) ConnectorProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(ConnectorProp), &Construct, SpatialObjectProp::id);
	}
};

/** PhysicalMaterial
	- 物性．
	- 形状オブジェクトからリンクすることで物性を与える
 **/
struct PhysicalMaterialProp : SceneObjectProp{
	static int id;
	real_t  density;          ///< density
	real_t	cor;              ///< coefficient of restitution
	real_t	static_friction;  ///< static friction coefficient
	real_t  dynamic_friction; ///< dynamic friction coefficient
	real_t  spring;           ///< contact spring
	real_t  damper;           ///< contact damper

	static string GetName(){ return "pmat"; }
	static void Construct(Property* p){ new(p) PhysicalMaterialProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(PhysicalMaterialProp), &Construct, SceneObjectProp::id);
		db->GetType(id)
			->AddAttr("density"         , Primitive::Real, 1, OFFSET(PhysicalMaterialProp, density         ), 1.0, AttrCategory::Param, Dimension::M_LLL)
			->AddAttr("cor"             , Primitive::Real, 1, OFFSET(PhysicalMaterialProp, cor             ), 0.3, AttrCategory::Param)
			->AddAttr("static_friction" , Primitive::Real, 1, OFFSET(PhysicalMaterialProp, static_friction ), 0.3, AttrCategory::Param)
			->AddAttr("dynamic_friction", Primitive::Real, 1, OFFSET(PhysicalMaterialProp, dynamic_friction), 0.3, AttrCategory::Param)
			->AddAttr("spring"          , Primitive::Real, 1, OFFSET(PhysicalMaterialProp, spring          ), 0.0, AttrCategory::Param)
			->AddAttr("damper"          , Primitive::Real, 1, OFFSET(PhysicalMaterialProp, damper          ), 0.0, AttrCategory::Param);
	}
};

/** VisualMaterial
	- 視覚的材質
 **/
struct VisualMaterialProp : SceneObjectProp{
	static int id;

	vec4_t      ambient;		///< ambient color
	vec4_t      diffuse;		///< diffuse color
	vec4_t      specular;		///< specular color
	vec4_t      emissive;		///< emissive color
	real_t      shininess;		///< shininess
	real_t      alpha;			///< transparency (used in combination with colorname)
	str32_t     colorname;		///< X11 web color name
	str256_t	texture;		///< texture file name

	static string GetName(){ return "vmat"; }
	static void Construct(Property* p){ new(p) VisualMaterialProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(VisualMaterialProp), &Construct, SceneObjectProp::id);
		db->GetType(id)
			->AddAttr("ambient"       , Primitive::Vec4,     1, OFFSET(VisualMaterialProp, ambient       ), vec4_t(0.2, 0.2, 0.2, 1.0))
			->AddAttr("diffuse"       , Primitive::Vec4,     1, OFFSET(VisualMaterialProp, diffuse       ), vec4_t(0.8, 0.8, 0.8, 1.0))
			->AddAttr("specular"      , Primitive::Vec4,     1, OFFSET(VisualMaterialProp, specular      ), vec4_t(0.0, 0.0, 0.0, 1.0))
			->AddAttr("emissive"      , Primitive::Vec4,     1, OFFSET(VisualMaterialProp, emissive      ), vec4_t(0.0, 0.0, 0.0, 1.0))
			->AddAttr("shininess"     , Primitive::Real,     1, OFFSET(VisualMaterialProp, shininess     ), 1.0)
			->AddAttr("alpha"         , Primitive::Real,     1, OFFSET(VisualMaterialProp, alpha         ), 1.0)
			->AddAttr("colorname"     , Primitive::String,  32, OFFSET(VisualMaterialProp, colorname     ), "")
			->AddAttr("texture"       , Primitive::String, 256, OFFSET(VisualMaterialProp, texture       ), "");
	}
};

/** Shape
	- 衝突判定や視覚化に用いられる形状
	- 形状はシーン中のどこに置いても良いがそれだけでは機能を持たず，
	  ShapeLinkからリンクすることによって所定の座標に配置する必要がある．
	- どの種類の形状がサポートされるかはアダプタ次第
 **/
struct ShapeProp : SceneObjectProp{
	static int id;
	bool		collision;			///< 衝突判定用
	bool        mass;               ///< 質量計算用
	bool		visual;				///< 可視化用
	str32_t	    auto_texcoord;		///< texture coordinate generation method
	vec2_t      texture_scale;
	vec2_t      texture_offset;

	/// texture coordinate generation method
	enum{
		None,
		Box,
		Sphere,
	};

	int AutoTexCoord(){
		if(auto_texcoord == "box")
			return Box;
		if(auto_texcoord == "sphere")
			return Sphere;
		return None;
	}

	static string GetName(){ return "shape_base"; }
	static void Construct(Property* p){ new(p) ShapeProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(ShapeProp), &Construct, SceneObjectProp::id);
		db->GetType(id)
			->AddAttr("collision"     ,	Primitive::Bool  ,  1, OFFSET(ShapeProp, collision),	true)
			->AddAttr("mass"          ,	Primitive::Bool  ,  1, OFFSET(ShapeProp, mass),	        true)
			->AddAttr("visual"        ,	Primitive::Bool  ,  1, OFFSET(ShapeProp, visual),		true)
			->AddAttr("auto_texcoord" , Primitive::String, 32, OFFSET(ShapeProp, auto_texcoord ), "")
		    ->AddAttr("texture_scale" , Primitive::Vec2  ,  1, OFFSET(ShapeProp, texture_scale ), vec2_t(1.0, 1.0))
			->AddAttr("texture_offset", Primitive::Vec2  ,  1, OFFSET(ShapeProp, texture_offset), vec2_t(0.0, 0.0));
	}
};

/** Box
	- 箱
 **/
struct BoxProp : ShapeProp{
	static int id;
	vec3_t	size;		///< length in each direction

	static string GetName(){ return "box"; }
	static void Construct(Property* p){ new(p) BoxProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(BoxProp), &Construct, ShapeProp::id);
		db->GetType(id)
			->AddAttr("size", Primitive::Vec3, 1, OFFSET(BoxProp, size), vec3_t(1.0, 1.0, 1.0), AttrCategory::Param, Dimension::L);
	}
};

/** Sphere
	- 球
 **/
struct SphereProp : ShapeProp{
	static int id;
	real_t	radius;		///< radius
	uint    slice;
	uint    stack;

	static string GetName(){ return "sphere"; }
	static void Construct(Property* p){ new(p) SphereProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(SphereProp), &Construct, ShapeProp::id);
		db->GetType(id)
			->AddAttr("radius", Primitive::Real, 1, OFFSET(SphereProp, radius), 1.0, AttrCategory::Param, Dimension::L)
			->AddAttr("slice" , Primitive::Int , 1, OFFSET(SphereProp, slice ), 16 , AttrCategory::Param)
			->AddAttr("stack" , Primitive::Int , 1, OFFSET(SphereProp, stack ), 16 , AttrCategory::Param);
	}
};

/** Cylinder
	- 円柱
 **/
struct CylinderProp : ShapeProp{
	static int id;
	real_t	radius;		///< radius
	real_t	height;		///< height
	uint    slice;

	static string GetName(){ return "cylinder"; }
	static void Construct(Property* p){ new(p) CylinderProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(CylinderProp), &Construct, ShapeProp::id);
		db->GetType(id)
			->AddAttr("radius", Primitive::Real, 1, OFFSET(CylinderProp, radius), 1.0, AttrCategory::Param, Dimension::L)
			->AddAttr("height", Primitive::Real, 1, OFFSET(CylinderProp, height), 1.0, AttrCategory::Param, Dimension::L)
			->AddAttr("slice" , Primitive::Int , 1, OFFSET(CylinderProp, slice ), 16 , AttrCategory::Param);
	}
};

/** Capsule
	- カプセル
 **/
struct CapsuleProp : ShapeProp{
	static int id;
	real_t	radius;		///< radius
	real_t	height;		///< height
	uint    slice;
	uint    stack;

	static string GetName(){ return "capsule"; }
	static void Construct(Property* p){ new(p) CapsuleProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(CapsuleProp), &Construct, ShapeProp::id);
		db->GetType(id)
			->AddAttr("radius", Primitive::Real, 1, OFFSET(CapsuleProp, radius), 1.0, AttrCategory::Param, Dimension::L)
			->AddAttr("height", Primitive::Real, 1, OFFSET(CapsuleProp, height), 1.0, AttrCategory::Param, Dimension::L)
			->AddAttr("slice" , Primitive::Int , 1, OFFSET(CapsuleProp, slice ), 16 , AttrCategory::Param)
			->AddAttr("stack" , Primitive::Int , 1, OFFSET(CapsuleProp, stack ), 16 , AttrCategory::Param);
	}
};

/** Mesh
	- メッシュ
	- メッシュデータを定義するファイルパスのみ保持
	- ロードして表示するのはアダプタの役割

	- 衝突判定形状として用いる場合:
	 - prism == false: メッシュの凸包が作られる
	 - prism == true : メッシュが角柱化される
 **/
struct MeshProp : ShapeProp{
	static int id;
	str256_t	filename;		///< ファイル名
	real_t      scale;			///< 縮尺 0.0を指定したらビルダーが単位にもとづいて設定する
	bool	    prism;			///< 角柱化
	vec3_t	    prismdir;		///< 角柱化の向き
	vec3_t      mirror;         ///< 鏡像反転の方向
	bool        stl_color;      ///< STLファイルから色情報を取得

	static string GetName(){ return "mesh"; }
	static void Construct(Property* p){ new(p) MeshProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(MeshProp), &Construct, ShapeProp::id);
		db->GetType(id)
			->AddAttr("filename" , Primitive::String, 256, OFFSET(MeshProp, filename ), ""           )
			->AddAttr("scale"    , Primitive::Real,     1, OFFSET(MeshProp, scale    ), 0.0          )
			->AddAttr("prism"    , Primitive::Bool,     1, OFFSET(MeshProp, prism    ), false        )
			->AddAttr("prismdir" , Primitive::Vec3,     1, OFFSET(MeshProp, prismdir ), vec3_t(0,0,1))
			->AddAttr("mirror"   , Primitive::Vec3,     1, OFFSET(MeshProp, mirror   ), vec3_t(0,0,0))
			->AddAttr("stl_color", Primitive::Bool,     1, OFFSET(MeshProp, stl_color), false        );
	}
};

/** Light
	- ライト
	- 主に視覚化における移動光源として用いる
 **/
struct LightProp : SceneObjectProp{
	static int id;
	static string GetName(){ return "light"; }
	static void Construct(Property* p){ new(p) LightProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(LightProp), &Construct, SceneObjectProp::id);
	}
};

/** Camera
	- カメラ
	- 視覚化における視点切り替え
 **/
struct CameraProp : SceneObjectProp{
	static int id;
	real_t fov;

	static string GetName(){ return "camera"; }
	static void Construct(Property* p){ new(p) CameraProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(CameraProp), &Construct, SceneObjectProp::id);
		db->GetType(id)
			->AddAttr("fov", Primitive::Real, 1, OFFSET(CameraProp, fov), 0.3, AttrCategory::Param);
	}
};

/** Attach
	- 形状やライト，カメラなどをコネクタに配置するのに使う
 **/
struct AttachProp : SceneObjectProp{
	static int id;

	str256_t	bone;

	static string GetName(){ return "attach"; }
	static void Construct(Property* p){ new(p) AttachProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(AttachProp), &Construct, SceneObjectProp::id);
		db->GetType(id)
			->AddAttr("connector", Primitive::Path, 1, 0)
			->AddAttr("shape"    , Primitive::Path, 1, 0)
			->AddAttr("vmat"     , Primitive::Path, 1, 0)
			->AddAttr("pmat"     , Primitive::Path, 1, 0)
			->AddAttr("light"    , Primitive::Path, 1, 0)
			->AddAttr("camera"   , Primitive::Path, 1, 0)
			->AddAttr("bone"     , Primitive::String, 256, OFFSET(AttachProp, bone), "", AttrCategory::Param);
	}
};

/** Joint
 **/
struct JointProp : SceneObjectProp{
	static int id;
	bool	enabled;
	static string GetName(){ return "joint_base"; }
	static void Construct(Property* p){ new(p) JointProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(JointProp), &Construct, SceneObjectProp::id);
		db->GetType(id)
			->AddAttr("sock",	 Primitive::Path, 1, 0)
			->AddAttr("plug",	 Primitive::Path, 1, 0)
			->AddAttr("enabled", Primitive::Bool, 1, OFFSET(JointProp, enabled), true);
	}
};

/** Joint1D
	- 1自由度関節
 **/
struct Joint1DProp : JointProp{
	static int id;
	vec2_t	  range;			///< position range
	vec2_t	  vrange;			///< velocity range
	real_t    flim;             ///< force limit
	real_t	  damper;			///< damper	: velocity feedback gain
	real_t	  spring;			///< spring : position feedback gain
	real_t	  pos;				///< position
	real_t	  vel;				///< velocity
	real_t	  targetpos;		///< target position
	real_t	  targetvel;		///< target velocity
	str256_t  filename;			///< dll filename for generic joint

	static string GetName(){ return "joint1d_base"; }
	static void Construct(Property* p){ new(p) Joint1DProp; }
	static void Register(TypeDB* db){
		// HingeとSliderで変数の物理次元が異なるので，登録は派生クラスで行う
		id = db->AddType(GetName(), sizeof(Joint1DProp), &Construct, JointProp::id);
		db->GetType(id)
			->AddAttr("filename", Primitive::String, 256, OFFSET(Joint1DProp, filename), "", AttrCategory::Param);
	}
};

/** Hinge
	- ヒンジ．1自由度回転関節
 **/
struct HingeProp : Joint1DProp{
	static int id;
	bool cyclic;

	static string GetName(){ return "hinge"; }
	static void Construct(Property* p){ new(p) HingeProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(HingeProp), &Construct, Joint1DProp::id);
		real_t inf = numeric_limits<real_t>::max();
		vec2_t inf2(-inf, inf);
		db->GetType(id)
			->AddAttr("range"    , Primitive::Vec2, 1, OFFSET(HingeProp, range    ), inf2, AttrCategory::Param, Dimension::R      )
			->AddAttr("vrange"   , Primitive::Vec2, 1, OFFSET(HingeProp, vrange   ), inf2, AttrCategory::Param, Dimension::R_T    )
			->AddAttr("flim"     , Primitive::Real, 1, OFFSET(HingeProp, flim     ), inf , AttrCategory::Param, Dimension::MLL_TT )
			->AddAttr("damper"   , Primitive::Real, 1, OFFSET(HingeProp, damper   ), 0.0 , AttrCategory::Param, Dimension::MLL_RT )
			->AddAttr("spring"   , Primitive::Real, 1, OFFSET(HingeProp, spring   ), 0.0 , AttrCategory::Param, Dimension::MLL_RTT)
			->AddAttr("pos"      , Primitive::Real, 1, OFFSET(HingeProp, pos      ), 0.0 , AttrCategory::State, Dimension::R	  )
			->AddAttr("vel"      , Primitive::Real, 1, OFFSET(HingeProp, vel      ), 0.0 , AttrCategory::State, Dimension::R_T	  )
			->AddAttr("targetpos", Primitive::Real, 1, OFFSET(HingeProp, targetpos), 0.0 , AttrCategory::State, Dimension::R      )
			->AddAttr("targetvel", Primitive::Real, 1, OFFSET(HingeProp, targetvel), 0.0 , AttrCategory::State, Dimension::R_T    )
			->AddAttr("cyclic"   , Primitive::Bool, 1, OFFSET(HingeProp, cyclic   ), true, AttrCategory::Param);
	}
};

/** Slider
	- スライダ．1自由度直動関節
 **/
struct SliderProp : Joint1DProp{
	static int id;
	
	static string GetName(){ return "slider"; }
	static void Construct(Property* p){ new(p) SliderProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(SliderProp), &Construct, Joint1DProp::id);
		real_t inf = numeric_limits<real_t>::max();
		vec2_t inf2(-inf, inf);
		db->GetType(id)
			->AddAttr("range",     Primitive::Vec2, 1, OFFSET(SliderProp, range    ), inf2, AttrCategory::Param, Dimension::L    )
			->AddAttr("vrange",    Primitive::Vec2, 1, OFFSET(SliderProp, vrange   ), inf2, AttrCategory::Param, Dimension::L_T  )
			->AddAttr("flim"     , Primitive::Real, 1, OFFSET(SliderProp, flim     ), inf , AttrCategory::Param, Dimension::ML_TT)
			->AddAttr("damper",    Primitive::Real, 1, OFFSET(SliderProp, damper   ), 0.0 , AttrCategory::Param, Dimension::M_T  )
			->AddAttr("spring",    Primitive::Real, 1, OFFSET(SliderProp, spring   ), 0.0 , AttrCategory::Param, Dimension::M_TT )
			->AddAttr("pos",       Primitive::Real, 1, OFFSET(SliderProp, pos      ), 0.0 , AttrCategory::State, Dimension::L    )
			->AddAttr("vel",       Primitive::Real, 1, OFFSET(SliderProp, vel      ), 0.0 , AttrCategory::State, Dimension::L_T  )
			->AddAttr("targetpos", Primitive::Real, 1, OFFSET(SliderProp, targetpos), 0.0 , AttrCategory::State, Dimension::L    )
			->AddAttr("targetvel", Primitive::Real, 1, OFFSET(SliderProp, targetvel), 0.0 , AttrCategory::State, Dimension::L_T  );
	}
};

/** Balljoint
	- 球面ジョイント．
 **/
struct BalljointProp : JointProp{
	static int id;

	real_t	  damper;			///< damper	: velocity feedback gain
	real_t	  spring;			///< spring : position feedback gain
	quat_t	  pos;				///< position
	vec3_t	  vel;				///< velocity
	quat_t	  targetpos;		///< target position
	vec3_t	  targetvel;		///< target velocity

	static string GetName(){ return "balljoint"; }
	static void Construct(Property* p){ new(p) BalljointProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(BalljointProp), &Construct, JointProp::id);
		db->GetType(id)
			->AddAttr("damper"   , Primitive::Real, 1, OFFSET(BalljointProp, damper   ), 0.0     , AttrCategory::Param, Dimension::MLL_RT )
			->AddAttr("spring"   , Primitive::Real, 1, OFFSET(BalljointProp, spring   ), 0.0     , AttrCategory::Param, Dimension::MLL_RTT)
			->AddAttr("pos"      , Primitive::Quat, 1, OFFSET(BalljointProp, pos      ), quat_t(), AttrCategory::State, Dimension::R	  )
			->AddAttr("vel"      , Primitive::Vec3, 1, OFFSET(BalljointProp, vel      ), vec3_t(), AttrCategory::State, Dimension::R_T	  )
			->AddAttr("targetpos", Primitive::Quat, 1, OFFSET(BalljointProp, targetpos), quat_t(), AttrCategory::State, Dimension::R      )
			->AddAttr("targetvel", Primitive::Vec3, 1, OFFSET(BalljointProp, targetvel), vec3_t(), AttrCategory::State, Dimension::R_T    );
	}
};

/** Spring
  - 並進自由度に関するバネ・ダンパ
 */
struct SpringProp : JointProp{
	static int id;

	real_t	  damper;			///< damper	: velocity feedback gain
	real_t	  spring;			///< spring : position feedback gain
	
	static string GetName(){ return "spring"; }
	static void Construct(Property* p){ new(p) SpringProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(SpringProp), &Construct, JointProp::id);
		db->GetType(id)
			->AddAttr("damper", Primitive::Real, 1, OFFSET(SpringProp, damper), 0.0     , AttrCategory::Param, Dimension::MLL_RT )
			->AddAttr("spring", Primitive::Real, 1, OFFSET(SpringProp, spring), 0.0     , AttrCategory::Param, Dimension::MLL_RTT);
	}
};

/** Fixjoint
 */
struct FixjointProp : JointProp{
	static int id;
	static string GetName(){ return "fixjoint"; }
	static void Construct(Property* p){ new(p) FixjointProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(FixjointProp), &Construct, JointProp::id);
	}
};

/** Mate
 */
struct PointToPointProp : JointProp{
	static int id;
	static string GetName(){ return "point_to_point"; }
	static void Construct(Property* p){ new(p) PointToPointProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(PointToPointProp), &Construct, JointProp::id);
	}
};
struct PointToLineProp : JointProp{
	static int id;
	static string GetName(){ return "point_to_line"; }
	static void Construct(Property* p){ new(p) PointToLineProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(PointToLineProp), &Construct, JointProp::id);
	}
};
struct PointToPlaneProp : JointProp{
	static int id;

	vec2_t	range;

	static string GetName(){ return "point_to_plane"; }
	static void Construct(Property* p){ new(p) PointToPlaneProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(PointToPlaneProp), &Construct, JointProp::id);
		db->GetType(id)
			->AddAttr("range", Primitive::Vec2, 1, OFFSET(PointToPlaneProp, range), vec2_t(), AttrCategory::Param, Dimension::M);
	}
};
struct LineToLineProp : JointProp{
	static int id;
	static string GetName(){ return "line_to_line"; }
	static void Construct(Property* p){ new(p) LineToLineProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(LineToLineProp), &Construct, JointProp::id);
	}
};
struct PlaneToPlaneProp : JointProp{
	static int id;
	static string GetName(){ return "plane_to_plane"; }
	static void Construct(Property* p){ new(p) PlaneToPlaneProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(PlaneToPlaneProp), &Construct, JointProp::id);
	}
};

/** Gear
	- 歯車
 */
struct GearProp : SceneObjectProp{
	static int id;
	real_t	    ratio;
	real_t      offset;
	str32_t	    type;

	static string GetName(){ return "gear"; }
	static void Construct(Property* p){ new(p) GearProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(GearProp), &Construct, SceneObjectProp::id);
		db->GetType(id)
			->AddAttr("up"    , Primitive::Path,   1 , 0)
			->AddAttr("down"  , Primitive::Path,   1 , 0)
			->AddAttr("ratio" , Primitive::Real,   1 , OFFSET(GearProp, ratio ), 1.0, AttrCategory::Param, Dimension::None)
			->AddAttr("offset", Primitive::Real,   1 , OFFSET(GearProp, offset), 0.0, AttrCategory::Param, Dimension::None)
			->AddAttr("type"  , Primitive::String, 32, OFFSET(GearProp, type  ), "");
	}
};

/** Gravity
	- 重力場
 **/
struct GravityProp : SceneObjectProp{
	static int id;
	vec3_t	accel;

	static string GetName(){ return "gravity"; }
	static void Construct(Property* p){ new(p) GravityProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(GravityProp), &Construct, SceneObjectProp::id);
		db->GetType(id)
			->AddAttr("accel",		Primitive::Vec3, 1, OFFSET(GravityProp, accel),    vec3_t(), AttrCategory::Param, Dimension::L_TT);
	}
};

/** ContactGroup **/
struct ContactGroupProp : SceneObjectProp{
	static int id;
	bool enable;
	bool all_bodies;

	static string GetName(){ return "contact_group"; }
	static void Construct(Property* p){ new(p) ContactGroupProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(ContactGroupProp), &Construct, SceneObjectProp::id);
		db->GetType(id)
			->AddAttr("enable"    , Primitive::Bool, 1, OFFSET(ContactGroupProp, enable    ), true )
			->AddAttr("all_bodies", Primitive::Bool, 1, OFFSET(ContactGroupProp, all_bodies), false);
	}
};

/** IK **/
struct IKProp : SpatialObjectProp{
	static int id;
	bool	enable_trn;
	bool	enable_rot;

	static string GetName(){ return "ik"; }
	static void Construct(Property* p){ new(p) IKProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(IKProp), &Construct, SpatialObjectProp::id);
		db->GetType(id)
			->AddAttr("sock", Primitive::Path, 1, 0)
			->AddAttr("enable_trn", Primitive::Bool, 1, OFFSET(IKProp, enable_trn), true , AttrCategory::Param)
			->AddAttr("enable_rot", Primitive::Bool, 1, OFFSET(IKProp, enable_rot), false, AttrCategory::Param);
	}
};

struct IKJointProp : SceneObjectProp{
	static int id;
	
	static string GetName(){ return "ikjoint"; }
	static void Construct(Property* p){ new(p) IKJointProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(IKJointProp), &Construct, SceneObjectProp::id);
		db->GetType(id)
			->AddAttr("path", Primitive::Path, 1, 0);
	}
};

struct IKComProp : SceneObjectProp{
	static int id;
	vec3_t	pos;
	bool    enable;

	static string GetName(){ return "ikcom"; }
	static void Construct(Property* p){ new(p) IKComProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(IKComProp), &Construct, SceneObjectProp::id);
		db->GetType(id)
			->AddAttr("pos"   , Primitive::Vec3, 1, OFFSET(IKComProp, pos   ), vec3_t(), AttrCategory::State, Dimension::L)
			->AddAttr("enable", Primitive::Bool, 1, OFFSET(IKComProp, enable), true    , AttrCategory::Param);
	}
};

///////////////////////////////////////////////////////////////////////////////////////////////////

/** Motor
	- モータ
 **/
struct MotorProp : SceneObjectProp{
	static int id;
	static string GetName(){ return "motor"; }
	static void Construct(Property* p){ new(p) MotorProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(MotorProp), &Construct, SceneObjectProp::id);
	}
};

/** Sensor
	- センサ
	- エージェントの子オブジェクトとして用いる
	- コネクタにリンクして取り付け位置を指定する
 **/
struct SensorProp : SceneObjectProp{
	static int id;
	static string GetName(){ return "sensor_base"; }
	static void Construct(Property* p){ new(p) SensorProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(SensorProp), &Construct, SceneObjectProp::id);
	}
};

/** JointSensor
	- 関節センサ
	- 関節の変位や速度を得る
 **/
struct JointSensorProp : SensorProp{
	static int id;
	static string GetName(){ return "joint_sensor"; }
	static void Construct(Property* p){ new(p) JointSensorProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(JointSensorProp), &Construct, SensorProp::id);
	}
};

/** ForceSensor
	- 
 **/
struct ForceSensorProp : SensorProp{
	static int id;
	static string GetName(){ return "force_sensor"; }
	static void Construct(Property* p){ new(p) ForceSensorProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(ForceSensorProp), &Construct, SensorProp::id);
	}
};

/** InertiaSensor
	- 慣性センサ
	- Bodyの子オブジェクトとして用いる
	- 取り付けた位置での剛体の加速度，角速度を得る
 **/
struct InertiaSensorProp : SensorProp{
	static int id;
	static string GetName(){ return "inertia_sensor"; }
	static void Construct(Property* p){ new(p) InertiaSensorProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(InertiaSensorProp), &Construct, SensorProp::id);
	}
};

/** ProximitySensor
	- 距離センサ
 **/
struct ProximitySensorProp : SensorProp{
	static int id;
	static string GetName(){ return "proximity_sensor"; }
	static void Construct(Property* p){ new(p) ProximitySensorProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(ProximitySensorProp), &Construct, SensorProp::id);
	}
};

/** LidarSensor
	- 平面距離センサ
 **/
struct LidarSensorProp : SensorProp{
	static int id;
	static string GetName(){ return "lidar_sensor"; }
	static void Construct(Property* p){ new(p) LidarSensorProp; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(LidarSensorProp), &Construct, SensorProp::id);
	}
};

///////////////////////////////////////////////////////////////////////////////////////////////////
/** シーンのインタフェース
 **/
class SceneBase : public TreeBase{
protected:
	/** checks if a name is used in this namespace **/
	bool FindName(const string& str, int parId);

public:
	static void Register(TypeDB* db);

	/** assigns a new name **/
	string	AssignName(const string& name, TypeInfo* tag, int parId);
	
	/** finds the nearest ancestor of an object (id) that is KindOf (type) **/
	int			FindAncestor(int id, int type, TypeDB* db);

	/** enumerates all objects below object (id) whose type is KindOf (type) **/
	void		EnumObjects (int id, int type, TypeDB* db, vector<int>& objs);

	/** calculates the relative transform of two spatial objects in the same hierarchy **/
	void		CalcRelativeTransform(int id0, int id1, TypeDB* db, Transform& trans);

	/** virtual functions that must be implemented in derived classes **/

	/** @brief Creates new object
		@param type		object type
		@param name		object name
		@return			id of new object
	 **/
	virtual int			CreateObject(int type, const string& name, TypeDB* typedb) = 0;

	/** @brief deletes an existing object
		@param id
		the following things are deleted:
		* all objects that belong to an object subtree whose root is an object specified by [id]
		* parent-child relation between object specified by [id] and its parent (unless [id] is root)
		* all links starting from an object that belongs to the subtree
		* all links that points to an object that belongs to the subtree
	 **/
	virtual void		DeleteObject(int id) = 0;

	/** @brief Returns object type
		@param id		object id
		@return			object type
	 **/
	virtual int			GetObjectType(int id) = 0;
	
	/** @brief Adds link
	 **/
	virtual void		AddLink(int srcId, int destId, const string& name) = 0;

	/** @brief Removes link
	 **/
	virtual void		RemoveLink(int srcId, int destId) = 0;

	/** @brief Get links from object (id)
	 **/
	virtual int 		GetLink (int id, const string& name) = 0;
	virtual void		GetLinks(int id, vector<int>& links, vector<string>& names) = 0;

	/** @brief Returns property address
		@param id		object id
		@return			pointer to property
	 **/
	virtual Property*	GetProperty(int id) = 0;

	/** @brief Copies properties of multiple objects into a buffer
		@param idArray	array of object ids
		@param num		number of objects
		@param buf		buffer to which properties are stored
	 **/
	//virtual void		GetPropertyArray(const int* idArray, size_t num, Buffer& buf) = 0;

	/** @brief Reads properties of multiple objects from a buffer
		@param idArray	array of object ids
		@param num		number of objects
		@param buf		buffer from which properties are read
	 **/
	//virtual void		SetPropertyArray(const int* idArray, size_t num, Buffer& buf) = 0;

	/** @brief Gets object liveness
		@param addr		absolute address to target subscene
		@param alive	reference to array to store liveness
		呼出し後にaliveの長さはオブジェクト最大数と一致し，アドレスaddr内に使用中のオブジェクトidが存在するとき
		alive[id] == 1，そうでなければalive[id] == 0となる．
	 **/
	virtual void		GetLivenessArray(Address& addr, vector<byte>& alive) = 0;

	/** @brief Sets physical location of an object
		@param id		object id
		@param path		physical path
		絶対パスの場合はそのままオブジェクトのロケーションを表す
		相対パスの場合はルートオブジェクトに向かって親オブジェクトの持つパスを連結したものがロケーションとなる
	 **/
	virtual void		SetLocation(int id, const string& path) = 0;

	/** @brief Gets physical location of an object
		@param id		object id

	 **/
	virtual string		GetLocation(int id) = 0;
};

}
