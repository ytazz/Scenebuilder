#include <sbscene.h>

#define AUTO(T, X, EXP) T X = static_cast<T>(EXP)

namespace Scenebuilder{;

int SceneObjectProp     ::id;
int SpatialObjectProp   ::id;
int NamespaceProp       ::id;
int BodyProp            ::id;
int ConnectorProp       ::id;
int PhysicalMaterialProp::id;
int VisualMaterialProp  ::id;
int ShapeProp           ::id;
int BoxProp             ::id;
int SphereProp          ::id;
int CylinderProp        ::id;
int CapsuleProp         ::id;
int MeshProp            ::id;
int LightProp           ::id;
int CameraProp          ::id;
int AttachProp          ::id;
int JointProp           ::id;
int Joint1DProp         ::id;
int HingeProp           ::id;
int SliderProp          ::id;
int UniversaljointProp  ::id;
int BalljointProp       ::id;
int SpringProp          ::id;
int FixjointProp        ::id;
int FreejointProp       ::id;
int PointToPointProp    ::id;
int PointToLineProp     ::id;
int PointToPlaneProp    ::id;
int LineToLineProp      ::id;
int PlaneToPlaneProp    ::id;
int GearProp            ::id;
int GravityProp         ::id;
int ContactGroupProp    ::id;
int IKProp              ::id;
int IKJointProp         ::id;
int IKComProp           ::id;
int MotorProp           ::id;
int SensorProp          ::id;
int JointSensorProp     ::id;
int ForceSensorProp     ::id;
int InertiaSensorProp   ::id;
int ProximitySensorProp ::id;
int LidarSensorProp     ::id;

Transform operator*(const Transform& t0, const Transform& t1){
	Transform t;

	mat3_t R;
	t1.rot.ToMatrix(R);
	if(t0.mirrorx){
		R.row(0) *= -1.0;
		R.col(0) *= -1.0;
	}
	if(t0.mirrory){
		R.row(1) *= -1.0;
		R.col(1) *= -1.0;
	}
	if(t0.mirrorz){
		R.row(2) *= -1.0;
		R.col(2) *= -1.0;
	}
	quat_t q;
	q.FromMatrix(R);
	t.rot = t0.rot * q;

	// スケールは積
	t.scale = t0.scale * t1.scale;

	// ミラー: 両方trueか両方falseならfalse -> ex-or
	t.mirrorx = (t0.mirrorx ^ t1.mirrorx);
	t.mirrory = (t0.mirrory ^ t1.mirrory);
	t.mirrorz = (t0.mirrorz ^ t1.mirrorz);

	// 平行移動
	vec3_t r = t1.trn;
	if(t0.mirrorx) r.x = -r.x;
	if(t0.mirrory) r.y = -r.y;
	if(t0.mirrorz) r.z = -r.z;
	t.trn = t0.scale * t0.rot * r + t0.trn;

	return t;
}

Transform Transform::Inv(){
	// 今のところミラーは考慮せず
	Transform tr;
	tr.rot = rot.Conjugated();
	tr.trn = -(tr.rot * trn);
	return tr;
}

void SceneBase::Register(TypeDB* db){
	SceneObjectProp     ::Register(db);
	SpatialObjectProp   ::Register(db);
	NamespaceProp       ::Register(db);
	BodyProp            ::Register(db);
	ConnectorProp       ::Register(db);
	PhysicalMaterialProp::Register(db);
	VisualMaterialProp  ::Register(db);
	ShapeProp           ::Register(db);
	BoxProp             ::Register(db);
	SphereProp          ::Register(db);
	CylinderProp        ::Register(db);
	CapsuleProp         ::Register(db);
	MeshProp            ::Register(db);
	LightProp           ::Register(db);
	CameraProp          ::Register(db);
	AttachProp          ::Register(db);
	JointProp           ::Register(db);
	Joint1DProp         ::Register(db);
	HingeProp           ::Register(db);
	SliderProp          ::Register(db);
	UniversaljointProp  ::Register(db);
	BalljointProp       ::Register(db);
	SpringProp          ::Register(db);
	FixjointProp        ::Register(db);
	FreejointProp       ::Register(db);
	PointToPointProp    ::Register(db);
	PointToLineProp     ::Register(db);
	PointToPlaneProp    ::Register(db);
	LineToLineProp      ::Register(db);
	PlaneToPlaneProp    ::Register(db);
	GearProp            ::Register(db);
	GravityProp         ::Register(db);
	ContactGroupProp    ::Register(db);
	IKProp              ::Register(db);
	IKJointProp         ::Register(db);
	IKComProp           ::Register(db);
	MotorProp           ::Register(db);
	LightProp           ::Register(db);
	SensorProp          ::Register(db);
	JointSensorProp     ::Register(db);
	ForceSensorProp     ::Register(db);
	InertiaSensorProp   ::Register(db);
	ProximitySensorProp ::Register(db);
	LidarSensorProp     ::Register(db);
}

string SceneBase::AssignName(const string& name, TypeInfo* type, int parId){
	string base, ret;

	// if "name" attribute is set
	if(!name.empty()){
		if(!FindName(name, parId))
			return name;
		base = name;
	}
	// otherwise, use basename
	else{
		base = type->name;
	}
	
	stringstream ss;
	int idx = 1;
	do{
		ss.str("");
		ss << base << idx++;
		ret = ss.str();
	}while(FindName(ret, parId));

	return ret;
}

bool SceneBase::FindName(const string& str, int parId){
	// 子オブジェクトから探す
	vector<int> children;
	GetChildren(parId, children);
	for(uint i = 0; i < children.size(); i++){
		if(GetName(children[i]) == str)
			return true;
	}

	// リンクから探す
	vector<int>    links;
	vector<string> names;
	GetLinks(parId, links, names);
	for(uint i = 0; i < names.size(); i++){
		if(names[i] == str)
			return true;
	}

	return false;
}

int SceneBase::FindAncestor(int id, int type, TypeDB* db){
	int parId = GetParent(id);
	if(parId == -1)
		return -1;
	if(db->KindOf(GetObjectType(parId), type))
		return parId;
	return FindAncestor(parId, type, db);
}

void SceneBase::EnumObjects(int id, int type, TypeDB* db, vector<int>& objs){
	vector<int> children;
	GetChildren(id, children);
	for(uint i = 0; i < children.size(); i++){
		EnumObjects(children[i], type, db, objs);
	}
	if(db->KindOf(GetObjectType(id), type))
		objs.push_back(id);
}

void SceneBase::CalcRelativeTransform(int id0, int id1, TypeDB* db, Transform& trans){
	if(id1 == -1 || id0 == id1){
		trans = Transform();
		return;
	}
	CalcRelativeTransform(id0, GetParent(id1), db, trans);
	// 
	int type = GetObjectType(id1);
	if(db->KindOf(type, SpatialObjectProp::id)){
		AUTO(SpatialObjectProp*, objProp, GetProperty(id1));
		trans = trans * *(Transform*)objProp;
	}
}

}
