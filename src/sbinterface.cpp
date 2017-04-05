#include "../include/sbinterface.h"

#include <expat/Source/lib/expat.h>

namespace Scenebuilder{;

//-------------------------------------------------------------------------------------------------

Context::Context(){
	//curBody		= 0;
}

Interface::Interface(){
	//resJoint	= 0;
	basename[SCENE]			= "scene";
	basename[NAMESPACE]		= "namespace";
	basename[BODY]			= "body";
	basename[CONNECTOR]		= "";
	basename[JOINT_HINGE]	= "hinge";
	basename[JOINT_SLIDER]	= "slider";
	basename[JOINT_BALL]	= "balljoint";
	basename[SHAPE_REF]		= "shape";
	basename[SHAPE_BOX]		= "box";
	basename[SHAPE_SPHERE]	= "sphere";
	basename[SHAPE_MESH]	= "mesh";
}

Interface::~Interface(){

}

//-------------------------------------------------------------------------------------------------

void Interface::Move(Vec3d v){
	ctx.pose.Pos() += v;
}
void Interface::MoveX(double x){
	ctx.pose.Pos().x += x;
}
void Interface::MoveY(double y){
	ctx.pose.Pos().y += y;
}
void Interface::MoveZ(double z){
	ctx.pose.Pos().z += z;
}
void Interface::Rot(Quaterniond q){
	ctx.pose.Ori() = ctx.pose.Ori() * q;
}
void Interface::RotX(double x){
	ctx.pose.Ori() = ctx.pose.Ori() * Quaterniond::Rot(x, 'x');
}
void Interface::RotY(double y){
	ctx.pose.Ori() = ctx.pose.Ori() * Quaterniond::Rot(y, 'y');
}
void Interface::RotZ(double z){
	ctx.pose.Ori() = ctx.pose.Ori() * Quaterniond::Rot(z, 'z');
}
void Interface::MirrorX(){

}
void Interface::MirrorY(){

}
void Interface::MirrorZ(){

}
void Interface::Push(){
	ctxStack.push(ctx);
}
void Interface::Pop(){
	if(!ctxStack.empty()){
		ctx = ctxStack.top();
		ctxStack.pop();
	}
}

//-------------------------------------------------------------------------------------------------

/*void Interface::Name(UTString name){
	ctx.name = name;
}

void Interface::Path(UTString path){
	ctx.path = path;
}*/

/*void Interface::BodyName(UTString name){
	ctx.basename[BODY] = name;
}

void Interface::ConnectorName(UTString name){
	ctx.basename[CONNECTOR] = name;
}

void Interface::HingeName(UTString name){
	ctx.basename[JOINT_HINGE] = name;
}

void Interface::SliderName(UTString name){
	ctx.basename[JOINT_SLIDER] = name;
}

void Interface::BalljointName(UTString name){
	ctx.basename[JOINT_BALL] = name;
}

void Interface::BoxName(UTString name){
	ctx.basename[SHAPE_BOX] = name;
}

void Interface::SphereName(UTString name){
	ctx.basename[SHAPE_SPHERE] = name;
}

void Interface::MeshName(UTString name){
	ctx.basename[SHAPE_MESH] = name;
}*/

void Interface::Mass(double mass){
	ctx.body.mass = mass;
}
void Interface::Inertia(double I){
	ctx.body.inertia = Matrix3d::Unit() * I;
}
void Interface::Inertia(double Ix, double Iy, double Iz){
	ctx.body.inertia = Matrix3d::Diag(Ix, Iy, Iz);
}
void Interface::Inertia(const Matrix3d& inertia){
	ctx.body.inertia = inertia;
}
void Interface::Dynamical(bool on){
	ctx.body.dynamical = on;
}
void Interface::Velocity(double vx, double vy, double vz){
	ctx.body.velocity = Vec3d(vx, vy, vz);
}
void Interface::AngVelocity(double wx, double wy, double wz){
	ctx.body.angVelocity = Vec3d(wx, wy, wz);
}
void Interface::JointPosRange(double rmin, double rmax){
	ctx.joint1D.range = Vec2d(rmin, rmax);
}
void Interface::Damper(double damper){
	ctx.joint1D.damper = damper;
}
void Interface::Spring(double spring){
	ctx.joint1D.spring = spring;
}
void Interface::JointPos(double pos){
	ctx.joint1D.pos = pos;
}
void Interface::JointVel(double vel){
	ctx.joint1D.vel = vel;
}
void Interface::JointTargetPos(double pos){
	ctx.joint1D.targetPos = pos;
}
void Interface::JointTargetVel(double vel){
	ctx.joint1D.targetVel = vel;
}
/*void Interface::ShapeUsage(bool collision, bool visual){
	ctx.shape.collision = collision;
	ctx.shape.visual = visual;
}*/
void Interface::BoxSize(double sx, double sy, double sz){
	ctx.box.size = Vec3d(sx, sy, sz);
}
void Interface::SphereRadius(double r){
	ctx.sphere.radius = r;
}
void Interface::MeshFileName(string filename){
	ctx.mesh.fileName = filename;
}

void Interface::Gravity(double gx, double gy, double gz){
	scene->SetGravity(Vec3d(gx, gy, gz));
}

//-------------------------------------------------------------------------------------------------

void Interface::Namespace(UTString name){
	NamespaceInfo* ns = new NamespaceInfo();
	ns->name				= AssignName(name, NAMESPACE, ctx.curObject);
	*(NamespaceDesc*)ns	= ctx.ns;
	ns->SetParent(ctx.curObject);
	ctx.curNS  = ns;
	ctx.curObj = -1;
}

void Interface::Body(UTString name){
	int id = scene->CreateBody(AssignName(name, BODY, ctx.curObject), ctx.pose, ctx.body, ctx.curObject);
	if(id != -1)
		ctx.curObject = id;
}

void Interface::End(){
	ctx.curObject = scene->GetParent(ctx.curObject);
}

void Interface::Connector(UTString name){
	if(ctx.curObject->type != BODY)
		return;
	ConnectorInfo* con = new ConnectorInfo();
	con->name				= AssignName(name, CONNECTOR, ctx.curObject);
	con->pose				= ctx.pose;
	*(ConnectorDesc*)con	= ctx.connector;
	con->SetParent(ctx.curObject);
}

void Interface::Hinge(UTString socket, UTString plug, UTString name){
	HingeInfo* hinge = new HingeInfo();
	hinge->name				= AssignName(name, JOINT_HINGE, ctx.curObject);
	hinge->sockPath			= socket;
	hinge->plugPath			= plug;
	*(JointDesc*)hinge		= ctx.joint;
	*(Joint1DDesc*)hinge	= ctx.joint1D;
	*(HingeDesc*)hinge		= ctx.hinge;
	hinge->SetParent(ctx.curObject);
	scene->joints.push_back(hinge);
}

void Interface::Shape(UTString path, UTString name){
	ShapeRefInfo* shape = new ShapeRefInfo();
	shape->name				= AssignName(name, SHAPE_REF, ctx.curObject);
	shape->path				= path;
	shape->pose				= ctx.pose;
	*(ShapeRefDesc*)shape	= ctx.shapeRef;
	shape->SetParent(ctx.curObject);
}

void Interface::Box(UTString name){
	BoxInfo* box = new BoxInfo();
	box->name				= AssignName(name, SHAPE_BOX, ctx.curObject);
	*(ShapeDesc*)box		= ctx.shape;
	*(BoxDesc*)box			= ctx.box;
	scene->shapes.push_back(box);
	box->SetParent(ctx.curObject);
}

void Interface::Sphere(UTString name){
	SphereInfo* sphere = new SphereInfo();
	sphere->name				= AssignName(name, SHAPE_SPHERE, ctx.curObject);
	*(ShapeDesc*)sphere		= ctx.shape;
	*(SphereDesc*)sphere	= ctx.sphere;
	scene->shapes.push_back(sphere);
	sphere->SetParent(ctx.curObject);
}

void Interface::Mesh(UTString name){
	MeshInfo* mesh = new MeshInfo();
	mesh->name				= AssignName(name, SHAPE_MESH, ctx.curObject);
	*(ShapeDesc*)mesh		= ctx.shape;
	*(MeshDesc*)mesh		= ctx.mesh;
	scene->shapes.push_back(mesh);
	mesh->SetParent(ctx.curObject);
}

//-------------------------------------------------------------------------------------------------
	
UTString Interface::AssignName(UTString name, int type, NamedObject* owner){
	UTString base, ret;

	// if "name" attribute is set
	if(!name.empty()){
		if(!owner->FindName(name))
			return name;
		base = name;
	}
	// otherwise, use basename
	else{
		base = basename[type];
	}
	
	stringstream ss;
	int idx = 1;
	do{
		ss.str("");
		ss << base << idx++;
		ret = ss.str();
	}while(owner->FindName(ret));

	owner->usedNames.insert(ret);

	return ret;
}

void Interface::ProcessAttribute(AttrInfo* attr, UTString value){
	// skip unknown attribute
	if(!attr)
		return;

	DecodeResult res;
	TypeDB::instance->Decode(res, attr, value);

	/*if(attr->id == ATTR_NAME){
		Name(value);
	}
	else*/
	if(attr->id == ATTR_MOVE){
		if(res.type == TYPE_VEC3)
			Move(res.vec3);
	}
	else if(attr->id == ATTR_MOVEX){
		if(res.type == TYPE_REAL)
			MoveX(res.real);
	}
	else if(attr->id == ATTR_MOVEY){
		if(res.type == TYPE_REAL)
			MoveY(res.real);
	}
	else if(attr->id == ATTR_MOVEZ){
		if(res.type == TYPE_REAL)
			MoveZ(res.real);
	}
	else if(attr->id == ATTR_ROT){
		if(res.type == TYPE_QUAT)
			Rot(res.quat);
	}
	else if(attr->id == ATTR_ROTX){
		if(res.type == TYPE_REAL)
			RotX(res.real);
	}
	else if(attr->id == ATTR_ROTY){
		if(res.type == TYPE_REAL)
			RotY(res.real);
	}
	else if(attr->id == ATTR_ROTZ){
		if(res.type == TYPE_REAL)
			RotZ(res.real);
	}
	else if(attr->id == ATTR_MASS){
		if(res.type == TYPE_REAL)
			Mass(res.real);
	}
	else if(attr->id == ATTR_INERTIA){
		if(res.type == TYPE_REAL)
			Inertia(res.real);
		if(res.type == TYPE_VEC3)
			Inertia(res.vec3[0], res.vec3[1], res.vec3[2]);
		if(res.type == TYPE_MAT3)
			Inertia(res.mat3);
	}
	else if(attr->id == ATTR_DYNAMICAL){
		if(res.type == TYPE_BOOL)
			Dynamical(res.boolean);
	}
	/*else if(attr->id == ATTR_JOINT_TYPE){
		JointTypeInfo* info = TypeDB::GetJointType(value);
		if(info)
			JointType(info->id);
	}*/
	/*else if(attr->id == ATTR_SOCKET){
		Socket(value);
	}
	else if(attr->id == ATTR_PLUG){
		Plug(value);
	}*/
	else if(attr->id == ATTR_JOINT_POS){
		if(res.type == TYPE_REAL)
			JointPos(res.real);
	}
	/*else if(attr->id == ATTR_SHAPE_TYPE){
		ShapeTypeInfo* info = TypeDB::GetShapeType(value);
		if(info)
			ShapeType(info->id);
	}*/
	else if(attr->id == ATTR_BOX_SIZE){
		if(res.type == TYPE_VEC3)
			BoxSize(res.vec3[0], res.vec3[1], res.vec3[2]);
	}

}

void Interface::ProcessNode(XMLNode* node){
	int tagid = node->tag->id;
	// push context unless "attr" tag
	if(tagid != TAG_ATTR)
		Push();

	// process attributes
	for(uint i = 0; i < node->atts.size(); i++)
		ProcessAttribute(node->atts[i].attr, node->atts[i].value);
	
	XMLNode* parent = 0;
	UTString name = node->GetAttr(ATTR_NAME);
	UTString path = node->GetAttr(ATTR_PATH);

	if(tagid == TAG_SCENE){
		//Begin(SCENE);
		parent = node;
	}
	else if(tagid == TAG_BODY){
		Body(name);
		parent = node;
	}
	else if(tagid == TAG_CONNECTOR){
		Connector(name);
	}
	else if(tagid == TAG_SHAPE){
		Shape(path, name);
	}
	else if(tagid == TAG_BOX){
		Box(name);
	}
	else if(tagid == TAG_SPHERE){
		Sphere(name);
	}
	else if(tagid == TAG_MESH){
		Mesh(name);
	}
	else if(tagid == TAG_IMPORT){
		UTString path = node->GetAttr(ATTR_PATH);
		parent = (XMLNode*)rootNode->Find(path, node->owner);
		if(parent)
			Namespace(name);
	}
		
	if(parent){
		for(uint i = 0; i < parent->children.size(); i++){
			ProcessNode(parent->GetChild(i));
		}
		End();
	}

	if(tagid != TAG_ATTR)
		Pop();
}

//-------------------------------------------------------------------------------------------------

void StartHandler(void *userData, const XML_Char *name, const XML_Char **atts){
	Interface* intf = (Interface*)userData;
	XMLNode* node = new XMLNode(name);
	
	while(*atts){
		node->atts.push_back(XMLNode::Attr(atts[0], atts[1]));
		atts += 2;
	}
	node->name = node->GetAttr(ATTR_NAME);

	if(!intf->rootNode){
		intf->rootNode = node;
	}
	else node->SetParent(intf->curNode);
	intf->curNode = node;
}

void EndHandler(void *userData, const XML_Char *name){
	Interface* intf = (Interface*)userData;
	intf->curNode = (XMLNode*)intf->curNode->owner;
}

bool Interface::Parse(const char* filename){
	// load file content to buffer
	string buff;
	ifstream ifs;
	ifs.open(filename, ifstream::in);

	if(!ifs.is_open())
		return false;

	char c;
	while(c = ifs.get(), ifs.good())
		buff += c;

	ifs.close();

	// create db
	TypeDB::Create();
	
	// parse xml
	XML_Parser parser = XML_ParserCreate(0);
	XML_SetElementHandler(parser, StartHandler, EndHandler);
	XML_SetUserData(parser, this);
	rootNode = 0;
	
	int ret = XML_Parse(parser, buff.c_str(), buff.size(), 1);

	XML_ParserFree(parser);

	if(ret == XML_STATUS_ERROR)
		return false;

	rootNode->Print(DSTR);

	//InflateTree();
	ProcessNode(rootNode);

	scene->Print(DSTR);

	return true;
}

}
