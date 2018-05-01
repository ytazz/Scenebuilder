#pragma once

#include <sbtypes.h>

#include <vector>
using namespace std;

namespace Scenebuilder{;

class ID{
public:
	int     tag;
	void*   owner;
	void*   owner2;
	string  name;

	int Match(ID* id){
		// tagが未指定のマスクは必ずマッチする
		if(tag == -1)
			return 1;
		// tag不一致
		if(tag != id->tag)
			return 0;
		// tag一致, node未指定
		if(!owner)
			return 2;
		// node不一致
		if(owner != id->owner)
			return 0;
		// 全一致
		return 3;
	}

	ID(int _tag = -1, void* _owner = 0, void* _owner2 = 0, string _name = ""){
		tag    = _tag;
		owner  = _owner;
		owner2 = _owner2;
		name   = _name ;
	}
};

class	Solver;
class	Link;

typedef std::vector< Link* >	Links;

/**
	拘束される変数
 */
class Variable : public UTRefCount, public ID{
public:
	/// variable types
	enum{
		Scalar = 1,
		Vec2   = 2,
		Vec3   = 3,
		Quat   = 4,
	};

	typedef void (Variable::*UpdateFunc)(uint k, real_t d);

	Solver*	 solver;
	Links	 links;			///< links to constraints
	Links    links_active;
	bool	 locked;		///< locked or not
	int	     type;			///< variable type
	int	     nelem;			///< number of elements
	int      index;
	int      index_weighted;

	real_t	scale, scale2, scale_inv, scale_inv2;	///< scaling factor, its inverse and squared inverse

	real_t   dmax, dmax2;   ///< upper limit of delta norm
	real_t   weight;
	vec3_t	 dx;            ///< delta
	vec3_t   dz, dzd;
	vec3_t	 J, Jinv;		///< square sum of Jacobian column and its inverse
	
public:
	void Lock(bool on = true);
	
	void SetScale(real_t sc);

	virtual void ResetState();
	
	void Prepare();

	/** propagate change of variable
		@param k			element index
		@param dd_noscale	change of change, unscaled
		@param caller		Link that called this function
	 */
	void UpdateVar1(uint k, real_t _ddx);  ///< with    update of constraint error
	void UpdateVar2(uint k, real_t _ddx);  ///< without update of constraint error
	void UpdateVar3(uint k);
	
	void UpdateError(uint k);

	void UpdateConjugate1(uint k, real_t  ddz );
	void UpdateConjugate2(uint k, real_t  _dz );
	void UpdateConjugate3(uint k, real_t  ddzd);

	void RegisterDelta(const vvec_t& dxvec);
	
	virtual void	Reset ()            = 0;
	virtual real_t	Get   (uint k)      = 0;
	virtual void	Modify(real_t rate) = 0;

	Variable(uint type, Solver* solver, ID _id, real_t _scale);
};

template<class T>
class VariableImpl : public Variable{
public:
	T val;
	T val_tmp;

	virtual void ResetState(){
		Variable::ResetState();
		val_tmp = val;
	}

	VariableImpl(uint _type, Solver* solver, ID _id, real_t _scale):Variable(_type, solver, _id, _scale){}
};

/**
	scalar variable
 */
class SVar : public VariableImpl<real_t>{
public:
	virtual void Reset(){
		val = val_tmp = 0.0;
	}
	virtual real_t Get(uint k){
		return val;
	}
	virtual void Modify(real_t alpha){
		val = val_tmp + alpha * dx[0];
	}

	SVar(Solver* solver, ID _id = ID(), real_t _scale = 1.0):VariableImpl(Variable::Scalar, solver, _id, _scale){
		Reset();
	}
};

/**
	2D vector variable
 */
class V2Var : public VariableImpl<vec2_t>{
public:
	virtual void Reset(){
		val    .clear();
		val_tmp.clear();
	}
	virtual real_t Get(uint k){
		return val[k];
	}
	virtual void Modify(real_t alpha){
		val[0] = val_tmp[0] + alpha * dx[0];
		val[1] = val_tmp[1] + alpha * dx[1];
	}

	V2Var(Solver* solver, ID _id = ID(), real_t _scale = 1.0):VariableImpl(Variable::Vec2, solver, _id, _scale){
		Reset();
	}
};

/**
	3D vector variable
 */
class V3Var : public VariableImpl<vec3_t>{
public:
	virtual void Reset(){
		val    .clear();
		val_tmp.clear();
	}
	virtual real_t Get(uint k){
		return val[k];
	}
	virtual void Modify(real_t alpha){
		val = val_tmp + alpha * dx;
	}

	V3Var(Solver* solver, ID _id = ID(), real_t _scale = 1.0):VariableImpl(Variable::Vec3, solver, _id, _scale){
		Reset();
	}
};

/**
	quaternion variable
 */
class QVar : public VariableImpl<quat_t>{
public:
	virtual void Reset(){
		val     = quat_t();
		val_tmp = quat_t();
	}
	virtual real_t Get(uint k){
		return val[k];
	}
	virtual void Modify(real_t alpha){
		val = quat_t::Rot(alpha * dx) * val_tmp;
		val.unitize();
	}

	QVar(Solver* solver, ID _id = ID(), real_t _scale = 1.0):VariableImpl(Variable::Quat, solver, _id, _scale){
		Reset();
	}
};

}
