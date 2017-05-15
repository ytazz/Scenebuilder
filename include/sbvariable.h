#pragma once

#include <sbtypes.h>

#include <vector>
using namespace std;

namespace Scenebuilder{;

class	Solver;
class	Link;

typedef std::vector< Link* >	Links;

/**
	拘束される変数
 */
class Variable : public UTRefCount{
public:
	/// variable types
	enum{
		Scalar = 1,
		Vec3   = 3,
		Quat   = 4,
	};

	typedef void (Variable::*UpdateFunc)(uint k, real_t d);

	Solver*	 solver;
	Links	 links;			///< links to constraints
	bool	 locked;		///< locked or not
	uint	 type;			///< variable type
	uint	 nelem;			///< number of elements
	uint     index;
	real_t   dmax, dmax2;    ///< upper limit of delta norm
	real_t   weight;
	vec3_t	 dx;            ///< delta
	vec3_t   dz, dzd;
	vec3_t	 J, Jinv;		///< square sum of Jacobian column and its inverse
	
public:
	void Lock(bool on = true);
	
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
	virtual real_t	Norm  ()            = 0;
	virtual void	Modify(real_t rate) = 0;

	Variable(uint type, Solver* solver);
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

	VariableImpl(uint _type, Solver* solver):Variable(_type, solver){}
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
	virtual real_t Norm(){
		return abs(val);
	}
	virtual void Modify(real_t alpha){
		val = val_tmp + alpha * dx[0];
	}

	SVar(Solver* solver):VariableImpl(Variable::Scalar, solver){
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
	virtual real_t Norm(){
		return val.norm();
	}
	virtual void Modify(real_t alpha){
		val = val_tmp + alpha * dx;
	}

	V3Var(Solver* solver):VariableImpl(Variable::Vec3, solver){
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
	virtual real_t Norm(){
		return ((vec4_t&)val).norm();
	}
	virtual void Modify(real_t alpha){
		val = quat_t::Rot(alpha * dx) * val_tmp;
		val.unitize();
	}

	QVar(Solver* solver):VariableImpl(Variable::Quat, solver){
		Reset();
	}
};

}
