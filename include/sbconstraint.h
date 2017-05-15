#pragma once

#include <sbvariable.h>

#include <map>

using namespace std;
using namespace Spr;

namespace Scenebuilder{;

class Solver;
class SLink;
class XLink;
class CLink;
class RLink;

/**
	constraint base class
 */
class Constraint : public UTRefCount{
public:
	typedef void (Constraint::*UpdateFunc)(uint k, real_t d);

	Solver*		solver;
	Links		links;		///< links to constrained variables
	uint		nelem;
	uint        index;
	bool		enabled;			///< enabled constraint (controlled by user)
	bool		active;				///< active constraint (task-related constraints, range constraints)
	
	/** error correction rate
		誤差修正率．
		(誤差修正量) = (誤差) x (correction rate)
		通常は[0,1)の値を設定する．理論上は(-1,1)であれば誤差は0へ収束する
		実用上，corrRateを2などに設定すると少ない反復で早い収束が得られることもある
		*/
	real_t	corrRate;

	/** maximum error correction
		誤差修正量の上限．
		誤差修正量はこの値を超えないようにクリッピングされる．
		優先度レベルが複数ある場合に定常偏差を低減するにはこの値を小さ目に設定する．
		ただし小さくするほど収束スピードは低下する．
	 */
	real_t	corrMax;

	//vec3_t		e;			///< error value
	//vec3_t		de;			///< change of error
	//vec3_t		ded;		///< desired change of error

	vec3_t		y;			///< constraint error
	vec3_t		dy;			///< change of constraint error
	vec3_t		dyd;		///< desired change of constraint error
	vec3_t		l;			///< multiplier
	vec3_t		dl;			///< change of multiplier
	vec3_t		J, Jinv;	///< square sum of Jacobian row and its inverse

	/// テスト用
	ofstream	file;

public:	
	SLink*		AddSLink(Variable* var, real_t coef = 1.0);
	XLink*		AddXLink(Variable* var);
	CLink*		AddCLink(Variable* var, vec3_t coef = vec3_t((real_t)1.0, (real_t)1.0, (real_t)1.0));
	RLink*		AddRLink(Variable* var);

	/// reset internal variables
	void ResetState();

	/// preparation
	void CalcError();
	void CalcCorrection();
	void RegisterDeviation(vvec_t& yvec);

	/// steepest-descent
	void UpdateGradient(uint k);

	/// G-S related routines
	void UpdateMultiplier(uint k);
	void UpdateError1(uint k, real_t ddy);
	void UpdateError2(uint k, real_t _dy);
	void UpdateError3(uint k, real_t ddy);
	void UpdateConjugate(uint k);

public:
	/// virtual functions to be overridden by derived classes ///
	virtual void CalcCoef(){}
	virtual void CalcDeviation();

public:
	/// G-S related virtual functions
	
	/** do projection on multiplier
	 */
	virtual void Project(real_t& l, uint k){}
	
	Constraint(Solver* solver, uint n);
};

/** fixation constraint for scalar
 */
struct FixConS : Constraint{
	real_t	desired;
	virtual void CalcDeviation();
	FixConS(Solver* solver, SVar* var);
};
struct MatchConS : Constraint{
	MatchConS(Solver* solver, SVar* var0, SVar* var1);
};

/** fixation constraint for vector3
 */
struct FixConV3 : Constraint{
	vec3_t	desired;
	virtual void CalcDeviation();
	FixConV3(Solver* solver, V3Var* var);
};
struct MatchConV3 : Constraint{
	MatchConV3(Solver* solver, V3Var* var0, V3Var* var1);
};

/** fixation constraint for quaternion
 */
struct FixConQ : Constraint{
	quat_t	desired;
	virtual void CalcDeviation();
	FixConQ(Solver* solver, QVar* var);
};
/** fixation constraint for quaternion
 */
struct MatchConQ : Constraint{
	virtual void CalcDeviation();
	MatchConQ(Solver* solver, QVar* var0, QVar* var1);
};


/**	range constraint for scalar variables
 */
struct RangeConS : Constraint{
	real_t	_min, _max;
	bool	on_lower, on_upper;

	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	RangeConS(Solver* solver, SVar* var);
};

/**	range constraint for difference of scalar variables
 */
struct DiffConS : Constraint{
	real_t	_min, _max;
	bool	on_lower, on_upper;

	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	DiffConS(Solver* solver, SVar* var0, SVar* var1);
};

}
