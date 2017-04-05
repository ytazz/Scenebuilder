#pragma once

#include <sbvariable.h>
#include <sbconstraint.h>

namespace Scenebuilder{;

class Variable;
class Constraint;

/**
	Link: Jacobian matrix from variable to constraint error
 */
class Link : public UTRefCount{
public:
	Variable*		var;
	Constraint*		con;
	
	void Connect();

	/// norm
	virtual real_t Norm() = 0;
	
	/// add norm of rows of Jacobian
	virtual void AddRowSqr(vec3_t& v) = 0;

	/// add norm of columns of Jacobian
	virtual void AddColSqr(vec3_t& v) = 0;

	/// x -> c(x)
	virtual void AddError() = 0;

	virtual void Forward (uint k, real_t d, Constraint::UpdateFunc func) = 0;
	virtual void Backward(uint k, real_t d, Variable  ::UpdateFunc func) = 0;

	Link(Variable* v, Constraint* c):var(v), con(c){}
};

typedef std::vector< UTRef<Link> >	LinkRefs;

/** link with scalar coefficient
	y = c(x) = k * x
	k is scalar
	x is scalar or vector3
	y is scalar or vector3
 */
class SLink : public Link{
public:
	real_t coef, coefsqr;
	
	void SetCoef(real_t k);

	virtual real_t Norm(){ return std::abs(coef); }
	virtual void AddRowSqr(vec3_t& v);
	virtual void AddColSqr(vec3_t& v);
	virtual void AddError ();
	virtual void Forward  (uint k, real_t d, Constraint::UpdateFunc func);
	virtual void Backward (uint k, real_t d, Variable  ::UpdateFunc func);

	SLink(Variable* v, Constraint* c, real_t k);
};

class V3Link : public Link{
public:
	vec3_t	coef, coefsqr;

	void SetCoef(vec3_t k);

	virtual real_t Norm(){ return coef.norm(); }

	V3Link(Variable* v, Constraint* c,vec3_t k = vec3_t(1.0,1.0,1.0)):Link(v, c){}
};

/** link with cross product matrix
	y = c(x) = k % x
	k is vector3
	x is vector3
	y is vector3
 */
class XLink : public V3Link{
public:
	virtual void AddRowSqr(vec3_t& v);
	virtual void AddColSqr(vec3_t& v);
	virtual void AddError ();
	virtual void Forward  (uint k, real_t d, Constraint::UpdateFunc func);
	virtual void Backward (uint k, real_t d, Variable  ::UpdateFunc func);

	XLink(Variable* v, Constraint* c):V3Link(v, c){}
};

/** link between vec3 constraint and scalar variable
	y = c(x) = k * x
	k is vector3
	x is scalar
	y is vector3

	* C stands for column
 */
class CLink : public V3Link{
public:

public:
	virtual void AddRowSqr(vec3_t& v);
	virtual void AddColSqr(vec3_t& v);
	virtual void AddError();
	virtual void Forward(uint k, real_t d, Constraint::UpdateFunc func);
	virtual void Backward(uint k, real_t d, Variable::UpdateFunc func);

	CLink(Variable* v, Constraint* c,vec3_t k):V3Link(v, c){}
};

/** link between scalar constraint and vec3 variable
	y = c(x) = k * x	(inner product)

	R stands for row
 */
class RLink : public V3Link{
public:
	virtual void AddRowSqr(vec3_t& v);
	virtual void AddColSqr(vec3_t& v);
	virtual void AddError ();
	virtual void Forward  (uint k, real_t d, Constraint::UpdateFunc func);
	virtual void Backward (uint k, real_t d, Variable  ::UpdateFunc func);

	RLink(Variable* v, Constraint* c):V3Link(v, c){}
};

}
