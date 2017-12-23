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
	bool			active;

	void Connect();

	/// add norm of rows of Jacobian
	virtual void AddRowSqr(vec3_t& v) = 0;

	/// add norm of columns of Jacobian
	virtual void AddColSqr(vec3_t& v) = 0;

	/// x -> c(x)
	virtual void AddError() = 0;

	virtual void RegisterCoef(vmat_t& J) = 0;

	/// con->func(J.col(k) * d)
	virtual void Col     (uint k, real_t  d, Constraint::UpdateFunc func) = 0;
	/// con->func(J.row(k) * d)
	virtual void Row     (uint k, vec3_t& d, Constraint::UpdateFunc func) = 0;
	/// var->func(J.col(k) * d)
	virtual void ColTrans(uint k, real_t  d, Variable  ::UpdateFunc func) = 0;

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

	virtual void AddRowSqr   (vec3_t& v);
	virtual void AddColSqr   (vec3_t& v);
	virtual void AddError    ();
	virtual void RegisterCoef(vmat_t& J);
	virtual void Col         (uint k, real_t  d, Constraint::UpdateFunc func);
	virtual void Row         (uint k, vec3_t& d, Constraint::UpdateFunc func);
	virtual void ColTrans    (uint k, real_t  d, Variable  ::UpdateFunc func);

	SLink(Variable* v, Constraint* c, real_t k);
};

class V2Link : public Link{
public:
	vec2_t	coef, coefsqr;

	void SetCoef(vec2_t k);

	V2Link(Variable* v, Constraint* c):Link(v, c){}
};

class V3Link : public Link{
public:
	vec3_t	coef, coefsqr;

	void SetCoef(vec3_t k);

	V3Link(Variable* v, Constraint* c):Link(v, c){}
};

/** link with cross product matrix
	y = c(x) = k % x
	k is vector3
	x is vector3
	y is vector3
 */
class X3Link : public V3Link{
public:
	virtual void AddRowSqr   (vec3_t& v);
	virtual void AddColSqr   (vec3_t& v);
	virtual void AddError    ();
	virtual void RegisterCoef(vmat_t& J);
	virtual void Col         (uint k, real_t  d, Constraint::UpdateFunc func);
	virtual void Row         (uint k, vec3_t& d, Constraint::UpdateFunc func);
	virtual void ColTrans    (uint k, real_t  d, Variable  ::UpdateFunc func);

	X3Link(Variable* v, Constraint* c):V3Link(v, c){}
};

/** link between vec2/3 constraint and scalar variable
	y = c(x) = k * x
	k is vector2/3
	x is scalar
	y is vector2/3

	* C stands for column
 */
class C2Link : public V2Link{
public:

public:
	virtual void AddRowSqr   (vec3_t& v);
	virtual void AddColSqr   (vec3_t& v);
	virtual void AddError    ();
	virtual void RegisterCoef(vmat_t& J);
	virtual void Col         (uint k, real_t  d, Constraint::UpdateFunc func);
	virtual void Row         (uint k, vec3_t& d, Constraint::UpdateFunc func);
	virtual void ColTrans    (uint k, real_t  d, Variable  ::UpdateFunc func);

	C2Link(Variable* v, Constraint* c):V2Link(v, c){}
};
class C3Link : public V3Link{
public:

public:
	virtual void AddRowSqr   (vec3_t& v);
	virtual void AddColSqr   (vec3_t& v);
	virtual void AddError    ();
	virtual void RegisterCoef(vmat_t& J);
	virtual void Col         (uint k, real_t  d, Constraint::UpdateFunc func);
	virtual void Row         (uint k, vec3_t& d, Constraint::UpdateFunc func);
	virtual void ColTrans    (uint k, real_t  d, Variable  ::UpdateFunc func);

	C3Link(Variable* v, Constraint* c):V3Link(v, c){}
};

/** link between scalar constraint and vec2/3 variable
	y = c(x) = k * x	(inner product)

	R stands for row
 */
class R2Link : public V2Link{
public:
	virtual void AddRowSqr   (vec3_t& v);
	virtual void AddColSqr   (vec3_t& v);
	virtual void AddError    ();
	virtual void RegisterCoef(vmat_t& J);
	virtual void Col         (uint k, real_t  d, Constraint::UpdateFunc func);
	virtual void Row         (uint k, vec3_t& d, Constraint::UpdateFunc func);
	virtual void ColTrans    (uint k, real_t  d, Variable  ::UpdateFunc func);

	R2Link(Variable* v, Constraint* c):V2Link(v, c){}
};
class R3Link : public V3Link{
public:
	virtual void AddRowSqr   (vec3_t& v);
	virtual void AddColSqr   (vec3_t& v);
	virtual void AddError    ();
	virtual void RegisterCoef(vmat_t& J);
	virtual void Col         (uint k, real_t  d, Constraint::UpdateFunc func);
	virtual void Row         (uint k, vec3_t& d, Constraint::UpdateFunc func);
	virtual void ColTrans    (uint k, real_t  d, Variable  ::UpdateFunc func);

	R3Link(Variable* v, Constraint* c):V3Link(v, c){}
};

/** general linear map
	y = A x
 */
class M2Link : public Link{
public:
	mat2_t	coef;
	mat2_t	coefsqr;

public:
	void SetCoef(const mat2_t& m);

	virtual void AddRowSqr   (vec3_t& v);
	virtual void AddColSqr   (vec3_t& v);
	virtual void AddError    ();
	virtual void RegisterCoef(vmat_t& J);
	virtual void Col         (uint k, real_t  d, Constraint::UpdateFunc func);
	virtual void Row         (uint k, vec3_t& d, Constraint::UpdateFunc func);
	virtual void ColTrans    (uint k, real_t  d, Variable  ::UpdateFunc func);
	
	//vec2_t Backward (vec2_t v);

	M2Link(Variable* v, Constraint* c):Link(v, c){}
};

class M3Link : public Link{
public:
	mat3_t	coef;
	mat3_t	coefsqr;

public:
	void SetCoef(const mat3_t& m);

	virtual void AddRowSqr   (vec3_t& v);
	virtual void AddColSqr   (vec3_t& v);
	virtual void AddError    ();
	virtual void RegisterCoef(vmat_t& J);
	virtual void Col         (uint k, real_t  d, Constraint::UpdateFunc func);
	virtual void Row         (uint k, vec3_t& d, Constraint::UpdateFunc func);
	virtual void ColTrans    (uint k, real_t  d, Variable  ::UpdateFunc func);
	
	//vec3_t Backward (vec3_t v);

	M3Link(Variable* v, Constraint* c):Link(v, c){}
};

}
