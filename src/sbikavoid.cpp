#include "Fix.h"
#include "Body.h"
#include "Link.h"

namespace TreeIK{;

//-------------------------------------------------------------------------------------------------
// constructors

FixCon::FixCon(Solver* _solver, Body* _body):Constraint(_solver, 3){
	body = _body;
}

FixConPos::FixConPos(Solver* _solver, Body* _body, vec3_t _pos, vec3_t _desired):FixCon(_solver, _body){
	pos = _pos;
	desired = _desired;
	for(Body* b = body; b; b = b->parent)
		AddCLink(b->pos);
}

FixConOri::FixConOri(Solver* _solver, Body* _body, quat_t _desired):FixCon(_solver, _body){
	desired = _desired;

	for(Body* b = body; b; b = b->parent)
		AddCLink(b->pos);
}

//-------------------------------------------------------------------------------------------------
// CalcCoef

void FixConPos::CalcCoef(){
	pos_abs = body->pose * pos;

	int i;
	Body* b;
	for(b = body, i = 0; b; b = b->parent, i++){
		((CLink*)links[i])->SetCoef(-1.0 * b->axis % (pos_abs - b->pivot));
	}
}

void FixConOri::CalcCoef(){
	int i;
	Body* b;
	for(b = body, i = 0; b; b = b->parent, i++){
		((CLink*)links[i])->SetCoef(-1.0 * b->axis);
	}
}

//-------------------------------------------------------------------------------------------------
// CalcError

void FixConPos::CalcDeviation(){
	y = desired - pos_abs;
}

void FixConOri::CalcDeviation(){
	quat_t q0 = desired;
	quat_t q1 = body->pose.Ori();
	quat_t qerror = q0.Conjugated() * q1;
	y = q0 * (qerror.Theta() * qerror.Axis());
}

}
