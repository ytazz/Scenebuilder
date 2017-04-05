#pragma once

#include "Fix.h"

namespace TreeIK{;

class Body;

struct PlaneCon : BinaryCon{
	enum{
		Equal,
		Greater,
		Less,
	};
	vec3_t pos;
	vec3_t dir;
	real_t offset;
	int mode;

	PlaneCon(Solver* _solver, Body* _body, Body* _ref);
};

}
