#pragma once

#include "Constraint.h"
#include "Fix.h"

namespace TreeIK{;

class Body;

/// 干渉回避拘束．各剛体に割り当てられた形状（球・カプセル）が交差しないようにする
struct AvoidCon : BinaryCon{
	AvoidCon(Solver* solver, Body* _body0, Body* _body1);
};

}
