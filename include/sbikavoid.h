#pragma once

#include "Constraint.h"
#include "Fix.h"

namespace TreeIK{;

class Body;

/// ������S���D�e���̂Ɋ��蓖�Ă�ꂽ�`��i���E�J�v�Z���j���������Ȃ��悤�ɂ���
struct AvoidCon : BinaryCon{
	AvoidCon(Solver* solver, Body* _body0, Body* _body1);
};

}
