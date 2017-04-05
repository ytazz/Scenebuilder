#pragma once

#include <sbconstraint.h>

#include <SprGraphics.h>
using namespace Spr;

namespace Scenebuilder{;

class IKSolver;
class IKBody;
class FixPosCon;
class FixOriCon;

class IKHandle : public UTRefCount{
public:
	IKSolver* ikSolver;
	IKBody*	  endBody;              ///< エンドボディ（ハンドルを取り付けるボディ）
	IKBody*	  refBody;              ///< 参照ボディ　（基準フレームを取り付けるボディ）
	pose_t	  endPose, endPoseAbs;  ///< エンドボディから見たハンドルの位置と向き
	pose_t	  refPose, refPoseAbs;  ///< 参照ボディから見た基準フレームの位置と向き
	pose_t	  desPose;              ///< 基準フレームに対するハンドルの位置と向きの目標値
	pose_t    curPose;              ///< 基準フレームに対するハンドルの位置と向きの現在値

	bool      enablePos;            ///< 位置の拘束の有効性
	bool      enableOri;            ///< 向きの拘束の有効性

	FixPosCon*	con_fix_pos;
	FixOriCon*	con_fix_ori;

public:
	void AddCons   ();
	void DeleteCons();

public:
	void SetEndPose(const pose_t& p);
	void SetRefPose(const pose_t& p);

	void GetEndPose(pose_t& p);
	void GetRefPose(pose_t& p);

	void SetDesiredPose(const pose_t& p);
	void GetDesiredPose(pose_t& p);
	void GetCurrentPose(pose_t& p);

	void EnablePos(bool on = true);
	void EnableOri(bool on = true);

	void Prepare();
	void CompFK ();
	void Draw   (GRRenderIf* render);

	IKHandle(IKSolver* _ikSolver, IKBody* _endBody, IKBody* _refBody);
};

class FixPosCon : public Constraint{
public:
	IKHandle* handle;

	virtual void CalcCoef();
	virtual void CalcDeviation();

	FixPosCon(IKSolver* _solver, IKHandle* h);
};

class FixOriCon : public Constraint{
public:
	IKHandle* handle;

	virtual void CalcCoef();
	virtual void CalcDeviation();
	
	FixOriCon(IKSolver* _solver, IKHandle* h);
};

}
