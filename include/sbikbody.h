#pragma once

#include <sbtypes.h>
#include <sbvariable.h>
#include <sbconstraint.h>

#include <vector>
using namespace std;

#include <SprGraphics.h>
using namespace Spr;

namespace Scenebuilder{;

class IKSolver;

/// 剛体
class IKBody : public UTRefCount{
public:
	struct JointType{
		enum{
			Hinge,
			Slider,
			Balljoint,
			Fixjoint,
		};
	};

	IKSolver*           ikSolver;
	IKBody*				parent;
	vector< IKBody* >	children;
	uint                type;
	uint                ndof;           ///< 関節の自由度
	bool                revolutive[3];  ///< 回転自由度ならtrue 直動自由度ならfalse
	bool                handled;	    ///< Handleにつながる鎖上にあればtrue

	pose_t	psock;				///< 親剛体に対するソケットの位置と向き
	pose_t	pplug;				///< この剛体に対するプラグの位置と向き
	
	pose_t	pose;				///< グローバル座標上の位置と向き
	pose_t	prel;				///< ソケットに対するプラグの位置と向き
	vec3_t	pivot;				///< グローバル座標上の関節の軸中心
	vec3_t	Jv[3], Jv_abs[3];	///< ヤコビアン（並進）
	vec3_t  Jw[3], Jw_abs[3];	///< ヤコビアン（回転）
	
	SVar*       pos[3];         ///< 関節変位（ヒンジ，スライダ）
	RangeConS*  range[3];       ///< 関節角度の範囲拘束

public:
	void    AddVars         ();
	void    DeleteVars      ();
	void    AddCons         ();
	void    DeleteCons      ();
	void    MarkHandled     (IKBody* ref);
	void    CalcJacobian    ();
	void    CalcRelativePose();
	void	Prepare         ();
	void	CompFK          ();

public:
	IKBody*	GetParent();
	void    SetParent(IKBody* par, int _type);

	/// 親リンクに対する相対的な位置と向きを設定
	void SetSocketPose(const pose_t& p);
	void SetPlugPose(const pose_t& p);

	void GetSocketPose(pose_t& p);
	void GetPlugPose(pose_t& p);

	/// 位置と向きの取得
	void SetPose(const pose_t& p);
	void GetPose(pose_t& p);

	/// 関節角度を取得
	real_t GetJointPos(uint i);
	
	/// 関節角度を設定
	void SetJointPos(uint i, real_t _pos);
	
	/// 関節角度を固定
	void LockPos(uint i, bool lock = true);
	
	/// 関節角度の範囲拘束を設定
	void SetPosLimit(uint i, real_t lower, real_t upper);

	void Draw(GRRenderIf* render);

	IKBody(IKSolver* _ikSolver);
};

}