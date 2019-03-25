#pragma once

#include <sbtypes.h>

#include <map>

namespace Scenebuilder{;

/** 計算式を文字列で受け取り数値に変換
	単位の扱い：
	- 変換後はSI単位とする：
		- L -> meter
		- M -> kilogram
		- T -> second
	- 式に現れる数値は，
		- 単位未指定の場合，設定された物理量の次元に対応するデフォルト単位からSI単位への変換係数がかけられる
		- 単位指定の場合，その単位からSI単位への変換係数がかけられる
 **/

/// 物理量の次元
struct Dimension{
	enum {
		None,		//< 無次元量
		L,			//< 長さ
		R,			//< 回転
		M,			//< 質量
		T,			//< 時間
		L_T,		//< 速度
		L_TT,		//< 加速度
		R_T,		//< 角速度
		R_TT,       //< 角加速度
		ML_TT,		//< 力
		MLL_TT,     //< モーメント
		MLL,		//< 慣性モーメント
		M_T,		//< 直動ダンパ
		M_TT,		//< 直動バネ
		MLL_RT,		//< 回転ダンパ
		MLL_RTT,	//< 回転バネ
		M_LLL,		//< 密度
	};
};

/** Calc

	数式の計算機能
 */
class Calc{
protected:
	/// 演算子や関数，定数
	struct Op{
		enum{
			Plus,	///< 正符号
			Minus,	///< 負符号
			Add,	///< 加算
			Sub,	///< 減算
			Mul,	///< 乗算
			Div,	///< 除算
			Not,
			Or,
			And,
			Pow,	///< 指数
			Num,	///< 数値
			Var,
			Expr,	///< 式
			Func,	///< 関数
			Unit,	///< 単位
		};
	};

	typedef double (*math_func_t)(double);
	
	struct Term : UTRefCount{
		int		    type;
		real_t	    val;
		string      varname;
		math_func_t func;

		Term* parent;
		vector<Term*>	children;
	};

	static real_t	unit_length;
	static real_t	unit_rotation;
	static real_t	unit_mass;
	static real_t	unit_time;

	vector< UTRef<Term> >	terms;
	Term*	root;
	Term*	cur;

	std::map<string, real_t>  vars;

	void	Reset();
	Term*	CreateTerm(int type);
	real_t	Eval(Term* term, bool* unitSpecified = 0);

public:
	/// デフォルト単位を指定
	static void SetUnitLength  (string_iterator_pair str);
	static void SetUnitRotation(string_iterator_pair str);
	static void SetUnitMass    (string_iterator_pair str);
	static void SetUnitTime    (string_iterator_pair str);

	/// 単位からスケールを取得
	static real_t	ScaleFromUnit(string_iterator_pair unit);

	/// 物理量の次元からスケールを取得
	static real_t	ScaleFromDimension(int dim);

	/// 定数値を取得
	static real_t  GetConstant(string_iterator_pair str);

	/// 関数を取得
	static math_func_t GetFunc(string_iterator_pair str);

    /// register variable
	void    SetVariable(const string& name, real_t val);
	real_t  GetVariable(const string& name);

	/** @brief 式を計算

		@param eval	式
		@param dim  単位未指定の数値の物理的次元
	 */
	real_t operator()(string_iterator_pair eval, int dim = Dimension::None);

	Calc();
};

}
