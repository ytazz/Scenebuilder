#include <sbcalc.h>
#include <sbmessage.h>
#include <sbtokenizer.h>

namespace Scenebuilder{;

real_t Calc::unit_length   = 1.0;
real_t Calc::unit_rotation = 1.0;
real_t Calc::unit_mass     = 1.0;
real_t Calc::unit_time     = 1.0;

Calc::Calc(){
	Reset();
}

void Calc::Reset(){
	terms.clear();
	root = cur = 0;
}

void Calc::SetUnitLength(string_iterator_pair str){
	unit_length = Calc()(str, Dimension::None);
}

void Calc::SetUnitRotation(string_iterator_pair str){
	unit_rotation = Calc()(str, Dimension::None);
}

void Calc::SetUnitMass(string_iterator_pair str){
	unit_mass = Calc()(str, Dimension::None);
}

void Calc::SetUnitTime(string_iterator_pair str){
	unit_time = Calc()(str, Dimension::None);
}

real_t Calc::ScaleFromUnit(string_iterator_pair unit){
	string_iterator_pair str0, str1;
	Tokenizer tok(unit, "/", true);

	// '/'で分割されている場合
	str0 = tok.GetToken();
	tok.Next();
	if(!tok.IsEnd()){
		str1 = tok.GetToken();
		real_t num = ScaleFromUnit(str0);
		real_t den = ScaleFromUnit(str1);
		return num/den;
	}

	// '^'で分割されている場合
	tok.Set(unit, "^", true);
	str0 = tok.GetToken();
	tok.Next();
	if(!tok.IsEnd()){
		str1 = tok.GetToken();
		real_t val = ScaleFromUnit(str0);
		real_t idx = to_int(str1);
		return pow(val, idx);
	}

	// 長さ
	if(unit == "km") return 1000.0;
	if(unit == "m" ) return    1.0;
	if(unit == "cm") return    0.01;
	if(unit == "mm") return    0.001;

	// 重さ
	if(unit == "kg") return 1.0;
	if(unit == "g" ) return 0.001;

	// 時間
	if(unit == "hour"  ) return 3600.0;
	if(unit == "h"     ) return 3600.0;
	if(unit == "min"   ) return   60.0;
	if(unit == "second") return    1.0;
	if(unit == "s"     ) return    1.0;

	// 角度
	if(unit == "rad") return 1.0;
	if(unit == "deg") return M_PI / 180.0;

	// 比率
	if(unit == "%") return 0.01;

	// 加速度
	const real_t G = 9.80665;
	if(unit == "G") return G;

	// 力
	if(unit == "N"  ) return 1.0;
	if(unit == "mN" ) return 0.001;
	if(unit == "kgf") return G;

	// モーメント
	if(unit == "Nm"   ) return 1.0;
	if(unit == "mNm"  ) return 0.001;
	if(unit == "kgfcm") return G * 0.01;

	throw CalcSyntaxError();
}

real_t Calc::ScaleFromDimension(int dim){
	real_t L = unit_length;
	real_t R = unit_rotation;
	real_t M = unit_mass;
	real_t T = unit_time;

	switch(dim){
	case Dimension::None   : return 1.0;
	case Dimension::L      : return L;
	case Dimension::R      : return R;
	case Dimension::M      : return M;
	case Dimension::T      : return T;
	case Dimension::L_T    : return L/T;
	case Dimension::L_TT   : return L/(T*T);
	case Dimension::R_T    : return R/T;
	case Dimension::ML_TT  : return (M*L)/(T*T);
	case Dimension::MLL_TT : return (M*L*L)/(T*T);
	case Dimension::MLL    : return M*L*L;
	case Dimension::M_T    : return M/T;
	case Dimension::M_TT   : return M/(T*T);
	case Dimension::MLL_RT : return (M*L*L)/(R*T);
	case Dimension::MLL_RTT: return (M*L*L)/(R*T*T);
	case Dimension::M_LLL  : return M/(L*L*L);
	}

	throw CalcSyntaxError();
}

real_t Calc::GetConstant(string_iterator_pair str){
	if(str == "pi") return M_PI;
	throw CalcSyntaxError();
}

Calc::math_func_t Calc::GetFunc(string_iterator_pair str){
	if(str == "sin" ) return &sin ;
	if(str == "cos" ) return &cos ;
	if(str == "tan" ) return &tan ;
	if(str == "exp" ) return &exp ;
	if(str == "log" ) return &log ;
	if(str == "sqrt") return &sqrt;
	throw CalcSyntaxError();
}

void Calc::SetVariable(const string& name, real_t val){
	vars[name] = val;
}

real_t Calc::GetVariable(const string& name){
	map<string, real_t>::iterator it = vars.find(name);
	if(it == vars.end())
		return 0.0;

	return it->second;
}

Calc::Term* Calc::CreateTerm(int type){
	UTRef<Term> term = new Term();
	term->type = type;

	terms.push_back(term);
	if(cur){
		cur->children.push_back(term);
		term->parent = cur;
	}
	else{
		term->parent = 0;
	}

	return term;
}

real_t Calc::operator()(string_iterator_pair eval, int dim){
	Reset();
	root = cur = CreateTerm(Op::Expr);
	
	string::const_iterator it, ibegin, iend;
	ibegin = eval.begin();
	iend   = eval.end();
	it = ibegin;
	while(it != iend){
		Term* prev = (cur->children.empty() ? 0 : cur->children.back());

		char c = *it;
		if(c == '+'){
			if(prev && (prev->type == Op::Num || prev->type == Op::Expr || prev->type == Op::Var))
				CreateTerm(Op::Add);
			else
				CreateTerm(Op::Plus);
			it++;
			continue;
		}
		if(c == '-'){
			if(prev && (prev->type == Op::Num || prev->type == Op::Expr || prev->type == Op::Var))
				CreateTerm(Op::Sub);
			else
				CreateTerm(Op::Minus);
			it++;
			continue;
		}
		if(c == '!'){
			CreateTerm(Op::Not);
			it++;
			continue;
		}
		if(c == '*'){
			if(prev && (prev->type == Op::Num || prev->type == Op::Expr || prev->type == Op::Var))
				CreateTerm(Op::Mul);
			else
				throw CalcSyntaxError();
			it++;
			continue;
		}
		if(c == '/'){
			if(prev && (prev->type == Op::Num || prev->type == Op::Expr || prev->type == Op::Var))
				CreateTerm(Op::Div);
			else
				throw CalcSyntaxError();
			it++;
			continue;
		}
		// logical OR
		//  it could be either | or || (bitwise OR is not supported)
		if(c == '|'){
			if(it+1 != iend && *(it+1) == '|')
				it++;
			
			if(prev && (prev->type == Op::Num || prev->type == Op::Expr || prev->type == Op::Var))
				CreateTerm(Op::Or);
			else
				throw CalcSyntaxError();
			it++;
			continue;
		}
		// logical AND
		//  it could be either & or && (bitwise AND is not supported)
		if(c == '&'){
			if(it+1 != iend && *(it+1) == '&')
				it++;
			
			if(prev && (prev->type == Op::Num || prev->type == Op::Expr || prev->type == Op::Var))
				CreateTerm(Op::And);
			else
				throw CalcSyntaxError();
			it++;
			continue;
		}
		if(c == '^'){
			if(prev && (prev->type == Op::Num || prev->type == Op::Expr || prev->type == Op::Var))
				CreateTerm(Op::Pow);
			else
				throw CalcSyntaxError();
			it++;
			continue;
		}
		if(c == '('){
			cur = CreateTerm(Op::Expr);
			it++;
			continue;
		}
		if(c == ')'){
			if(!cur->parent)
				throw CalcSyntaxError();
			cur = cur->parent;
			it++;
			continue;
		}
		if(c == '$'){
			it++;
			if(it == iend || *it != '(')
				throw SyntaxError();
			it++;
			string_iterator_pair strval;
			strval.first = it;
			while(it != iend && (isalpha(*it) || isdigit(*it)))
				it++;
			strval.second = it;
			if(it == iend || *it != ')')
				throw SyntaxError();

			Term* t = CreateTerm(Op::Var);
			t->varname = string(strval.first, strval.second);
			it++;
			continue;
		}
		if(isdigit(c) || c == '.'){
			// [it0, it1) 数値部, [it1, it2) 単位部
			string_iterator_pair strval;
			strval.first = it;

			// 小数点の上 (0桁以上)
			while(it != iend && isdigit(*it))
				it++;
			// 小数点
			if(it != iend && *it == '.'){
				it++;
				// 小数点の下 (1桁以上)
				if(it == iend || !isdigit(*it))
					throw CalcSyntaxError();
				while(it != iend && isdigit(*it))
					it++;
			}
			// 累乗部分
			if(it != iend && *it == 'e'){
				it++;
				if(it != iend && (*it == '+' || *it == '-'))
					it++;
				
				// 指数部 (1桁以上)
				if(it == iend)
					throw CalcSyntaxError();
				if(!isdigit(*it))
					throw CalcSyntaxError();
				while(it != iend && isdigit(*it))
					it++;
			}
			strval.second = it;
			
			Term* t = CreateTerm(Op::Num);
			t->val = to_real(strval);
			continue;
		}
		if(isalpha(c) || c == '[' || c == ']' || c == '%'){
			string::const_iterator it0, it1;
			it0 = it;

			// - '['で始まる場合は']'までを単位とする
			// - それ以外は(isalpha | '%')を単位とする
			bool isUnit = false;
			if(it != iend && *it == '['){
				it++;
				it0 = it;

				while(it != iend && *it != ']')
					it++;
				it1 = it;
				if(it != iend)
					it++;
				isUnit = true;
			}
			else{
				it0 = it;
				while(it != iend && (isalpha(*it) || *it == '%'))
					it++;
				it1 = it;
			}
			
			string str(it0, it1);

			// 単位かどうかを優先的に調べる
			try{
				real_t unit = ScaleFromUnit(str);
				Term* t = CreateTerm(Op::Unit);
				t->val = unit;
				continue;
			}
			catch(Exception&){
				if(isUnit)
					throw CalcSyntaxError();
			}

			// 定数
			try{
				real_t val = GetConstant(str);
				Term* t = CreateTerm(Op::Num);
				t->val = val;
				continue;
			}
			catch(Exception&){}

			// 関数
			try{
				double (*func)(double) = GetFunc(str);
				Term* t = CreateTerm(Op::Func);
				t->func = func;
				continue;
			}
			catch(Exception&){}

			throw CalcSyntaxError();
		}
		else{
			throw CalcSyntaxError();
		}
	}

	// 数式を評価
	bool unitSpecified = false;
	real_t val = Eval(root, &unitSpecified);

	// 単位が明示されていない場合は物理的次元からスケールを決定
	if(!unitSpecified)
		val *= ScaleFromDimension(dim);

	return val;
}

real_t Calc::Eval(Calc::Term* term, bool* unitSpecified){
	if(term->children.empty())
		throw CalcSyntaxError();

	// まず()式を評価して数値にする
	vector<Term*>::iterator it;
	for(it = term->children.begin(); it != term->children.end(); it++){
		Term* child = *it;
		if(child->type == Op::Expr){
			child->val = Eval(child);
			child->type = Op::Num;
		}
	}

	// substitute values to variables
	for(it = term->children.begin(); it != term->children.end(); it++){
		Term* child = *it;
		if(child->type == Op::Var){
			child->val = GetVariable(child->varname);
			child->type = Op::Num;
		}
	}

	// 関数
	for(it = term->children.begin(); it != term->children.end(); ){
		Term* child = *it;
		Term* next  = (it+1 == term->children.end() ? 0 : *(it+1));
		if(child->type == Op::Func){
			if(!next || next->type != Op::Num)
				throw CalcSyntaxError();
			child->val = child->func(next->val);
			child->type = Op::Num;
			it = term->children.erase(it+1);
			continue;
		}
		it++;
	}

	// 単項演算子 後ろから走査して処理
	for(it = term->children.begin(); it != term->children.end(); ){
		Term* child = *it;
		Term* prev  = (it == term->children.begin() ? 0 : *(it-1));
		Term* next  = (it+1 == term->children.end() ? 0 : *(it+1));

		// +符号は消すのみ
		if(child->type == Op::Plus){
			it = term->children.erase(it);
			continue;
		}
		if(child->type == Op::Minus){
			// 次の項が無いのはエラー
			if(!next)
				throw CalcSyntaxError();
			// 次の項が数値なら符号を反転し，符号自体は消す
			if(next->type == Op::Num){
				next->val = -next->val;
				it = term->children.erase(it);
				continue;
			}
			// 次の項が符号なら，次の符号を反転し，この符号は消す
			if(next->type == Op::Plus){
				next->type = Op::Minus;
				it = term->children.erase(it);
				continue;
			}
			if(next->type == Op::Minus){
				next->type = Op::Plus;
				it = term->children.erase(it);
				continue;
			}
			// それ以外はエラー
			throw CalcSyntaxError();
		}
		if(child->type == Op::Not){
			if(!next)
				throw CalcSyntaxError();
			if(next->type == Op::Num){
				next->val = !next->val;
				it = term->children.erase(it);
				continue;
			}
			throw CalcSyntaxError();
		}
		// 次へ
		it++;
	}

	// 二項演算子 ^
	for(it = term->children.begin(); it != term->children.end(); ){
		Term* child = *it;
		Term* prev  = (it == term->children.begin() ? 0 : *(it-1));
		Term* next  = (it+1 == term->children.end() ? 0 : *(it+1));

		if(child->type == Op::Pow){
			if(!(prev && next && prev->type == Op::Num && next->type == Op::Num))
				throw CalcSyntaxError();

			prev->val = pow(prev->val, next->val);
			it = term->children.erase(it);
			it = term->children.erase(it);
			continue;
		}
		it++;
	}

	// 二項演算子 * /
	for(it = term->children.begin(); it != term->children.end(); ){
		Term* child = *it;
		Term* prev  = (it == term->children.begin() ? 0 : *(it-1));
		Term* next  = (it+1 == term->children.end() ? 0 : *(it+1));

		if(child->type == Op::Mul){
			// 両隣は数値項でないとエラー
			if(!(prev && next && prev->type == Op::Num && next->type == Op::Num))
				throw CalcSyntaxError();
			// 前の数値項を演算結果に置換し，演算子と後ろの数値項は削除
			prev->val = prev->val * next->val;
			it = term->children.erase(it);
			it = term->children.erase(it);
			continue;
		}
		if(child->type == Op::Div){
			// 両隣は数値項でないとエラー
			if(!(prev && next && prev->type == Op::Num && next->type == Op::Num))
				throw CalcSyntaxError();
			// 前の数値項を演算結果に置換し，演算子と後ろの数値項は削除
			//  零除算チェック
			if(next->val == 0.0)
				throw CalcSyntaxError();
			prev->val = prev->val / next->val;
			it = term->children.erase(it);
			it = term->children.erase(it);
			continue;
		}
		it++;
	}

	// 二項演算子 + -
	for(it = term->children.begin(); it != term->children.end(); ){
		Term* child = *it;
		Term* prev  = (it == term->children.begin() ? 0 : *(it-1));
		Term* next  = (it+1 == term->children.end() ? 0 : *(it+1));

		if(child->type == Op::Add){
			// 両隣は数値項でないとエラー
			if(!(prev && next && prev->type == Op::Num && next->type == Op::Num))
				throw CalcSyntaxError();
			// 前の数値項を演算結果に置換し，演算子と後ろの数値項は削除
			prev->val = prev->val + next->val;
			it = term->children.erase(it);
			it = term->children.erase(it);
			continue;
		}
		if(child->type == Op::Sub){
			// 両隣は数値項でないとエラー
			if(!(prev && next && prev->type == Op::Num && next->type == Op::Num))
				throw CalcSyntaxError();
			// 前の数値項を演算結果に置換し，演算子と後ろの数値項は削除
			prev->val = prev->val - next->val;
			it = term->children.erase(it);
			it = term->children.erase(it);
			continue;
		}
		it++;
	}
	// || &&
	for(it = term->children.begin(); it != term->children.end(); ){
		Term* child = *it;
		Term* prev  = (it == term->children.begin() ? 0 : *(it-1));
		Term* next  = (it+1 == term->children.end() ? 0 : *(it+1));

		if(child->type == Op::Or){
			if(!(prev && next && prev->type == Op::Num && next->type == Op::Num))
				throw CalcSyntaxError();
			
			prev->val = (real_t)((prev->val != 0.0) || (next->val != 0.0));
			it = term->children.erase(it);
			it = term->children.erase(it);
			continue;
		}
		if(child->type == Op::And){
			if(!(prev && next && prev->type == Op::Num && next->type == Op::Num))
				throw CalcSyntaxError();
			
			prev->val = (real_t)((prev->val != 0.0) && (next->val != 0.0));
			it = term->children.erase(it);
			it = term->children.erase(it);
			continue;
		}
		it++;
	}

	// 単位
	for(it = term->children.begin(); it != term->children.end(); ){
		Term* child = *it;
		Term* prev  = (it == term->children.begin() ? 0 : *(it-1));
		
		if(child->type == Op::Unit){
			// 直前が数値ならそれに単位の係数をかける
			if(prev && prev->type == Op::Num){
				prev->val *= child->val;
				it = term->children.erase(it);
			}
			// 数値がない場合は'1'だと思って単位を数値に変換
			else{
				child->type = Op::Num;
			}
			if(unitSpecified)
				*unitSpecified = true;
			continue;
		}
		it++;
	}

	// ここまで来て単一の数値項が残っていればそれが演算結果，それ以外はエラー
	if(term->children.size() == 1 && term->children[0]->type == Op::Num)
		return term->children[0]->val;
	throw CalcSyntaxError();
}

}