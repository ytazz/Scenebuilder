#pragma once

#include <sbtypes.h>

/* 文字列のトークン分解

 */

namespace Scenebuilder{;

template<typename T>
class TokenizerBase{
public:
	basic_string_iterator_pair<T> str;
	basic_string_iterator_pair<T> token;
	basic_string<T>               delim;
	bool                          compress;
	bool                          skip_par;

	bool IsDelim(T c){
		for(uint i = 0; i < delim.size(); i++)
			if(delim[i] == c)
				return true;
		return false;
	}

	void SkipParenthesis(){
		int depth = 0;
		while(token.second != str.second){
			if(*token.second == (T)('('))
				depth++;
			if(*token.second == (T)(')'))
				depth--;
			token.second++;
			if(!skip_par || depth == 0)
				break;
		}
	}

public:
	static void Split(vector< basic_string<T> >& tokens, basic_string_iterator_pair<T> line, const basic_string<T>& d, bool comp, bool skip = false){
		tokens.clear();
		TokenizerBase<T> tok(line, d, comp, skip);
		while(!tok.IsEnd()){
			tokens.push_back(to_string(tok.GetToken()));
			tok.Next();
		}
	}

	void Set(basic_string_iterator_pair<T> s, const basic_string<T>& d, bool comp, bool skip = false){
		str      = s;
		delim    = d;
		compress = comp;
		skip_par = skip;
		token.first = token.second = str.first;

		while((token.second != str.second) && !IsDelim(*token.second))
			SkipParenthesis();
	}

	basic_string_iterator_pair<T> GetToken(){
		return token;
	}

	void Next(){
		token.first = token.second;
		if(IsEnd())
			return;

		do{
			token.first++;
		}
		while(compress && (token.first != str.second) && IsDelim(*token.first));
	
		token.second = token.first;
		while((token.second != str.second) && !IsDelim(*token.second))
			SkipParenthesis();
	}

	bool IsEnd(){
		return (token.first == str.second);
	}

	TokenizerBase(){
		delim.resize(1);
		delim[0] = (T)(' ');
		compress = false;
		skip_par = false;
	}

	TokenizerBase(basic_string_iterator_pair<T> s, const basic_string<T>& d, bool comp, bool skip = false){
		Set(s, d, comp, skip);
	}
};

typedef TokenizerBase<char>    Tokenizer;
typedef TokenizerBase<wchar_t> TokenizerW;

}
