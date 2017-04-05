#include <sbtypes.h>

#include <boost/lexical_cast.hpp>
using namespace boost;

namespace Scenebuilder{;

int to_int(string_iterator_pair str){
	try{
		return lexical_cast<int>(to_string(eat_white(str)));
	}
	catch(bad_lexical_cast){
		throw SyntaxError();
	}
}

real_t to_real(string_iterator_pair str){
	try{
		return lexical_cast<real_t>(to_string(eat_white(str)));
	}
	catch(bad_lexical_cast){
		throw SyntaxError();
	}
}

string to_string(string_iterator_pair str){
	return string(str.begin(), str.end());
}

wstring to_string(wstring_iterator_pair str){
	return wstring(str.begin(), str.end());
}

}
