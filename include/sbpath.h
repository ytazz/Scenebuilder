#pragma once

#include <sbtypes.h>

#include <string>
using namespace std;

namespace Scenebuilder{;

class Path : public string{
protected:
	void ReplaceDelim();

public:
	// 
	static char        Delim (){ return '\\'; }
	static const char* Delims(){ return "/\\"; }

	// extract file extension without period
	string Ext() const;

	// extract file name part
	string File() const;

	// extract directory part without terminal delimitor
	string Dir() const;

	// drive letter without colon (for windows)
	string Drive() const;

	// returns true if path points to a file
	// 有効なパスでないばあいはfalseが返るので注意
	bool IsFile() const;

	// returns true if path points to a directory
	// 有効なパスでないばあいはfalseが返るので注意
	bool IsDir() const;

	// returns true if absolute path
	bool IsAbsolute() const;

	// returns true is relative path
	bool IsRelative() const;

	// creates current directory path
	static Path Current();

	Path& operator=(const char*   p);
	Path& operator=(const string& p);

	Path(){}
	Path(const string& s);
};

// デリミタを付与して連結
Path operator+(const Path& p0, const Path& p1);

}
