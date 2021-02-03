#pragma once

/**
	コンソール操作
	- WinAPIのオーバヘッドが大きいので自前のバッファにテキストを保持し，
	  Refreshで一括して更新する
 */

#include <sbtypes.h>

namespace Scenebuilder{;

class Console{
public:
	static int    numRows;
	static int    numColumns;
	static string buffer;

public:
	static void SetFontSize      (int sz);
	static void SetBufferSize    (int nrow, int ncol);
	static void SetDisplaySize   (int nrow, int ncol);
	static void SetWindowPosition(int x, int y);
	static void SetCursorPosition(int x, int y);
	static void Write            (int x, int y, const string& str);
	static void Fill             (int x, int y, int w, int h, char c);
	static void Refresh          (int y, int nrow);
	static void Input            (string& str);
};

}
