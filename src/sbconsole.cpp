#include <sbconsole.h>

#include <windows.h>

namespace Scenebuilder{;

int    Console::numRows;
int    Console::numColumns;
string Console::buffer;

void Console::SetFontSize(int sz){
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_FONT_INFOEX info;
	info.cbSize = sizeof(CONSOLE_FONT_INFOEX);
	GetCurrentConsoleFontEx(hStdout, FALSE, &info);
	info.dwFontSize.X = sz/2;
	info.dwFontSize.Y = sz;
	SetCurrentConsoleFontEx(hStdout, FALSE, &info);
}

void Console::SetBufferSize(int nrow, int ncol){
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	
	CONSOLE_SCREEN_BUFFER_INFOEX coninfo;
	coninfo.cbSize = sizeof(CONSOLE_SCREEN_BUFFER_INFOEX);
	GetConsoleScreenBufferInfoEx(hStdout, &coninfo);
	
	coninfo.dwSize.X = ncol;
	coninfo.dwSize.Y = nrow;
	coninfo.srWindow.Right  = ncol;
	coninfo.srWindow.Bottom = nrow;
	
	SetConsoleScreenBufferInfoEx(hStdout, &coninfo);

	numRows    = nrow;
	numColumns = ncol;
	buffer.resize(ncol * nrow);
}

void Console::SetDisplaySize(int nrow, int ncol){
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	
	SMALL_RECT sr;
	sr.Left   = 0;
	sr.Top    = 0;
	sr.Right  = ncol;
	sr.Bottom = nrow;
	SetConsoleWindowInfo(hStdout, FALSE, &sr);
}

void Console::SetWindowPosition(int x, int y){
	HWND hconsole = GetConsoleWindow();
	RECT rc;
	GetWindowRect(hconsole, &rc);
	MoveWindow(hconsole, (x == -1 ? rc.left : x), (y == -1 ? rc.top  : y), rc.right - rc.left, rc.bottom - rc.top, FALSE);
}

void Console::SetCursorPosition(int x, int y){
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	
	COORD cd;
	cd.X = x;
	cd.Y = y;
	SetConsoleCursorPosition(hStdout, cd);
}

void Console::Write(int x, int y, const string& str){
	copy(str.begin(), str.end(), buffer.begin() + (numColumns*y+x));
}

void Console::Fill(int x, int y, int w, int h, char c){
	for(int i = 0; i < h; i++)for(int j = 0; j < w; j++)
		buffer[numColumns*(y+i) + (x+j)] = c;
}

void Console::Refresh(int y, int nrow){
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	COORD c;
	c.X = 0;
	c.Y = y;
	DWORD len;
	WriteConsoleOutputCharacter(hStdout, &buffer[numColumns*y], numColumns*nrow, c, &len);		
}

}
