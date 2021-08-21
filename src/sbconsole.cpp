#include <sbconsole.h>

#ifdef USE_CURSES
extern "C"{
# include <curses.h>
}
#endif

#ifdef _WIN32
# include <windows.h>
# undef min
#endif

namespace Scenebuilder{;

int    Console::numRows;
int    Console::numColumns;
string Console::buffer;

#ifdef USE_CURSES
static WINDOW* winOut = 0;
static WINDOW* winIn  = 0;
#endif

void Console::Init(){
#ifdef USE_CURSES
	initscr();
#endif
}

void Console::Cleanup(){
#ifdef USE_CURSES
	endwin();
	//noraw();
#endif
}

void Console::SetFontSize(int sz){
#ifdef _WIN32
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_FONT_INFOEX info;
	info.cbSize = sizeof(CONSOLE_FONT_INFOEX);
	GetCurrentConsoleFontEx(hStdout, FALSE, &info);
	info.dwFontSize.X = sz/2;
	info.dwFontSize.Y = sz;
	SetCurrentConsoleFontEx(hStdout, FALSE, &info);
#endif
}

void Console::SetBufferSize(int nrow, int ncol){
#ifdef _WIN32
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	
	CONSOLE_SCREEN_BUFFER_INFOEX coninfo;
	coninfo.cbSize = sizeof(CONSOLE_SCREEN_BUFFER_INFOEX);
	GetConsoleScreenBufferInfoEx(hStdout, &coninfo);
	
	coninfo.dwSize.X = ncol;
	coninfo.dwSize.Y = nrow;
	coninfo.srWindow.Left   = 0;
	coninfo.srWindow.Top    = 0;
	coninfo.srWindow.Right  = ncol;
	coninfo.srWindow.Bottom = nrow;
	
	SetConsoleScreenBufferInfoEx(hStdout, &coninfo);
#endif

	numRows    = nrow;
	numColumns = ncol;
	buffer.resize(ncol * nrow);
}

void Console::SetDisplaySize(int nrow, int ncol){
#ifdef _WIN32
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	
	SMALL_RECT sr;
	sr.Left   = 0;
	sr.Top    = 0;
	sr.Right  = ncol;
	sr.Bottom = nrow;
	SetConsoleWindowInfo(hStdout, FALSE, &sr);
#endif

#ifdef USE_CURSES
	resize_term(nrow, ncol);
	nrow = std::min(nrow, LINES);
	ncol = std::min(ncol, COLS );
	winOut = newwin(nrow-1, ncol, 0     , 0);
	winIn  = newwin(1     , ncol, nrow-1, 0);
	WINDOW* wtmp = newwin(10, 10, 0, 0);
#endif
}

void Console::SetWindowPosition(int x, int y){
#ifdef USE_CURSES
#else
	HWND hconsole = GetConsoleWindow();
	RECT rc;
	GetWindowRect(hconsole, &rc);
	MoveWindow(hconsole, (x == -1 ? rc.left : x), (y == -1 ? rc.top  : y), rc.right - rc.left, rc.bottom - rc.top, FALSE);
#endif
}

void Console::SetCursorPosition(int x, int y){
#ifdef USE_CURSES
	wmove(winIn, 0, x);
#else
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	
	COORD cd;
	cd.X = x;
	cd.Y = y;
	SetConsoleCursorPosition(hStdout, cd);
#endif
}

void Console::Write(int x, int y, const string& str){
	int len = (int)str.size();
	int sz  = (int)buffer.size();
	int i0  = (numColumns*y+x);
	int i1  = i0 + len;
	if( 0 <= i0 && i0 < sz &&
		0 <= i1 && i1 < sz )
		copy(str.begin(), str.end(), buffer.begin() + i0);
}

void Console::Fill(int x, int y, int w, int h, char c){
	for(int i = 0; i < h; i++)for(int j = 0; j < w; j++)
		buffer[numColumns*(y+i) + (x+j)] = c;
}

void Console::Refresh(int y, int nrow){
#ifdef USE_CURSES
	WINDOW* win;
	int ywin;
	int ybuf;
	if(y == numRows-1){
		win = winIn;
		ywin = 0;
		ybuf = numRows-1;
	}
	else{
		win  = winOut;
		ywin = y;
		ybuf = y;
	}
	
	wmove(win, ywin, 0);
	for(int i = 0; i < nrow; i++)
		wdeleteln(win);
	for(int i = 0; i < nrow; i++)
		waddnstr(win, &buffer[numColumns*(ybuf+i)], numColumns);
	wrefresh(win);
#else
	HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
	COORD c;
	c.X = 0;
	c.Y = y;
	DWORD len;
	WriteConsoleOutputCharacter(hStdout, &buffer[numColumns*y], numColumns*nrow, c, &len);		
#endif
}

void Console::Input(string& str){
#if defined _WIN32 && !defined USE_CURSES
	// input with timeout
	fflush(stdin);
	HANDLE hstdin  = GetStdHandle(STD_INPUT_HANDLE);
	int ret = WaitForSingleObject(hstdin, 100);
	if(ret == WAIT_OBJECT_0){
		getline(cin, str);
	}
#endif

#ifdef USE_CURSES
	char line[1024];
	wgetstr(winIn, line);
	str = line;
#endif
}

}
