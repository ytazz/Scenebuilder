#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <sbsci.h>
#include <sbmessage.h>

#include <Foundation/UTPreciseTimer.h>

namespace Scenebuilder{;

Sci::Sci(){
	handle          = 0;
	isOnline        = false;
	txBufferEnabled = false;
	rxTotal         = 0;
	txTotal         = 0;
}

Sci::~Sci(){
	Terminate();
}

void Sci::Init(const string& port, int baud, int byteSize, int stopBits){
	//既存の接続（もしあれば）を解除
	Terminate();

	// 通信ポートを開く
	handle = CreateFile(
		port.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);
	if(handle == INVALID_HANDLE_VALUE)
		throw Exception();
	
	// 通信設定
	DCB dcb;
	ZeroMemory(&dcb, sizeof(DCB));
	if(!GetCommState(handle, &dcb))
		throw Exception();

	// ボーレート
	dcb.BaudRate	= baud;

	// パリティなし
	dcb.fParity		= true;
	dcb.Parity		= NOPARITY;

	// ストップビット1
	dcb.StopBits	= ONESTOPBIT;
	if(stopBits == 2)
		dcb.StopBits = TWOSTOPBITS;

	// バイトサイズ
	dcb.ByteSize	= byteSize;

	// バイナリモード
	dcb.fBinary		= true;
	
	// フロー制御なし
	dcb.fOutxCtsFlow = false;
	dcb.fOutxDsrFlow = false;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fInX		= false;
	dcb.fOutX		= false;
	
	if(!SetCommState(handle, &dcb))
		throw Exception();
	
	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout         = 10;
	timeouts.ReadTotalTimeoutMultiplier  = 10;
	timeouts.ReadTotalTimeoutConstant    = 1000;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant   = 1000;
	SetCommTimeouts(handle, &timeouts );
	
	PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
	
	isOnline  = true;
	rxTotal = 0;
	txTotal = 0;

}

void Sci::Terminate(){
	if(isOnline){
		CloseHandle(handle);
		handle = 0;

		isOnline = false;
	}
}

void Sci::EnableTxBuffer(bool on){
	txBufferEnabled = on;
	txBuffer.clear();
}

size_t Sci::FlushTxBuffer(){
	if(!isOnline)
		throw Exception();

	if(!txBufferEnabled)
		return 0;

	DWORD nBytesWritten;
	WriteFile(handle, (LPVOID)&txBuffer[0], txBuffer.size(), &nBytesWritten, 0);

	FlushFileBuffers(handle);
	txBuffer.clear();

	txTotal += nBytesWritten;

	return nBytesWritten;
}

size_t Sci::Out(const byte* c, size_t n){
	if(!isOnline)
		throw Exception();

	if(txBufferEnabled){
		txBuffer.insert(txBuffer.end(), c, c+n);
		return n;
	}
	else{
		DWORD nBytesWritten;
		WriteFile(handle, (LPVOID)c, (DWORD)n, &nBytesWritten, 0);
		FlushFileBuffers(handle);
	
		txTotal += nBytesWritten;

		return nBytesWritten;
	}

}

size_t Sci::In(byte* c, size_t n, bool full){
	if(!isOnline)
		throw Exception();

	DWORD nreadTotal = 0;
	DWORD nread;
	while(nreadTotal < n){
		ReadFile(handle, (LPVOID)c, (DWORD)(n - nreadTotal), &nread, 0 );
		nreadTotal += nread;

		if(!full || nread == 0 || nreadTotal == n)
			break;

		c += nread;
	}
	
	rxTotal += nreadTotal;

	return nreadTotal;
}

void Sci::CountBytes(size_t* rx, size_t* tx){
	DWORD _rx, _tx;
	COMSTAT stat;
	ClearCommError(handle, 0, &stat);
	_rx = stat.cbInQue;
	_tx = stat.cbOutQue;

	if(rx)
		*rx = _rx;
	if(tx)
		*tx = _tx;
}

}
