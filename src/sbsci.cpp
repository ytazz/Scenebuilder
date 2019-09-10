#ifdef _WIN32
# define WIN32_LEAN_AND_MEAN
# include <windows.h>
#endif

#include <sbsci.h>
#include <sbmessage.h>

namespace Scenebuilder{;

Sci::Sci(){
	handle          = 0;
	isOnline        = false;
	txBufferEnabled = false;
	rxTotal         = 0;
	txTotal         = 0;

	rxTimeoutPerByte    = 10;
	rxTimeoutMultiplier = 10;
	rxTimeout           = 1000;
	txTimeoutMultiplier = 10;
	txTimeout           = 1000;
}

Sci::~Sci(){
	Terminate();
}

void Sci::Init(const string& port, int baud, int byteSize, int stopBits, int parity){
	//�����̐ڑ��i��������΁j������
	Terminate();

#ifdef _WIN32
	// �ʐM�|�[�g���J��
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
	
	// �ʐM�ݒ�
	DCB dcb;
	ZeroMemory(&dcb, sizeof(DCB));
	if(!GetCommState(handle, &dcb))
		throw Exception();

	// �{�[���[�g
	dcb.BaudRate	= baud;

	// �p���e�B�Ȃ�
	dcb.fParity		= true;
	if(parity == Parity::None)
		dcb.Parity = NOPARITY;
	if(parity == Parity::Odd)
		dcb.Parity = ODDPARITY;
	if(parity == Parity::Even)
		dcb.Parity = EVENPARITY;

	// �X�g�b�v�r�b�g1
	dcb.StopBits	= ONESTOPBIT;
	if(stopBits == 2)
		dcb.StopBits = TWOSTOPBITS;

	// �o�C�g�T�C�Y
	dcb.ByteSize	= byteSize;

	// �o�C�i�����[�h
	dcb.fBinary		= true;
	
	// �t���[����Ȃ�
	dcb.fOutxCtsFlow = false;
	dcb.fOutxDsrFlow = false;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fInX		= false;
	dcb.fOutX		= false;
	
	if(!SetCommState(handle, &dcb))
		throw Exception();
	
	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout         = 	 rxTimeoutPerByte   ;
	timeouts.ReadTotalTimeoutMultiplier  = 	 rxTimeoutMultiplier;
	timeouts.ReadTotalTimeoutConstant    = 	 rxTimeout          ;
	timeouts.WriteTotalTimeoutMultiplier = 	 txTimeoutMultiplier;
	timeouts.WriteTotalTimeoutConstant   = 	 txTimeout          ;
	SetCommTimeouts(handle, &timeouts );
	
	PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
#endif

	isOnline  = true;
	rxTotal = 0;
	txTotal = 0;

}

void Sci::Terminate(){
	if(isOnline){
#ifdef _WIN32
		CloseHandle(handle);
#endif
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
	if(txBuffer.empty())
		return 0;

	uint32_t nBytesWritten;
#ifdef _WIN32
	WriteFile(handle, (LPVOID)&txBuffer[0], (DWORD)txBuffer.size(), (LPDWORD)&nBytesWritten, 0);

	FlushFileBuffers(handle);
#endif
	txBuffer.clear();

	txTotal += nBytesWritten;

	return nBytesWritten;
}

void Sci::ClearRxBuffer(){
#ifdef _WIN32
	PurgeComm(handle, PURGE_RXCLEAR);
#endif
}

size_t Sci::Out(const byte* c, size_t n){
	if(!isOnline)
		throw Exception();

	if(txBufferEnabled){
		txBuffer.insert(txBuffer.end(), c, c+n);
		return n;
	}
	else{
		uint32_t nBytesWritten;
#ifdef _WIN32
		WriteFile(handle, (LPVOID)c, (DWORD)n, (LPDWORD)&nBytesWritten, 0);
		FlushFileBuffers(handle);
#endif
		txTotal += nBytesWritten;

		return nBytesWritten;
	}

}

size_t Sci::In(byte* c, size_t n, bool full){
	if(!isOnline)
		throw Exception();

	uint32_t nreadTotal = 0;
	uint32_t nread;
	while(nreadTotal < n){
#ifdef _WIN32
		ReadFile(handle, (LPVOID)c, (DWORD)(n - nreadTotal), (LPDWORD)&nread, 0 );
#endif
		nreadTotal += nread;

		if(!full || nread == 0 || nreadTotal == n)
			break;

		c += nread;
	}
	
	rxTotal += nreadTotal;

	return nreadTotal;
}

void Sci::CountBytes(size_t* rx, size_t* tx){
	uint32_t _rx, _tx;
#ifdef _WIN32
	COMSTAT stat;
	ClearCommError(handle, 0, &stat);
	_rx = stat.cbInQue;
	_tx = stat.cbOutQue;
#endif
	if(rx)
		*rx = _rx;
	if(tx)
		*tx = _tx;
}

}
