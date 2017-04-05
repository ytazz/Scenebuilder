#pragma once

#include <sbtypes.h>
#include <sbevent.h>

namespace Scenebuilder{;

class Sci{
public:
	void*	handle;       ///< �n���h��
	void*   overlapped;
	bool	isOnline;     ///< �ڑ���
	
	bool         txBufferEnabled;
	vector<byte> txBuffer;

	uint    rxTotal;    ///< ����M�o�C�g��
	uint    txTotal;    ///< �����M�o�C�g��
	
public:
	
	/**	@brief ������
		@param	name	�I�[�v������f�o�C�X�t�@�C���D�f�t�H���g��COM1
		@param	baud	�{�[���[�g [bps]
	 */
	void Init(const string& port, int baud = 57600, int byteSize = 8, int stopBits = 1);

	/** @brief �I��
		�ڑ���ؒf����D�f�X�g���N�^������Ă΂��D
	 */
	void Terminate();

	/** @�ڑ������ǂ���
	 **/
	bool IsOnline(){ return isOnline; }

	/** �o�̓o�b�t�@��L��/������
	     �o�b�t�@�L������Out�Ăяo���ŏo�̓o�b�t�@�Ɋi�[���CFlushTxBuffer�Ăяo���Ŏ��ۂ̑��M���s��
     */
	void   EnableTxBuffer(bool on = true);

	/// �o�̓o�b�t�@���t���b�V��
	size_t FlushTxBuffer ();

	/** @brief ���M
		@param c	���M�f�[�^
		@param n	���M�v���o�C�g��
		@return		���ۂ̑��M�o�C�g��
	 */
	size_t Out(const byte* c, size_t n);

	/** @brief ���M(template��)
		@param c	���M�f�[�^
		T�^�̃f�[�^��1���M����D(double x = 1.0; Out(x);)
	 */
	template<class T>
	size_t Out(const T& c){
		return Out((const byte*)&c, sizeof(T));
	}

	/** @brief ���M(�������)
	 */
	size_t TextOut(const char* c){
		return Out((const byte*)c, strlen(c));
	}
	
	/** @brief ��M
		@param c	��M�o�b�t�@
		@param n	��M�v���o�C�g��
		@param full �v���o�C�g����M����܂Ŗ߂�Ȃ�
		@return		���ۂ̎�M�o�C�g��
	 */
	size_t In(byte* c, size_t n, bool full = true);

	/**	@brief ��M(template��)
		@param c	��M�ϐ�
		T�^�̃f�[�^��1��M����D(double x; In(&x);)
	 */
	template<class T>
	size_t In(T* c, bool full = true){
		return In((byte*)c, sizeof(T), true);
	}

	/** @brief ��M/���M�o�b�t�@�ɂ��܂��Ă���f�[�^�̃o�C�g�����擾����
	 */
	void CountBytes(size_t* rx, size_t* tx);
	
	 Sci();
	~Sci();
};

}
