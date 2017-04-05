#pragma once

#include <sbtypes.h>

namespace Scenebuilder{;

/**
	共有メモリの基本機能
 **/

class SharedMemory{
public:
	void*			hFile;
	byte*           pView;
	const char*		name;
	
public:
	/** @brief 共有メモリを作成する
		@param name		共有メモリの名前
		@param sz		共有メモリのバイト数
	 */
	void	Create(const char* name, size_t sz);

	/** @brief 既存の共有メモリを開く
		@param name		共有メモリの名前
	 */
	void	Open(const char* name);

	/**
	 **/
	bool	IsOpen();

	/** @brief 共有メモリを閉じる
	 */
	void	Close();

	/**	@brief 共有メモリのアドレスを取得する
		@brief offset	オフセット
		@brief sz		サイズ
	 */
	byte*	Get(size_t offset, size_t sz);

	/** 共有メモリを開いてポインタを返す */
	template<class T>
	T* TryOpen(const char* smname, size_t num, bool zerofill = true, bool constructor = true){
		T*	obj;
		bool	created = false;
		// まず既存の共有メモリを開いてみる
		try{
			Open(smname);
		}
		catch(FileError&){
			// 失敗したら作成
			try{
				Create(smname, sizeof(T)*num);
				created = true;
			}
			catch(FileError&){
				return 0;	// 新規作成にも失敗したらエラー
			}
		}

		// アドレスを取得
		byte* addr = Get(0, sizeof(T)*num);

		// 最初なら初期化
		if(created){
			if(zerofill){
				// 零フィルして
				fill(addr, addr + sizeof(T)*num, 0);
			}
			
			if(constructor){
				// コンストラクタ呼び出し
				for(uint i = 0; i < num; i++)
					new (addr + sizeof(T)*i) T;
			}
		}
		
		obj = (T*)addr;

		return obj;
	}

	SharedMemory();
	~SharedMemory();
};

}
