#pragma once

#include <sbscene.h>
#include <sbconverter.h>
#include <sbtokenizer.h>

namespace Scenebuilder{;

/** シーン状態の保存と復帰

	- ファイル形式
		<full path of object0>
		<full path of object1>
		...
		<full path of objectN>
		[empty line]
		time
		state of object0
		state of object1
		...
		state of objectN
		[empty line]
		time

		...
 **/

class LogBase{
public:
	/// 保存するオブジェクトの属性
	struct ObjectInfo{
		Address		addr;		///< シーン中のアドレス
		int			id;			///< オブジェクトID
		TypeInfo*	type;		///< オブジェクトタイプ
		uint		nbytes;		///< バイト数

		ObjectInfo(){
			id     = -1;
			type   =  0;
			nbytes =  0;
		}
	};

	SceneBase*			scene;
	TypeDB*				typedb;
	int					rootId;
	bool				binary;

	vector<ObjectInfo>	infos;
	
public:
	/// clear all
	virtual void Clear();

	/** register scene, typedb and root id
		this must be done first.
	 **/
	void Set(SceneBase* s, TypeDB* db, int id);

	/** select binary mode */
	void SetBinary(bool on);

	/** register object tree for logging */
	void AddRecurs(int top);

	/** register single object for logging */
	void Add(int id);

	LogBase();
};

class LogReader : public LogBase{
protected:
	struct Section{
		enum{
			Info,
			Timestamp,
			State,
		};
	};
	/// 1時刻分のスナップショット
	struct Snapshot : UTRefCount{
		real_t		time;							///< タイムスタンプ
		vector<string_iterator_pair>	ranges;		///< オブジェクトのプロパティをシリアル化した文字列の範囲
		vector<const byte*>				props;		///< オブジェクトのプロパティへのポインタ（バイナリの場合）
	};
	typedef vector< UTRef<Snapshot> >	Snapshots;

	Snapshots		snapshots;

	string			contents;		///< 読み込み時のファイルの中身
	Tokenizer		tok;
	const byte*		ptr;
	const byte*		ptr_end;
	uint			nbytes;			///< 総バイト数
	uint			nbytesHeader;	///< ヘッダのバイト数

protected:
	void ParseHeader();
	void ParseBinary();
	void ParseText  ();
		
public:
	/// load data from existing file
	void Load(const char* filename);

	/// number of snapshots
	int  Count();

	/// nearest snapshot index
	int  Nearest(real_t time);

	/// get time stamp
	real_t Time(int idx);

	/// read state at 'time' and copy to scene
	void Read(int idx);

};

class LogWriter : public LogBase{
protected:
	ofstream		logFile;		///< ログ取り用file stream
	vector<byte>	buf;			///< ストリームへ書き出すまえの一次バッファ
	
protected:
	void WriteHeader();
	void WriteBinary(real_t time);
	void WriteText  (real_t time);

public:
	/// open file for logging
	void Open(const char* filename);
	///
	void Close();
	/// add current state of scene with timestamp 'time'
	void Write(real_t time);

	LogWriter();
};

}
