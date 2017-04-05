#ifndef SB_SYNC_H
#define SB_SYNC_H

#include <sbscene.h>

namespace Scenebuilder{;

/** Synchronizer
	- 動的に変化し得る項目:
	 - オブジェクトの作成と削除	-> ID対応
	 - プロパティの変化			-> タイムスタンプ
	 - ステートの変化				-> タイムスタンプ
	 - 親子関係の変化			-> 考慮しない
	 - リンクの作成と削除		-> 考慮しない

	- 同期の手順
	 - ID対応を解決
	  - 1. 片方にオブジェクトがありID対応が無い　-> 作成
	  - 2. ID対応があり片方にオブジェクトが無い　-> 削除
	  - 3. ID対応があり両方にオブジェクトがある
	   - プロパティとステートのタイムスタンプに不一致があれば
	     古い方を更新
	 - 1で作成されたオブジェクトの親子関係とリンクを設定する
	 
 **/
class Synchronizer{
protected:
	SceneBase*	scene[2];
	
	/** ID map
		- 
	 **/
	/// id -> {0,1}
	vector<byte>	alive[2];
	/// 
	vector<int>		idMap[2];
	/// array of live object ids
	vector<int>		liveId[2];

	Buffer			buf;
	
public:
	/** @brief	synchronize object tree
	 **/
	void SyncTree(Address& addr0, Address& addr1);

	/** @brief	synchronize properties
	 **/
	void SyncProperty();

	/** @brief	synchronize states
	 **/
	void SyncState();
	
	/** @brief register target scenes
	 **/
	void Set(SceneBase* s0, SceneBase* s1);

	Synchronizer();
	~Synchronizer();

};

}

#endif
