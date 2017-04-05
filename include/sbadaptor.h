#pragma once

#include <sbscene.h>

namespace Scenebuilder{;

class ModelContainer;

/** Adaptor
	
	シーンと外部ライブラリとのアダプタ．
	Scenebuilderのシーンオブジェクトの型とプロパティにもとづき，
	外部ライブラリの内部オブジェクトを作成・削除あるいは設定を行う．
	各アダプタが提供する機能や処理の内容はアダプタ毎の仕様となる．

 **/
class Adaptor{
protected:
	/// オブジェクトの対応状態
	struct SupportState{
		enum{
			Ignored	  = 0,		///< 無視する
			Supported = 1,		///< 対応する
			Undefined = 2,		///< 未判定
		};
	};

	/**
		アダプタ補助情報の基本クラス
		- Aux間の依存関係の保持
		- 更新の伝播
	 */
	struct Aux : UTRefCount{
		Adaptor*		adaptor;	///< アダプタへの参照
		int				id;			///< 対応するオブジェクトID

		virtual ~Aux(){}
	};
	typedef	vector< UTRef<Aux> >	Auxs;
	
	SceneBase*		scene;			///< target scene
	TypeDB*			typedb;			///< type db
	ModelContainer*	models;			///< model container
	Address			addr;			///< absolute address of target subscene
	int				rootId;			///< id of root object of target subscene

	vector<byte>	alive;			///< idからliveness（使用中オブジェクトかどうか）をひくための配列
	vector<byte>	state;			///< idからサポート状況をひくための配列
	vector<byte>	toCreate;		///< idからCreateObjectを呼ぶ必要があるかをひくための配列
	vector<byte>	toDelete;		///< idからDeleteObjectを呼ぶ必要があるかをひくための配列

	Auxs			handle;			///< idからライブラリ固有ハンドルをひくための配列

	/// オブジェクトのサポート状態が未確定か
	bool	IsUndefined(int id);
	
	/// オブジェクトにデータを関連付ける
	void	RegAux(int id, Aux* aux);
	
	/// データを削除
	void	RemoveAux(int id);
	
	/// 関連付けられているデータを取得
	Aux*	GetAux(int id);

public:
	Adaptor();
	virtual ~Adaptor();

	/// 適切なIDか
	bool	IsValidID(int id);

	/// オブジェクトがサポートされているか
	bool	IsSupported(int id);
	
	/// オブジェクトが無視されているか
	bool	IsIgnored(int id);
	
	/** @brief シーン，型データベース，モデルコンテナを設定
		@param s   シーン
		@param db  型データベース
		@param m   モデルコンテナ
		@param id  ルートオブジェクトID
	 **/
	void Set(SceneBase* s, TypeDB* db, ModelContainer* m, int id = -1);

	/** @brief ミラー処理を行う

		シーン中で現在使用されているオブジェクトIDと外部ライブラリの対応状況を比較し，
		必要に応じて外部ライブラリ固有オブジェクトの作成と削除を行う
	 **/
	void Mirror();

	/** @brief 同期処理を行う

		@param download
		trueならシーンからアダプタへの同期．
		falseならアダプタからシーンへの同期．
		@param cat
		同期するプロパティのカテゴリ
		@param type
		同期するオブジェクトの型．-1なら全ての型が対象となる．

		シーンオブジェクトとアダプタの内部オブジェクトとの間でプロパティを同期する
	 **/
	void SyncProperty(bool download, int cat = AttrCategory::All, int type = -1);

	/** @brief IDで指定したシーンオブジェクトに対応するライブラリ固有オブジェクトを取得する

		@param	id	    オブジェクトID
		@param	typeId	IDで指定したオブジェクトが属するべき型
	 **/
	Aux*	HandleByID  (int id     , int typeId);
	
	/** @brief パスで指定したシーンオブジェクトに対応するライブラリ固有オブジェクトを取得する

		@param	path	オブジェクトを指示するパス
		@param	typeId	パスで指定したオブジェクトが属するべき型
	 **/
	Aux*	HandleByName(string path, int typeId);

public:

	/// virtual functions to be implemented by derived classes
	/// 派生クラスが実装すべき仮想関数

	/** \if english
	    @brief	creates library-dependent data of an object
		@param	id	object id
		@return	element of SupportState
		\endif

		\if japanese
		@brief  シーンオブジェクトに対応するライブラリ固有オブジェクトを作成する
		@param  id  オブジェクトID
		@return サポート状態
		\endif
	 **/
	virtual int		CreateObject(int id) = 0;

	/** @brief  シーンオブジェクトに対応するライブラリ固有オブジェクトを削除する
		@param  id  オブジェクトID

		- Sceneの仕様でScene::DeleteObjectはサブツリーに属するオブジェクトと関連リンクをすべて削除することになっている
		- サブツリー削除時のAdaptor::DeleteObjectが呼ばれる順番は不定
		- ただしリンク元のオブジェクトは削除されないので，リンク切れに対して適切に処理するのはアダプタの責任
	 **/
	virtual void	DeleteObject(int id) = 0;

	/** @brief  シーンオブジェクトのプロパティとライブラリ固有オブジェクトの状態を同期する
	    @param  id        オブジェクトID
		@param  download
		trueならばシーンオブジェクトのプロパティをライブラリ固有オブジェクトの状態に反映する．
		falseならばライブラリ固有オブジェクトの状態をシーンオブジェクトのプロパティに反映する．
		@param  cat       同期対象のプロパティのカテゴリ
	 **/
	virtual void	SyncObjectProperty(int id, bool download, int cat) = 0;

};

}
