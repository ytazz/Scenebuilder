#pragma once

#include <sbscene.h>
#include <sbmodel.h>
#include <sbxml.h>
#include <sbpath.h>
#include <sbcalc.h>

#include <stack>
#include <map>
#include <sstream>
using namespace std;

namespace Scenebuilder{;

/** Importプロパティ

	同一XMLファイル中のmoduleタグ，あるいは別のXMLファイルに記述されたXMLツリーのインポート箇所を定義する．
 */
struct ImportProp : SpatialObjectProp{
	static int id;
	str256_t	path;		///< moduleタグへのXML上の相対パス
	str256_t	filename;	///< XMLファイルへのディレクトリパス

	static string GetName(){ return "import"; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(ImportProp), &Construct, SpatialObjectProp::id);
		db->GetType(id)
			->AddAttr("path"    , Primitive::String, 256, OFFSET(ImportProp, path    ))
			->AddAttr("filename", Primitive::String, 256, OFFSET(ImportProp, filename));
	}
};

/** Moduleプロパティ

	他の箇所でインポートされるべきモジュールを定義する．
 */
struct ModuleProp : Property{
	static int id;
	static string GetName(){ return "module"; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(ModuleProp));
	}
};

/** Linkプロパティ

	他のタグへの参照を定義する．
 */
struct LinkProp : Property{
	static int id;
	str256_t	name;
	str256_t	path;

	static string GetName(){ return "link"; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(LinkProp));
		db->GetType(id)
			->AddAttr("name", Primitive::String, 256, OFFSET(LinkProp, name))
			->AddAttr("path", Primitive::String, 256, OFFSET(LinkProp, path));
	}
};

/** Forプロパティ
	- 子タグをループ展開する
 */
struct ForProp : Property{
	static int id;
	str256_t	var;
	int			begin;
	int			end;

	static string GetName(){ return "for"; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(ForProp));
		db->GetType(id)
			->AddAttr("var"  , Primitive::String, 256, OFFSET(ForProp, var  ))
			->AddAttr("begin", Primitive::Int   , 1  , OFFSET(ForProp, begin), 0)
			->AddAttr("end"  , Primitive::Int   , 1  , OFFSET(ForProp, end  ), 0);
	}
};

/** Varプロパティ
	- 変数を定義する
 */
struct VarProp : Property{
	static int id;
	str256_t	name ;
	real_t      value;
	
	static string GetName(){ return "var"; }
	static void Register(TypeDB* db){
		id = db->AddType(GetName(), sizeof(VarProp));
		db->GetType(id)
		  ->AddAttr("name"  , Primitive::String, 256, OFFSET(VarProp, name ))
	      ->AddAttr("value" , Primitive::Real  , 1  , OFFSET(VarProp, value));
	}
};

/**	Builder

    XMLに記述されたシーン情報をもとにシーンオブジェクトを生成する
 **/
class Builder{
protected:
	struct Attrs : map<string, string>{
		Attrs();
	};
	struct Vars : map<string, string>{
	};

	struct Context{
		Attrs attrs;
		Vars  vars;
	};
	struct ContextStack : public vector<Context>{

	};

	string		baseFilename;
	//AttrStack	attrStack;		///< stack of attributes
	Attrs		tempAttr;		///< temporary attributes
	//Vars		vars;			///< loop variables
	ContextStack ctxStack;
	
	/// tree of XML nodes
	stack< UTRef<XML> >		xmlStack;

	/// reference to type db
	TypeDB*		typedb;
	
	/// reference to scene
	SceneBase*	scene;
	int			curObjId;		///< current object id
	int			parsedRootId;	///< id of top-most object created in Parse

	/// model container
	ModelContainer*	models;
	
protected:
	/** @brief 属性値を取得

		はじめにtemporary attibuteをさがし，なければglobal attributeの階層の低い方から上に向かって探す．
	 **/
	string		GetAttrValue    (string name);
	void		FillProperty    (TypeInfo* type, Property* prop);
	void		AddLinks        (int srcId, int refId, TypeInfo* type);
	void		ProcessAttribute(int typeId, AttrInfo* attr, string value);
	void		ProcessNode     (int nodeId);
	void        RegisterVar     (string name);
	void        UnregisterVar   (string name);
	void        SetVarValue     (string name, int val);
public:
	static void Register(TypeDB* db);

	/** @brief シーン，型データベース，モデルコンテナを設定
		@param s	シーン
		@param db	型データベース
		@param m    モデルコンテナ
	 **/
	void Set(SceneBase* s, TypeDB* db, ModelContainer* m);

	/** @brief XMLを読み込んでシーンオブジェクトを生成する
		@param  filename	XMLファイルへのディレクトリパス
		@param  base		ベースオブジェクトID
		@return             生成された最上位オブジェクトのID

		XMLファイルからシーン構造を読み込み，ベースオブジェクトの子としてシーンオブジェクトを生成する．
		Parseを呼ぶ前にSetを呼んでおく必要がある．
	 */
	int Parse(string filename, int base = -1);

	/** @brief  オブジェクトを生成する
		@param	type  オブジェクトの型
		@param	name  オブジェクトの名前

		フォーカスされているオブジェクトの子として新しいオブジェクトを作成し，
		新しいオブジェクトにフォーカスを移す．
	 **/
	void Create(int type, string name = "");

	/** @brief  フォーカスを親オブジェクトに移す
	 **/
	void End();

	/** @brief  リンクを生成する
		@param  srcId リンク元オブジェクトID
		@param  refId パスの基点となるオブジェクトID
		@param  path  パス
		@param  name  リンク名

		フォーカスされているオブジェクトから別オブジェクトへのリンクを作成する
	 **/
	void Link(int srcId, int refId, const string& path, const string& name);
	
	Builder();
	virtual ~Builder();
};

}
