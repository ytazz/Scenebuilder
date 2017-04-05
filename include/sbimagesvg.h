#pragma once

#include <sbxml.h>

#include <stack>
#include <fstream>
using namespace std;

/** 機能限定SVGクラス
	- Inkscapeが出力するSVGに関するメモ：
	 - 作業座標原点は用紙左下だが、SVGファイル中では左上が原点となり，同時にy軸の方向が反転する
	 - 円は<path>タグで書かれるが、自動的にtransformがかかるため座標が取り出しにくい
		丸みつき矩形をパスに変換すると円形のパスが得られる
	 - 円や矩形は、線の太さを考慮して指定サイズの内側に描画される。このためデータ中では線の太さを差し引いて内側の座標値が書かれるため分かりづらい。
	 -  以上より、データから座標値が用意に読み取れるのは曲線のみ
 */

namespace Scenebuilder{;

class SVG{
public:
	/// レイヤー/グループ gタグから取得
	struct Layer : UTRefCount{
		string	name;				///< レイヤー名: id属性から取得
		string	inkscapeName;		///< レイヤー名: inkscapeの拡張タグから取得
		bool	shown;				///< 表示状態
	};

	/* Cubic Bezier curve */
	struct Segment{
		Vec2f	p0, p1;		///< end points
		Vec2f	c0, c1;		///< control points
		
		/* get intermediate value. t \in [0, 1] */
		Vec2f	GetPoint(float t)const;
		// 微分により接線方向を計算
		Vec2f	GetTangent(float t)const;
		// 差分により接線方向を計算
		Vec2f	GetTangentDiff(float t)const;

		void GetPoints(vector<Vec2f>& points, bool last);

		Segment(Vec2f _p0, Vec2f _c0, Vec2f _p1 = Vec2f(), Vec2f _c1 = Vec2f()):p0(_p0), c0(_c0), p1(_p1), c1(_c1){}
	};

	/// パス
	struct Path : UTRefCount{
		Layer*			layer;		//< 所属レイヤー
		string			name;		//< パスの名前
		vector<Segment>	segments;	//< パスセグメント
		Vec2f           bbmin;
		Vec2f           bbmax;

		/* convert polycurve to point lists */
		void GetPoints(vector<Vec2f>& points);

		/* bboxを計算 曲線は考慮せず端点のみ */
		void CalcBBox();
	};

	/// ロード用変数
	typedef vector< UTRef<Layer> >	Layers;
	typedef vector< UTRef<Path > >	Paths;
	Layers			layers;
	Paths			paths;
	vector<Layer*>	layerStack;
	
	/// セーブ用変数
	float			strokeWidth;
	string			strokeColor;
	string			fillColor;
	Vec2f           scale;
	Vec2f			offset;
	int				width;
	int				height;
	ofstream		file;

	std::stack<Affinef>  affModel;
	Affinef              affView;
	Affinef              affViewInv;
	Affinef              affProj;
	Affinef              affTrans;
	float                aspect;

protected:
	void Assert(bool eval);

	/// 描画属性を書き出す
	void WriteStyle();

	/// 3Dから2Dへの変換
	Vec2f Transform(Vec3f p);

public:
	SVG();

	/// ロード用関数
	void LoadPath(Path* path, XMLNode* node);
	void Load(XMLNode* node);
	void Load(XML& xml);

	/// セーブ用関数
	void SaveStart      (string filename, int width, int height);
	void SaveEnd        ();
	void SetScale       (float sx, float sy);
	void SetOffset      (float ox, float oy);
	void SetStrokeColor (string c);
	void SetStrokeWidth (float  w);
	void SetFillColor   (string c);
	void PushModelMatrix();
	void PopModelMatrix ();
	void MultModelMatrix(const Affinef& a);
	void SetModelMatrix (const Affinef& a);
	void SetViewMatrix  (const Affinef& a);
	void SetProjMatrix  (const Affinef& a);
	void SetAspect      (float a);
	void BeginLayer     (string name, bool shown);
	void EndLayer       ();
	void BeginPath      ();
	void EndPath        (bool loop);
	void MoveTo         (Vec2f p);
	void MoveTo         (Vec3f p);
	void LineTo         (Vec2f p);
	void LineTo         (Vec3f p);
	void DrawLine       (Vec2f p0, Vec2f p1);
	void DrawLine       (Vec3f p0, Vec3f p1);
	void DrawCircle     (Vec2f c,  float r);
	void DrawCircle     (Vec3f c,  float r, int ndiv);
	void DrawRect       (Vec2f p0,   Vec2f p1);
	void DrawBox        (Vec3f rmin, Vec3f rmax);
	void DrawSphere     (Vec3f p, float r);

};

}
