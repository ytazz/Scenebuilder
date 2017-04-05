#pragma once

/** 共通ビットマップ画像形式

*/

#include <sbtypes.h>

namespace Scenebuilder{;

class BitmapImage : public UTRefCount{
public:
	uint			width;
	uint			height;
	uint            bpp;
	vector<byte>	pixels;		///< pixel data, rgb
	
public:
	/// resize
	void Resize(uint w, uint h);

	/// get pixel
	void Get(int x, int y, byte& r, byte& g, byte& b);
	void Set(int x, int y, byte  r, byte  g, byte  b);
	void Get(int x, int y,       Vec3f& c);
	void Set(int x, int y, const Vec3f& c);

	/// load from file
	virtual bool Load(string filename) = 0;

	/// save to file
	virtual bool Save(string filename){ return false; }

	BitmapImage();

};

}
