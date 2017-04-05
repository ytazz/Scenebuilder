#include <sbimage.h>

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

BitmapImage::BitmapImage(){
	width  = 0;
	height = 0;
	bpp    = 3;
}

void BitmapImage::Resize(uint w, uint h){
	pixels.resize(w*h*bpp);
	width  = w;
	height = h;
}

void BitmapImage::Get(int x, int y, byte& r, byte& g, byte& b){
	x = std::min(std::max(0, x), (int)width -1);
	y = std::min(std::max(0, y), (int)height-1);
	byte* p = &pixels[bpp*(width*y+x)];
	r = p[0];
	g = p[1];
	b = p[2];
}

void BitmapImage::Set(int x, int y, byte  r, byte  g, byte  b){
	x = std::min(std::max(0, x), (int)width -1);
	y = std::min(std::max(0, y), (int)height-1);
	byte* p = &pixels[bpp*(width*y+x)];
	p[0] = r;
	p[1] = g;
	p[2] = b;
}

void BitmapImage::Get(int x, int y, Vec3f& c){
	x = std::min(std::max(0, x), (int)width -1);
	y = std::min(std::max(0, y), (int)height-1);
	byte* p = &pixels[bpp*(width*y+x)];
	c[0] = (float)p[0] / 255.0f;
	c[1] = (float)p[1] / 255.0f;
	c[2] = (float)p[2] / 255.0f;
}

void BitmapImage::Set(int x, int y, const Vec3f& c){
	x = std::min(std::max(0, x), (int)width -1);
	y = std::min(std::max(0, y), (int)height-1);
	byte* p = &pixels[bpp*(width*y+x)];
	p[0] = (int)(255.0f * c[0]);
	p[1] = (int)(255.0f * c[1]);
	p[2] = (int)(255.0f * c[2]);
}

}
