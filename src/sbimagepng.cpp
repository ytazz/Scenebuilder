#include <sbimagepng.h>
#include <sbmessage.h>

#ifdef USE_LIBPNG
# include <png.h>
#endif

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

ImagePNG::ImagePNG(){

}

bool ImagePNG::Load(string filename){
#ifdef USE_LIBPNG
	Message::Extra("pnglib version %lu", (unsigned long)png_access_version_number());

	FILE* file = fopen(filename.c_str(), "rb");
	if(!file){
		Message::Error("failed to open %s", filename.c_str());
		return false;
	}

	png_structp png  = png_create_read_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
	png_infop   info = png_create_info_struct(png);

	// エラーでここに飛んでくる
	if(setjmp(png_jmpbuf(png))){
		Message::Error("error occured while reading png file");
		png_destroy_read_struct(&png, &info, 0);
		fclose(file);
		return false;
	}

	// I/O初期化
	png_init_io(png, file);

	// 全部読む
	png_read_png(png, info, PNG_TRANSFORM_IDENTITY, 0);

	// 情報取得
	width    = png_get_image_width (png, info);
	height   = png_get_image_height(png, info);
	bpp      = png_get_channels(png, info);
	
	// 画像データ取得
	byte** rows = png_get_rows(png, info);
	pixels.resize(bpp * width * height);
	for(uint i = 0; i < height; i++){
		byte* row = rows[i];
		copy(row, row + (bpp * width), &pixels[i * (bpp * width)]);
	}

	png_destroy_read_struct(&png, &info, 0);
	fclose(file);

	return true;
#else
    return false;
#endif
}

}
