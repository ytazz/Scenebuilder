#pragma once

#include <sbtypes.h>
#include <sbimage.h>

namespace Scenebuilder{;

/**
 *	BMP image loader
 *  24 bit only
 */

class ImageBMP : public BitmapImage{
public:
#pragma pack(push,1)
	struct FileHeader{
		unsigned short signature;
		unsigned int   sizeFile;
		unsigned short res1;
		unsigned short res2;
		unsigned int   offset;
	};
	struct InfoHeader{
		unsigned int   sizeHeader;
		unsigned int   width;
		unsigned int   height;
		unsigned short nplanes;
		unsigned short nbits;
		unsigned int   compress;
		unsigned int   sizeBitmap;
		unsigned int   hres;
		unsigned int   vres;
		unsigned int   ncolors;
		unsigned int   nimportantColors;
	};
#pragma pack(pop)

	FileHeader  fileHeader;
	InfoHeader  infoHeader;

public:
	/// load from file
	bool Load(string filename);

	/// save to file
	bool Save(string filename);

	ImageBMP();

};

}
