#pragma once

#include <sbtypes.h>
#include <sbimage.h>

namespace Scenebuilder{;

/**
 *	PNG image loader
 */

class ImagePNG : public BitmapImage{
public:
	
public:
	/// load from file
	bool Load(string filename);

	ImagePNG();

};

}
