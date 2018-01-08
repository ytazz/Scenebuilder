#include <sbimagebmp.h>
#include <sbmessage.h>

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

ImageBMP::ImageBMP(){

}

bool ImageBMP::Load(string filename){
	ifstream ifs;
	ifs.open(filename.c_str(), ios_base::in | ios_base::binary);
	if(!ifs.is_open())
		throw FileError();

	ifs.seekg(0, ifs.end);
	int len = (int)ifs.tellg();
	ifs.seekg(0, ifs.beg);

	string contents;
	contents.resize(len);
	ifs.read(&contents[0], len);
	
	ifs.close();

	char* ptr = &contents[0];
	fileHeader = *(FileHeader*)ptr;
	infoHeader = *(InfoHeader*)(ptr + sizeof(FileHeader));
	char* data = ptr + fileHeader.offset;

	if(infoHeader.nbits != 24){
		Message::Error("only 24 bit bmp is supported");
		return false;
	}

	width   = infoHeader.width;
	height  = infoHeader.height;
	bpp     = 3;
	int nbytesLine = bpp * width;
	while(nbytesLine % 4)
		nbytesLine++;

	// 上下反転，BGR -> RGB
	pixels.resize(bpp * width * height);
	for(uint i = 0; i < height; i++){
		for(uint j = 0; j < width; j++){
			unsigned char* p = (unsigned char*)&data[(height - 1 - i) * nbytesLine + bpp * j];
			byte* pix = &pixels[3 * (i * width + j)];
			pix[0] = p[2];
			pix[1] = p[1];
			pix[2] = p[0];
		}
	}

	contents.clear();

	return true;
}

bool ImageBMP::Save(string filename){
	ofstream ofs;
	ofs.open(filename.c_str(), ios_base::out | ios_base::binary);
	if(!ofs.is_open())
		throw FileError();

	int nbytesLine = bpp * width;
	while(nbytesLine % 4)
		nbytesLine++;

	int szHeader = sizeof(FileHeader) + sizeof(InfoHeader);
	int szData   = nbytesLine * height;

	fileHeader.signature[0] = 'B';
	fileHeader.signature[1] = 'M';
	fileHeader.sizeFile     = szHeader + szData;
	fileHeader.res1         = 0;
	fileHeader.res2         = 0;
	fileHeader.offset       = szHeader;

	infoHeader.sizeHeader       = 40;
	infoHeader.width            = width;
	infoHeader.height           = height;
	infoHeader.nplanes          = 1;
	infoHeader.nbits            = 24;
	infoHeader.compress         = 0;
	infoHeader.sizeBitmap       = szData;
	infoHeader.hres             = 0;
	infoHeader.vres             = 0;
	infoHeader.ncolors          = 0;
	infoHeader.nimportantColors = 0;


	ofs.write((const char*)&fileHeader, sizeof(FileHeader));
	ofs.write((const char*)&infoHeader, sizeof(InfoHeader));

	vector<byte> data;
	data.resize(szData);
	fill(data.begin(), data.end(), 0);

	// 上下反転，BGR -> RGB
	for(uint i = 0; i < height; i++){
		for(uint j = 0; j < width; j++){
			byte* p   = (byte*)&data[(height - 1 - i) * nbytesLine + bpp * j];
			byte* pix = &pixels[3 * (i * width + j)];
			p[0] = pix[2];
			p[1] = pix[1];
			p[2] = pix[0];
		}
	}

	vector<byte> dummy;
	dummy.resize(fileHeader.offset - (sizeof(FileHeader) + sizeof(InfoHeader)));
	fill(dummy.begin(), dummy.end(), 0);

	ofs.write((const char*)&dummy[0], dummy.size());
	ofs.write((const char*)&data [0], data .size());

	return true;
}

}
