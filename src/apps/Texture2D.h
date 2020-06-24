#pragma once
#include "lodepng/lodepng.h"
#include "tinyexr.h"

class Texture2D
{
public:

	Texture2D() : image(nullptr), width(0), height(0), handle(0), dataSize(0), uploadedScanline(0), initializedData(0)
	{
	}

	void Initialize(int width, int height, int bpp, void *rgbaData)
	{
		if (handle == 0)
		{
			glGenTextures(1, &handle);
		}
		glBindTexture(GL_TEXTURE_2D, handle);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		GLint gl_internal_format = GL_RGBA8;
		gl_format = GL_RGBA;
		gl_type = GL_UNSIGNED_BYTE;

		if (bpp == 8)
		{
			gl_internal_format = GL_R8;
			gl_format = GL_RED;
			gl_type = GL_UNSIGNED_BYTE;
		}
		else if (bpp == 16)
		{
			gl_internal_format = GL_R16;
			gl_format = GL_RED;
			gl_type = GL_UNSIGNED_SHORT;
		}
		else if (bpp == 32)
		{
			gl_internal_format = GL_RGBA8;
			gl_format = GL_RGBA;
			gl_type = GL_UNSIGNED_BYTE;
		}
		else if (bpp == 64)
		{
			gl_internal_format = GL_RGBA16;
			gl_format = GL_RGBA;
			gl_type = GL_UNSIGNED_SHORT;
		}
		else if (bpp == 128)
		{
			gl_internal_format = GL_RGBA32F;
			gl_format = GL_RGBA;
			gl_type = GL_FLOAT;
		}
		else
		{
			assert(false);
		}

		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glPixelStorei(GL_PACK_ALIGNMENT, 1);

		// Reallocate if needed
		if (this->width != width || this->height != height || this->bpp != bpp || image == nullptr)
		{
			this->width = width;
			this->height = height;
			this->bpp = bpp;

			if (image != nullptr)
			{
				free(image);
			}

			dataSize = width * height * (bpp / 8);
			image = (unsigned char*)malloc(dataSize);
		}
		else
		{
			assert(dataSize == width * height * (bpp / 8));
		}

		if (rgbaData == nullptr)
		{
			memset(image, 0, dataSize);
		}
		else
		{
			memcpy(image, rgbaData, dataSize);
		}

		glTexImage2D(
			GL_TEXTURE_2D, 0,
			gl_internal_format,
			width, height, 0,
			gl_format,
			gl_type, rgbaData
		);
	}

	bool Load(const char *filepath)
	{
		unsigned char *loadedImage = nullptr;
		unsigned newWidth, newHeight;
		int err = lodepng_decode_file(&loadedImage, &newWidth, &newHeight, filepath, LCT_RGBA, 8);
		if (err != 0)
		{
			return false;
		}

		Initialize(newWidth, newHeight, 32, loadedImage);
		free(loadedImage);
		return true;
	}

	bool LoadGrey16(const char *filepath)
	{
		unsigned char *loadedImage = nullptr;
		unsigned newWidth, newHeight;
		int err = lodepng_decode_file(&loadedImage, &newWidth, &newHeight, filepath, LCT_GREY, 16);
		if (err != 0)
		{
			return false;
		}

		// Crappy lodepng actually loads the input swapped from native order :-/

		SwapArray((unsigned short *)loadedImage, newWidth*newHeight);
		Initialize(newWidth, newHeight, 16, loadedImage);
		free(loadedImage);
		return true;
	}

	bool Save(const std::string &filepath, bool isSRGB, bool flipY)
	{
		// Crappy lodepng actually expects the input to be swapped from native order :-/
		SwapPixels();

		if (flipY)
		{
			VerticalFlipPixels();
		}

		unsigned int error;

		if (bpp == 8)
		{
			error = lodepng_encode_file(filepath.c_str(), image, width, height, LCT_GREY, 8);
		}
		else if (bpp == 16)
		{
			error = lodepng_encode_file(filepath.c_str(), image, width, height, LCT_GREY, 16);
		}
		else if (bpp == 32)
		{
			error = lodepng_encode_file(filepath.c_str(), image, width, height, LCT_RGBA, 8);
		}
		else if (bpp == 64)
		{
			error = lodepng_encode_file(filepath.c_str(), image, width, height, LCT_RGBA, 16);
		}
		else if (bpp = 128)
		{
			const char *errorMsg;
			error = SaveEXR((float *)image, width, height, 4, /*fp16*/0, filepath.c_str(), &errorMsg);
			if (error != TINYEXR_SUCCESS)
			{
				if (errorMsg)
				{
					fprintf(stderr, "err: %s\n", errorMsg);
					FreeEXRErrorMessage(errorMsg);
				}
			}
		}

		if (flipY)
		{
			VerticalFlipPixels();
		}

		SwapPixels();
		return error == 0;
	}

	~Texture2D()
	{
		if (image) free(image);
		glDeleteTextures(1, &handle);
	}

	unsigned int dataSize;
	GLenum gl_format;
	GLenum gl_type;
	unsigned int uploadedScanline;
	unsigned int initializedData;// this is always in bytes not pixels

	void UploadScanlines()
	{
		int scanlineSize = width * (bpp / 8);
		int availableScanline = initializedData / scanlineSize;

		if (availableScanline == height || (availableScanline - uploadedScanline) > 50)
		{
			int scanlinesToUpload = availableScanline - uploadedScanline;
			glBindTexture(GL_TEXTURE_2D, handle);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0, uploadedScanline, width, scanlinesToUpload, gl_format, gl_type, image + uploadedScanline * scanlineSize);

			uploadedScanline += scanlinesToUpload;
		}
	}

	void ResetScanlines()
	{
		uploadedScanline = 0;
		initializedData = 0;
	}

	unsigned int width, height, bpp;
	unsigned char *image;
	GLuint handle;

	static inline unsigned short Swap(unsigned short s)
	{
		unsigned char *c = (unsigned char *)&s;
		unsigned short ret;
		unsigned char *cr = (unsigned char *)&ret;
		cr[0] = c[1];
		cr[1] = c[0];
		return ret;
	}

	static void SwapArray(unsigned short *s, int numItems)
	{
		for (int i = 0; i < numItems; i++)
		{
			s[i] = Swap(s[i]);
		}
	}

	void SwapPixels()
	{
		if (bpp == 16)
		{
			SwapArray((unsigned short *)image, width * height);
		}
		else if (bpp == 64)
		{
			SwapArray((unsigned short *)image, width * height * 4);
		}
	}

	void VerticalFlipPixels()
	{
		int half = height / 2;
		size_t scanlineSize = width * (bpp / 8);
		unsigned char *temp = new unsigned char[scanlineSize];
		
		for (int i = 0; i < half; i++)
		{
			memcpy(temp, image+i * scanlineSize, scanlineSize);
			memcpy(image+i * scanlineSize, image+(height - 1 -i)*scanlineSize, scanlineSize);
			memcpy(image + (height - 1 - i)*scanlineSize, temp, scanlineSize);
		}

		delete[] temp;
	}

	template<class PixelType> PixelType*GetPixelPtr(int x, int y)
	{
		assert(sizeof(PixelType) == bpp / 8);
		assert(x >= 0 && x < width);
		assert(y >= 0 && y < height);
		return ((PixelType *)image) + x + y * width;
	}
};
