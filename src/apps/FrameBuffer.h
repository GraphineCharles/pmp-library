#pragma once

#include "Texture2D.h"

class FrameBuffer
{
public:

	FrameBuffer() : fbo(0) {}

	void Initialize(int width, int height, int bpp)
	{
		if (fbo)
		{
			glDeleteFramebuffers(1, &fbo);
			fbo = 0;
		}

		glGenFramebuffers(1, &fbo);
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);


		image.Initialize(width, height, bpp, nullptr);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, image.handle, 0);

		assert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

	~FrameBuffer() {
		if (fbo)
		{
			glDeleteFramebuffers(1, &fbo);
			fbo = 0;
		}
	}

	void BindForRendering()
	{
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		glViewport(0, 0, image.width, image.height);
	}

	// if with or height is 0 the viewport will not be reset!!!!
	static void BindSystemFrameBuffer(int width, int height)
	{
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		if (width && height)
		{
			glViewport(0, 0, width, height);
		}
	}

	void ReadPixels()
	{
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		glReadPixels(0, 0, image.width, image.height, image.gl_format, image.gl_type, image.image);
		image.VerticalFlipPixels();
	}

	unsigned int fbo;
	Texture2D image;
};