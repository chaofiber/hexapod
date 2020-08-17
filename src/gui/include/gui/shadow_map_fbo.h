#pragma once
#include <glad/glad.h>

class ShadowMapFBO
{
public:
    ShadowMapFBO();

    ~ShadowMapFBO();

    bool Init(GLuint bufferWidth, GLuint bufferHeight);

    void BindForWriting();

    void BindForReading(GLuint TextureUnit);
    GLuint bufferWidth, bufferHeight;
	GLuint fbo;
    GLuint shadowMap;
};