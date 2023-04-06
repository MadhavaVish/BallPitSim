#version 460

vec2 positions[6] = vec2[](
    vec2(-1.0, 1.0),
    vec2(1.0, -1.0),
    vec2(-1.0, -1.0),
    vec2(1.0, -1.0),
    vec2(-1.0, 1.0),
    vec2(1.0, 1.0)

);
vec2 texCoords[6] = vec2[]
(
    vec2(0.0, 0.0),
    vec2(1.0, 1.0),
    vec2(0.0, 1.0),
    vec2(1.0, 1.0),
    vec2(0.0, 0.0),
    vec2(1.0, 0.0)
);
layout(binding = 3) readonly uniform Nice{
    int windowWidth;
    int windowHeight;
    int width;
    int height;
    float vignette;
} ubo;

layout (location = 0) out vec2 outUV;

void main() {

    float viewportAspect = float(ubo.windowWidth)/float(ubo.windowHeight);

    float aspect = float(ubo.width)/float(ubo.height);
    vec2 pos = positions[gl_VertexIndex];

    if( viewportAspect < aspect)
    {
        pos.y *= viewportAspect;
        pos.y /= aspect;
    }
    else
    {
        pos.x /= viewportAspect;
        pos.x *= aspect;
    }

    outUV = texCoords[gl_VertexIndex];
    gl_Position = vec4(pos, 0.0, 1.0);

  
}