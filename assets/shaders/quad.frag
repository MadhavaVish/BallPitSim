#version 460

layout (location = 0) in vec2 inUV;
layout (location = 0) out vec4 outFragColor;

layout (binding = 0) uniform sampler2D samplerColor;

layout(binding = 3) readonly uniform Nice{
    int windowWidth;
    int windowHeight;
    int width;
    int height;
    float vignette;
} ubo;

void main() {
    vec4 color = sqrt(texture(samplerColor, inUV));
    vec2 uv = inUV;
    uv *=  1 - uv.yx;
    float vig = uv.x*uv.y; // multiply with sth for intensity
    
    vig = min(pow(vig, 0.3 * ubo.vignette)+0.15, 1); // change pow for modifying the extend of the  vignette
        
    outFragColor = color * vig;
}