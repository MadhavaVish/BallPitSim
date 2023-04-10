#version 460


struct Uniformbuff
{
    mat4 ModelView;
    mat4 Projection;
    float Particlescale;
};
layout(binding = 0) readonly uniform UBOStruct { Uniformbuff camera; };
layout(std430, binding = 1) readonly buffer PositionBuffer {vec4 positions[]; };
layout(location = 0) in vec3 InPosition;
layout(location = 1) in vec3 InNormal;

layout(location = 0) out vec3 FragNormal;
void main() {
    gl_Position = camera.Projection * camera.ModelView * (vec4((camera.Particlescale * InPosition), 1.0) + positions[gl_InstanceIndex]);
    FragNormal = InNormal;
}