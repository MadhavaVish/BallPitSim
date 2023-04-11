#version 460


struct Uniformbuff
{
    mat4 ModelView;
    mat4 Projection;
    float Particlescale;
};
layout(binding = 0) readonly uniform UBOStruct { Uniformbuff camera; };
layout(std430, binding = 1) readonly buffer PositionBuffer {vec4 positions[]; };
layout(std430, binding = 2) readonly buffer MassBuffer {float masses[]; };
layout(std430, binding = 3) readonly buffer SpeedBuffer {float speeds[]; };
layout(location = 0) in vec3 InPosition;
layout(location = 1) in vec3 InNormal;

layout(location = 0) out vec3 FragNormal;
layout(location = 1) out vec4 outColor;
void main() {
    gl_Position = camera.Projection * camera.ModelView * (vec4((camera.Particlescale*0.001 * masses[gl_InstanceIndex]/5 * InPosition), 1.0) + positions[gl_InstanceIndex]);
    FragNormal = InNormal;
    outColor = mix(vec4(1,1,1,1), vec4(1,0,0,1), vec4(speeds[gl_InstanceIndex]/20));
}