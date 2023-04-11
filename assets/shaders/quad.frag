#version 460


layout(location = 0) in vec3 FragNormal;
layout(location = 1) in vec4 inColor;
layout (location = 0) out vec4 outFragColor;
void main() 
{
    const vec3 lightVector = -normalize(vec3(5, 4, -3));
	const float d = max(dot(lightVector, normalize(FragNormal)), 0.4);
    outFragColor = inColor * d ;
}