struct Frame
{
	vec3 m_X, m_Y, m_Z;
};

Frame setFromUp(in vec3 up)
{
	Frame frame;
	frame.m_Z = up;
	vec3 temp = abs(frame.m_Z.x) > 0.99f ? vec3(0.f, 1.f, 0.f) : vec3(1.f, 0.f, 0.f);
	frame.m_X = normalize(cross(frame.m_Z, temp));
	frame.m_Y = cross(frame.m_X, frame.m_Z);

    return frame;
}

vec3 ToWorld(in Frame frame, in vec3 a)
{
//	return frame.trans * a;
    return frame.m_X * a.x + frame.m_Y * a.y + frame.m_Z * a.z;
}

vec3 ToLocal(in Frame frame, in vec3 a)
{
//    return vec3(dot(v, frame._x), dot(v, frame._y), dot(v, frame._z));
	return vec3(dot(a, frame.m_X), dot(a, frame.m_Y),dot(a, frame.m_Z));
}