
bool IntersectTriangle(in Ray ray, vec3 v0, vec3 v1, vec3 v2, inout Intersection isect)
{
    vec3 A = v1 - v0;
    vec3 B = v2 - v0;
    vec3 pvec = cross(ray.direction, B);
    float det = dot(A, pvec);

    float invDeterminant = 1.0 / det;

    vec3 tvec = ray.origin - v0;
    float u = dot(tvec, pvec) * invDeterminant;
    if (u < 0 || u  > 1) return false;

    vec3 qvec = cross(tvec, A);
    float v = dot(ray.direction, qvec) * invDeterminant;
    if (v < 0 || u + v > 1) return false;

    float t = dot(B, qvec) * invDeterminant;
    if (t < isect.t_hit && t > 0)
    {
        isect.t_hit = t;
        isect.barycentric = vec2(u, v);
        return true;

    }
    return false;
}

float IntersectAABB(in Ray ray, in vec3 bmin, in vec3 bmax, in float t_hit)
{
	float tx1 = (bmin.x - ray.origin.x) * ray.invDir.x, tx2 = (bmax.x - ray.origin.x) * ray.invDir.x;
	float tmin = min(tx1, tx2), tmax = max(tx1, tx2);
	float ty1 = (bmin.y - ray.origin.y) * ray.invDir.y, ty2 = (bmax.y - ray.origin.y) * ray.invDir.y;
	tmin = max(tmin, min(ty1, ty2)), tmax = min(tmax, max(ty1, ty2));
	float tz1 = (bmin.z - ray.origin.z) * ray.invDir.z, tz2 = (bmax.z - ray.origin.z) * ray.invDir.z;
	tmin = max(tmin, min(tz1, tz2)), tmax = min(tmax, max(tz1, tz2));
	if (tmax >= tmin && tmin < t_hit && tmax > 0) return tmin; else return 1e30f;
}

bool IntersectBVH(in Ray ray, inout Intersection isect)
{
	BVHNode node = bvhNodes[0], stack[32];
	uint stackPtr = 0;
	bool hit = false; 
	while (true)
	{
		if (node.triCount > 0)
		{
			for (uint i = 0; i < node.triCount; i++)
			{
				Tri tri = triangles[node.leftFirst + i];
				vec3 v0 = vertices[tri.modelOffset + indices[tri.v_indices]].position;
				vec3 v1 = vertices[tri.modelOffset + indices[tri.v_indices + 1]].position;
				vec3 v2 = vertices[tri.modelOffset + indices[tri.v_indices + 2]].position;
				if (IntersectTriangle(ray, v0, v1, v2, isect))
				{
					hit = true;
					isect.objIdx = node.leftFirst + i;
				}
				continue;
			}
			if (stackPtr == 0)
			{
				break;
			}
			else node = stack[--stackPtr];
			continue;
		}
		BVHNode child1 = bvhNodes[node.leftFirst];
		BVHNode child2 = bvhNodes[node.leftFirst + 1];
		vec3 aabbMin = vec3(child1.minx, child1.miny, child1.minz);
		vec3 aabbmax = vec3(child1.maxx, child1.maxy, child1.maxz);
		float dist1 = IntersectAABB(ray, aabbMin, aabbmax, isect.t_hit);
		aabbMin = vec3(child2.minx, child2.miny, child2.minz);
		aabbmax = vec3(child2.maxx, child2.maxy, child2.maxz);
		float dist2 = IntersectAABB(ray, aabbMin, aabbmax, isect.t_hit);

		if (dist1 > dist2) 
		{ 
			float d = dist1; dist1 = dist2; dist2 = d;
			BVHNode c = child1; child1 = child2; child2 = c; 
		}
		if (dist1 == 1e30f)
		{
			if (stackPtr == 0)
			{
				break; 
			}
			else
			{
				node = stack[--stackPtr];
			}
		}
		else
		{
			node = child1;
			if (dist2 != 1e30f) stack[stackPtr++] = child2;
		}
	}
	return hit;
}