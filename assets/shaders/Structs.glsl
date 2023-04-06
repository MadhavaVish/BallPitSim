struct RayGenUBO
{
	vec4 position;
	vec4 coord_x;
	vec4 coord_y;
	vec4 coord_z;
	vec4 horizontal;
	vec4 vertical;
	float invWidth;
	float invHeight;
	float focusDist;
	float aperture;
	uint frameIndex;
};

struct Ray
{
	vec3 origin;
	vec3 direction;
	vec3 invDir;
};

struct Intersection
{
	float  t_hit;
	uint   objIdx;
	uint   lightIdx;
	vec2   barycentric;
};

struct Vertex
{
	vec3 position;
	float u;
};
struct Normal 
{
	vec3 normal;
	float v;
};

struct Material
{
	vec3 albedo;
	float emission;
};

struct BVHNode
{
	float minx;
	float miny;
	float minz;
	uint leftFirst;
	float maxx;
	float maxy;
	float maxz;
	uint triCount;
};

struct Tri
{
	uint shadeSmooth;
	uint materialIdx;
	uint modelOffset;
	uint v_indices;
};

void getPrimaryRay(in uint x,in uint y, in vec2 offset, in RayGenUBO ubo, inout Ray r)
{
	float u = (x + offset.x) * ubo.invWidth;
	float v = (y + offset.y) * ubo.invHeight;
	u = u * 2 - 1;
	v = v * 2 - 1;
	vec3 dir = vec3(normalize(normalize(ubo.coord_z * ubo.focusDist + u * ubo.horizontal - v * ubo.vertical) * ubo.focusDist));
	r.origin = vec3(ubo.position);
	r.direction = dir;
	r.invDir = (1.0/dir);
}

Ray getRay(in vec3 o, in vec3 dir)
{
	Ray r;
	r.origin = o;
	r.direction = dir;
	r.invDir = 1.0/dir;
	return r;
}

vec3 rayPnt(in Ray r, in float t)
{
	return r.origin + r.direction * t;
}
