#define M_PI 3.1415926535897932384626433832795
#define M_TWOPI 6.283185307179586476925
#define M_ONE_OVER_PI 0.3183098861837906715378

float cosTheta(vec3 w) { return w.z; }
float cosTheta2(vec3 w) { return w.z * w.z; }
float sinTheta2(vec3 w) { return max(0.0f, 1.0f - cosTheta2(w)); }
float sinTheta(vec3 w) { return sqrt(sinTheta2(w)); }
float tanTheta(vec3 w) { return sinTheta(w) / cosTheta(w); }
float tanTheta2(vec3 w) { return sinTheta2(w) / cosTheta2(w); }