//#extension GL_EXT_control_flow_attributes : require


// Generates a seed for a random number generator from 2 inputs plus a backoff
// https://github.com/nvpro-samples/optix_prime_baking/blob/332a886f1ac46c0b3eea9e89a59593470c755a0e/random.h
// https://github.com/nvpro-samples/vk_raytracing_tutorial_KHR/tree/master/ray_tracing_jitter_cam
// https://en.wikipedia.org/wiki/Tiny_Encryption_Algorithm
//uint InitRandomSeed(uint val0, uint val1)
//{
//	uint v0 = val0, v1 = val1, s0 = 0;
//
//	[[unroll]] 
//	for (uint n = 0; n < 16; n++)
//	{
//		s0 += 0x9e3779b9;
//		v0 += ((v1 << 4) + 0xa341316c) ^ (v1 + s0) ^ ((v1 >> 5) + 0xc8013ea4);
//		v1 += ((v0 << 4) + 0xad90777d) ^ (v0 + s0) ^ ((v0 >> 5) + 0x7e95761e);
//	}
//
//	return v0;
//}
//
//uint RandomInt(inout uint seed)
//{
//	// LCG values from Numerical Recipes
//    return (seed = 1664525 * seed + 1013904223);
//}
//
//float RandomFloat(inout uint seed)
//{
//	//// Float version using bitmask from Numerical Recipes
//	//const uint one = 0x3f800000;
//	//const uint msk = 0x007fffff;
//	//return uintBitsToFloat(one | (msk & (RandomInt(seed) >> 9))) - 1;
//
//	// Faster version from NVIDIA examples; quality good enough for our use case.
//	return (float(RandomInt(seed) & 0x00FFFFFF) / float(0x01000000));
//}

//vec3 sampleCosineHemisphere(inout float pdf, inout uint seed)
//{
//	float a = RandomFloat(seed) * M_TWOPI;
//	float b = RandomFloat(seed);
//	float c = sqrt(1 - b);
//	float d = sqrt(b);
//	pdf = d * M_ONE_OVER_PI;
//	
//	return vec3(cos(a)*c, sin(a) * c, d);
//}
//

uint randSeed( ivec2 p, uint frame )
{
    uint n = frame;
    n = (n<<13)^n; n=n*(n*n*15731+789221)+1376312589; // by Hugo Elias
    n += p.y;
    n = (n<<13)^n; n=n*(n*n*15731+789221)+1376312589;
    n += p.x;
    n = (n<<13)^n; n=n*(n*n*15731+789221)+1376312589;
    return n;
}

uint WangHash(in uint s ) 
{ 
	s = (s ^ 61) ^ (s >> 16);
	s *= 9, s = s ^ (s >> 4);
	s *= 0x27d4eb2d;
	s = s ^ (s >> 15); 
	return s; 
}

uint RandomInt( inout uint s ) 
{
	s ^= s << 13;
	s ^= s >> 17;
	s ^= s << 5; 
	return s; 
}

float RandomFloat( inout uint s ) 
{ 
	return RandomInt( s ) * 2.3283064365387e-10f; /* = 1 / (2^32-1) */ 
} 