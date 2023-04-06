
vec3 Schlick(vec3 r0, float radians)
{
    float exponential = pow(1.0f - radians, 5.0f);
    return r0 + (vec3(1.0f) - r0) * exponential;
}

float SchlickWeight(float u)
{
    float m = clamp((1.0f - u), 0.0, 1.0);
    float m2 = m * m;
    return m * m2 * m2;
}

float Schlick(float r0, float radians)
{
    return mix(1.0f, SchlickWeight(radians), r0);
}

float SchlickR0FromRelativeIOR(float eta)
{
    return pow((eta - 1.0f),2.0) / pow((eta + 1.0f),2.0);
}

float SchlickDielectic(float cosThetaI, float relativeIor)
{
    float r0 = SchlickR0FromRelativeIOR(relativeIor);
    return r0 + (1.0f - r0) * SchlickWeight(cosThetaI);
}

float Dielectric(float cosThetaI, float ni, float nt)
{
    // Copied from PBRT. This function calculates the full Fresnel term for a dielectric material.
    // See Sebastion Legarde's link above for details.

    cosThetaI = clamp(cosThetaI, -1.0f, 1.0f);

    // Swap index of refraction if this is coming from inside the surface
    if(cosThetaI < 0.0f) {
        float temp = ni;
        ni = nt;
        nt = temp;

        cosThetaI = -cosThetaI;
    }

    float sinThetaI = sqrt(max(0.0f, 1.0f - cosThetaI * cosThetaI));
    float sinThetaT = ni / nt * sinThetaI;

    // Check for total internal reflection
    if(sinThetaT >= 1.0) {
        return 1.0;
    }

    float cosThetaT = sqrt(max(0.0f, 1.0f - sinThetaT * sinThetaT));

    float rParallel     = ((nt * cosThetaI) - (ni * cosThetaT)) / ((nt * cosThetaI) + (ni * cosThetaT));
    float rPerpendicuar = ((ni * cosThetaI) - (nt * cosThetaT)) / ((ni * cosThetaI) + (nt * cosThetaT));
    return (rParallel * rParallel + rPerpendicuar * rPerpendicuar) / 2;
}
        