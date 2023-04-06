#include "Trig.glsl"

bool sameHemisphere(vec3 w, vec3 wp) {
    return w.z * wp.z > 0.0f;
}


vec3 schlickF0FromRelativeIOR(float eta) {
    float a = (1.0 - eta) / (1.0 + eta);
    return vec3(a * a);
}

vec3 Fr_Schlick(float cosThetaI, vec3 f0) {
    float a = max(0.0f, 1.0f - cosThetaI);
    float a2 = a * a;
    float a5 = a2 * a2 * a;
    return f0 + (vec3(1.0f) - f0) * a5;
}

float D_GGX(vec3 wh, float alpha) {
    float alpha2 = alpha * alpha;
    float a = 1.0f + cosTheta2(wh) * (alpha2 - 1.0f);
    return alpha2 / (M_PI * a * a);
}

float G1_Smith_GGX(vec3 w, float alpha) {
    float tan2ThetaW = tanTheta2(w);
    if (tan2ThetaW > 1e30) return 0.0f;
    float alpha2 = alpha * alpha;
//    assert(alpha2 * tan2ThetaW >= -1.0f);
    float lambda = (-1.0f + sqrt(alpha2 * tan2ThetaW + 1.0f)) / 2.0f;
    return 1.0f / (1.0f + lambda);
}

float G2_SmithUncorrelated_GGX(vec3 wi, vec3 wo, float alpha) {
    return G1_Smith_GGX(wi, alpha) * G1_Smith_GGX(wo, alpha);
}

float G2_SmithHeightCorrelated_GGX(vec3 wi, vec3 wo, float alpha) {
    float tan2ThetaO = tanTheta2(wo);
    float tan2ThetaI = tanTheta2(wi);
    if (tan2ThetaO > 1e30) return 0.0f;
    if (tan2ThetaI > 1e30) return 0.0f;
    float alpha2 = alpha * alpha;
//    assert(alpha2 * tan2ThetaO >= -1.0f);
//    assert(alpha2 * tan2ThetaI >= -1.0f);
    float lambda_wo = (-1.0f + sqrt(alpha2 * tan2ThetaO + 1.0f)) / 2.0f;
    float lambda_wi = (-1.0f + sqrt(alpha2 * tan2ThetaI + 1.0f)) / 2.0f;
    return 1.0f / (1.0f + lambda_wo + lambda_wi);
}

float G2_None(vec3 wi, vec3 wo, float alpha) {
    return 1.0f;
}


// BxDF functions
vec3 diffuse_Lambert(vec3 wi, vec3 wo, vec3 diffuseColor) {
    if (!sameHemisphere(wi, wo)) {
        return vec3(0.0f);
    }

    return diffuseColor * M_ONE_OVER_PI;
}

vec3 microfacetReflection_GGX(vec3 wi, vec3 wo, vec3 f0, float eta, float alpha) {
    if (!sameHemisphere(wi, wo) || cosTheta(wi) == 0.0f || cosTheta(wo) == 0.0f) {
        return vec3(0.0f);
    }

    vec3 wh = wi + wo;
    if (wh.x == 0.0f && wh.y == 0.0f && wh.z == 0.0f) {
        return vec3(0.0f);
    }
    wh = normalize(wh);

    vec3 F;
    if (eta < 1.0f) {
        float cosThetaT = dot(wi, wh);
        float cos2ThetaT = cosThetaT * cosThetaT;
        F = cos2ThetaT > 0.0f ? Fr_Schlick(abs(cosThetaT), f0) : vec3(1.0f);
    }
    else {
        F = Fr_Schlick(abs(dot(wh, wo)), f0);
    }

    float G = G2_SmithHeightCorrelated_GGX(wi, wo, alpha);
    float D = D_GGX(wh, alpha);
    return F * G * D / (4.0f * abs(cosTheta(wi)) * abs(cosTheta(wo)));
}

vec3 microfacetTransmission_GGX(vec3 wi, vec3 wo, vec3 f0, float eta, float alpha) {
    if (sameHemisphere(wi, wo) || cosTheta(wi) == 0.0f || cosTheta(wo) == 0.0f) {
        return vec3(0.0f);
    }

    vec3 wh = normalize(wi + eta * wo);
    if (cosTheta(wh) < 0.0f) {
        wh = -wh;
    }

    bool sameSide = dot(wo, wh) * dot(wi, wh) > 0.0f;
    if (sameSide) {
        return vec3(0.0f);
    }

    vec3 F;
    if (eta < 1.0f) {
        float cosThetaT = dot(wi, wh);
        float cos2ThetaT = cosThetaT * cosThetaT;
        F = cos2ThetaT > 0.0f ? Fr_Schlick(abs(cosThetaT), f0) : vec3(1.0f);
    }
    else {
        F = Fr_Schlick(abs(dot(wh, wo)), f0);
    }

    float G = G2_SmithHeightCorrelated_GGX(wi, wo, alpha);
    float D = D_GGX(wh, alpha);
    float denomSqrt = dot(wi, wh) + eta * dot(wo, wh);
    return (vec3(1.0f) - F) * D * G * abs(dot(wi, wh)) * abs(dot(wo, wh))
        / (denomSqrt * denomSqrt * abs(cosTheta(wi)) * abs(cosTheta(wo)));
}


vec3 sampleUniformSphere(float u1, float u2) {
    float cosTheta = 1.0f - 2.0f * u1;
    float sinTheta = sqrt(max(0.0f, 1.0f - cosTheta * cosTheta));
    float phi = 2.0f * M_PI * u2;
    return vec3(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);
}

float pdfUniformSphere(vec3 wi, vec3 wo) {
    return 1.0f / (4.0f * M_PI);
}


vec3 sampleCosineHemisphere(float u1, float u2) {
    float cosTheta = sqrt(max(0.0f, 1.0f - u1));
    float sinTheta = sqrt(u1);
    float phi = 2.0f * M_PI * u2;
    return vec3(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);
}

float pdfCosineHemisphere(vec3 wi, vec3 wo) {
    return sameHemisphere(wi, wo) ? cosTheta(wi) / M_PI : 0.0f;
}


vec3 sampleGGX(float alpha, float u1, float u2) {
    float phi = 2.0f * M_PI * u1;
    float cosTheta2 = (1.0f - u2) / ((alpha * alpha - 1.0f) * u2 + 1.0f);
    float sinTheta = sqrt(max(0.0f, 1.0f - cosTheta2));
    return vec3(sinTheta * cos(phi), sinTheta * sin(phi), sqrt(cosTheta2));
}

float pdfGGX_reflection(vec3 wi, vec3 wo, float alpha) {
    if (!sameHemisphere(wi, wo)) {
        return 0.0f;
    }

    vec3 wh = normalize(wi + wo);
    float pdf_h = D_GGX(wh, alpha) * abs(cosTheta(wh));
    float dwh_dwi = 1.0f / (4.0f * dot(wi, wh));
    return pdf_h * dwh_dwi;
}

float pdfGGX_transmission(vec3 wi, vec3 wo, float eta, float alpha) {
    if (sameHemisphere(wi, wo)) {
        return 0.0f;
    }

    vec3 wh = normalize(wi + eta * wo);
    bool sameSide = dot(wo, wh) * dot(wi, wh) > 0.0f;
    if (sameSide) return 0.0f;

    float pdf_h = D_GGX(wh, alpha) * abs(cosTheta(wh));
    float sqrtDenom = dot(wi, wh) + eta * dot(wo, wh);
    float dwh_dwi = abs(dot(wi, wh)) / (sqrtDenom * sqrtDenom);
    return pdf_h * dwh_dwi;
}


// See: http://jcgt.org/published/0007/04/01/paper.pdf
vec3 sampleGGX_VNDF(vec3 wo, float alpha, float u1, float u2) {
//    // Transform view direction to hemisphere configuration
//    vec3 woHemi = normalize(vec3(alpha * wo.x, alpha * wo.y, wo.z));
//
//    // Create orthonormal basis
//    float length2 = woHemi.x * woHemi.x + woHemi.y * woHemi.y;
//    vec3 b1 = length2 > 0.0f
//        ? vec3(-woHemi.y, woHemi.x, 0.0f) * (1.0f / sqrt(length2))
//        : vec3(1.0f, 0.0f, 0.0f);
//    vec3 b2 = cross(woHemi, b1);
//
//    // Parameterization of projected area
//    float r = sqrt(u1);
//    float phi = 2.0f * M_PI * u2;
//    float t1 = r * cos(phi);
//    float t2 = r * sin(phi);
//    float s = 0.5f * (1.0f + woHemi.z);
//    t2 = (1.0f - s) * sqrt(1.0f - t1 * t1) + s * t2;
//
//    // Reprojection onto hemisphere
//    vec3 whHemi = t1 * b1 + t2 * b2 + sqrt(max(0.0f, 1.0f - t1 * t1 - t2 * t2)) * woHemi;
//
//    // Transforming half vector back to ellipsoid configuration
//    return normalize(vec3(alpha * whHemi.x, alpha * whHemi.y, max(0.0f, whHemi.z)));
    vec3 V = normalize(vec3(alpha * wo.x, alpha * wo.y, wo.z));
    // orthonormal basis
    vec3 T1 = (V.z < 0.9999f) ? normalize(cross(V, vec3(0, 0, 1))) : vec3(1, 0, 0);
    vec3 T2 = cross(T1, V);
    // sample point with polar coordinates (r, phi)
    float a = 1.0f / (1.0f + V.z);
    float r = sqrt(u1);
    float phi = (u2 < a) ? u2 / a * M_PI : M_PI + (u2 - a) / (1.0f - a) * M_PI;
    float P1 = r * cos(phi);
    float P2 = r * sin(phi) * ((u2 < a) ? 1.0f : V.z);
    // compute normal
    vec3 N = P1 * T1 + P2 * T2 + sqrt(max(0.0f, 1.0f - P1 * P1 - P2 * P2)) * V;
    // unstretch
    N = normalize(vec3(alpha * N.x, alpha * N.y, max(0.0f, N.z)));
    return N;
}

float pdfGGX_VNDF_reflection(vec3 wi, vec3 wo, float alpha) {
    if (!sameHemisphere(wi, wo)) {
        return 0.0f;
    }

    vec3 wh = normalize(wi + wo);
    float pdf_h = G1_Smith_GGX(wo, alpha) * D_GGX(wh, alpha) * abs(dot(wh, wo)) / abs(cosTheta(wo));
    float dwh_dwi = 1.0f / (4.0f * dot(wi, wh));
    return pdf_h * dwh_dwi;
}

float pdfGGX_VNDF_transmission(vec3 wi, vec3 wo, float eta, float alpha) {
//    if (sameHemisphere(wi, wo)) 
//    {
//        return 0.0f;
//    }

    vec3 wh = normalize(wi + eta * wo);
//    bool sameSide = dot(wo, wh) * dot(wi, wh) > 0.0f;
//    if (sameSide) return 0.0f;

    float pdf_h = G1_Smith_GGX(wo, alpha) * D_GGX(wh, alpha) * abs(dot(wh, wo)) / abs(cosTheta(wo));
    float sqrtDenom = dot(wi, wh) + eta * dot(wo, wh);
    float dwh_dwi = abs(dot(wi, wh)) / (sqrtDenom * sqrtDenom);
    return pdf_h * dwh_dwi;
}