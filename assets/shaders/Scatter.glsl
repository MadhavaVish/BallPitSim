#include "GGX.glsl"
//#include "Frame.glsl"
//#include "Random.glsl"
//#include "Fresnel.glsl"
#include "Random.glsl"

struct Mat
{
	vec3 baseColor_;
    float roughness_;
    float metalness_;
    float transmission_;
    float ior_;
    vec3 emittance_;

    // Precomputed values
    float alpha_;
};

float computeRelativeIOR(in Mat mat, in vec3 wo) {
    bool entering = cosTheta(wo) > 0.0f;
    return entering ? 1.0f /  mat.ior_ :  mat.ior_;
}

void computeLobeProbabilities(in Mat mat, in vec3 wo, out float pDiffuse, out float pSpecular, out float pTransmission) {
    float eta = computeRelativeIOR(mat, wo);
    vec3 f0 = mix(schlickF0FromRelativeIOR(eta), mat.baseColor_, mat.metalness_);
    vec3 fresnel = Fr_Schlick(abs(cosTheta(wo)), f0);

    float diffuseWeight = (1.0f - mat.metalness_) * (1.0f - mat.transmission_);
    float transmissionWeight = (1.0f - mat.metalness_) * mat.transmission_;
    vec3 diff = (mat.baseColor_) ;
    pDiffuse = max(diff.x, max(diff.y, diff.z))* diffuseWeight;
    pSpecular = max(fresnel.x, max(fresnel.y,fresnel.z));
    vec3 trans = vec3(1.0f) - fresnel;
    pTransmission = max(trans.x, max(trans.y, trans.z)) * transmissionWeight;

    float normFactor = 1.0f / (pDiffuse + pSpecular + pTransmission);
    pDiffuse *= normFactor;
    pSpecular *= normFactor;
    pTransmission *= normFactor;
}

bool refr(vec3 wi, vec3 n, float eta, inout vec3 wt, float ior) {
    float cosThetaI = dot(n, wi);
    float sin2ThetaI = max(0.f, 1.f - cosThetaI * cosThetaI);
    float sin2ThetaT = eta * eta * sin2ThetaI;
    if (sin2ThetaT >= 1.f) {
        return false;
    }
    float cosThetaT = sqrt(1.f - sin2ThetaT);
    wt = eta * -wi + (eta * cosThetaI - cosThetaT) * n;
    return true;
}
vec3 refl(in vec3 w, in vec3 n) {
    return 2.0 * dot(w, n) * n - w;
}
vec3 sampleDirection(inout uint seed, in Mat mat, in vec3 wo, inout float pdf) {
    float eta = computeRelativeIOR(mat, wo);
    float pDiffuse, pSpecular, pTransmission;
    computeLobeProbabilities(mat, wo, pDiffuse, pSpecular, pTransmission);

    vec3 wi;
    float r = RandomFloat(seed);
    float u1 = RandomFloat(seed);
    float u2 = RandomFloat(seed);
    if (r < pDiffuse) {

        wi = sign(cosTheta(wo)) * sampleCosineHemisphere(u1, u2);
    }
    else if (r < pDiffuse + pSpecular) {

        vec3 wo_upper = sign(cosTheta(wo)) * wo; // sign(+wo) * +wo = +wo, sign(-wo) * -wo = +wo
        vec3 wh = sign(cosTheta(wo)) * sampleGGX_VNDF(wo_upper, mat.alpha_, u1, u2);
        if (dot(wo, wh) < 0.0f) {
            return vec3(0.0f);
        }

        wi = refl(wo, wh);
        if (!sameHemisphere(wi, wo)) {
            wi = -wi;
//            return vec3(0.0f);
        }
    }
    else {

        vec3 wo_upper =  sign(cosTheta(wo))*wo; // sign(+wo) * +wo = +wo, sign(-wo) * -wo = +wo
        vec3 wh = sign(cosTheta(wo)) * sampleGGX_VNDF(wo_upper, mat.alpha_, u1, u2);
        refr(wo, wh, eta, wi, mat.ior_);
    }

    
    pdf = pDiffuse * pdfCosineHemisphere(wi, wo) + pSpecular * pdfGGX_VNDF_reflection(wi, wo, mat.alpha_) + pTransmission * pdfGGX_VNDF_transmission(wi, wo, eta, mat.alpha_);
    
    return wi;
}
vec3 lerp(vec3 x, vec3 y, float t) {
	return x * (1.f - t) + y * t;
}
vec3 evaluate(in Mat mat, in vec3 wi, in vec3 wo) {
    float eta = computeRelativeIOR(mat, wo);
    vec3 f0 = lerp(schlickF0FromRelativeIOR(eta), mat.baseColor_, mat.metalness_);

    vec3 diffuse = diffuse_Lambert(wi, wo, mat.baseColor_);
    vec3 specular = microfacetReflection_GGX(wi, wo, f0, eta, mat.alpha_);
    vec3 transmission = mat.baseColor_ * microfacetTransmission_GGX(wi, wo, f0, eta, mat.alpha_);
    
    float diffuseWeight = (1.0f - mat.metalness_) * (1.0f - mat.transmission_);
    float transmissionWeight = (1.0f - mat.metalness_) * mat.transmission_;

    return diffuseWeight * diffuse + specular + transmissionWeight * transmission;
}