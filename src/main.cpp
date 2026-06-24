#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"
#include "material.hpp"

#include <string>
#include <random>
#include <algorithm>

using namespace std;
template<typename T>
T new_clamp(T val, T minv, T maxv) {
    return val < minv ? minv : (val > maxv ? maxv : val);
}
float evaluateChebyshevT(int n, float x) {
    if (n == 0) return 1.0f;
    if (n == 1) return x;
    float t0 = 1.0f;
    float t1 = x;
    float t_n = x;
    for (int i = 2; i <= n; ++i) {
        t_n = 2.0f * x * t1 - t0;
        t0 = t1;
        t1 = t_n;
    }
    return t_n;
}
double choose(int n, int k) {
    if (k < 0 || k > n) return 0.0;
    if (k == 0 || k == n) return 1.0;
    double res = 1.0;
    for (int i = 1; i <= k; ++i) {
        res = res * (n - k + i) / i;
    }
    return res;
}

// 🌟 完美对齐 scene07 物理线性亮度的【系数计算函数】
void computeCelShadingCoefficients(int K, std::vector<float>& alpha) {
    alpha.assign(K + 1, 0.0f);
    int num_samples = 100; 
    
    // 🌟 严格尊崇论文 S5.2 节：针对物理区间 [-1, 4] 拟合。
    // 根据场景物体的材质反射率，分界线定在 0.50f 处效果最为明显（区分明暗面）
    auto target_style_m = [](float u) {
        return (u < 0.6f) ? 0.4f : 0.9f;
    };

    for (int i = 0; i <= K; ++i) {
        float sum = 0.0f;
        for (int j = 1; j <= num_samples; ++j) {
            float t = cosf((2.0f * j - 1.0f) / (2.0f * num_samples) * M_PI); 
            float u = 0.5f * (t + 1.0f);
            float g_val = target_style_m(u); 
            float T_i = evaluateChebyshevT(i, t);
            
            sum += g_val * T_i;
        }
        alpha[i] = sum * (i == 0 ? 1.0f : 2.0f) / (float)num_samples;
    }
}

Vector3f sampleHemisphere(const Vector3f &n) {
    // 💡 升级为 thread_local，确保多线程并行时每个线程拥有独立的随机数引擎，绝不卡死冲突
    thread_local std::random_device rd;
    thread_local std::mt19937 gen(rd());
    thread_local std::uniform_real_distribution<float> dis(0.0, 1.0);

    float u1 = dis(gen);
    float u2 = dis(gen);

    float r = sqrt(u1);
    float theta = 2.0f * M_PI * u2;

    float x = r * cos(theta);
    float y = r * sin(theta);
    float z = sqrt(1.0f - u1);

    // 建立正交基
    Vector3f w = n.normalized();
    Vector3f a = (fabs(w.x()) > 0.9f) ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0);
    Vector3f v = Vector3f::cross(w, a).normalized();
    Vector3f u = Vector3f::cross(v, w);

    return (x * u + y * v + z * w).normalized();
}
Vector3f sampleGGXNormal(const Vector3f &N, float roughness, float u1, float u2) {
    float alpha = roughness * roughness;
    float alpha2 = alpha * alpha;

    float cosTheta2 = (1.0f - u1) / (u1 * (alpha2 - 1.0f) + 1.0f);
    float cosTheta = sqrtf(cosTheta2);
    float sinTheta = sqrtf(std::max(0.0f, 1.0f - cosTheta2));
    float phi = 2.0f * M_PI * u2;

    float x = sinTheta * cosf(phi);
    float y = sinTheta * sinf(phi);
    float z = cosTheta;

    Vector3f w = N.normalized();
    Vector3f a = (fabs(w.x()) > 0.9f) ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0);
    Vector3f v = Vector3f::cross(w, a).normalized();
    Vector3f u = Vector3f::cross(v, w);

    return (x * u + y * v + z * w).normalized();
}

float pdfGGX(const Vector3f &N, const Vector3f &H, const Vector3f &V, float roughness) {
    float alpha = roughness * roughness;
    float alpha2 = alpha * alpha;
    float cosHN = std::max(0.0f, Vector3f::dot(H, N));
    float cosHV = std::max(0.0f, Vector3f::dot(H, V));
    if (cosHN <= 0.0f || cosHV <= 0.0f) return 0.0f;
    float denom = (cosHN * cosHN * (alpha2 - 1.0f) + 1.0f);
    float D = alpha2 / (M_PI * denom * denom);
    return D * cosHN / (4.0f * cosHV + 1e-5f);
}
// 评估 BRDF 值：传入入射L, 出射V, 法线N, 材质, 基础UV
Vector3f evaluateBRDF(const Vector3f &N, const Vector3f &V, const Vector3f &L, Material *m, float u, float v) {
    float cosThetaP = std::max(0.0f, Vector3f::dot(N, L));
    float cosViewN  = std::max(0.0f, Vector3f::dot(N, V));
    if (cosThetaP <= 0.0f || cosViewN <= 0.0f) return Vector3f::ZERO;

    float roughness = new_clamp(1.0f - (m->getShininess() / 100.0f), 0.05f, 1.0f);
    float alpha2 = roughness * roughness * roughness * roughness;

    // 漫反射部分
    Vector3f fr_diffuse = m->getDiffuseColor(u, v) / M_PI;

    // 高光部分 (GGX)
    Vector3f H = (L + V).normalized();
    float cosHN = std::max(0.0f, Vector3f::dot(H, N));
    float cosHV = std::max(0.0f, Vector3f::dot(H, V));
    
    float denomD = (cosHN * cosHN * (alpha2 - 1.0f) + 1.0f);
    float D_val = alpha2 / (M_PI * denomD * denomD + 1e-5f);
    
    Vector3f F0 = m->getSpecularColor().length() > 1e-5 ? m->getSpecularColor() : Vector3f(0.04f);
    Vector3f F = F0 + (Vector3f(1.0f) - F0) * powf(1.0f - cosHV, 5.0f);
    
    float k = (roughness + 1.0f) * (roughness + 1.0f) / 8.0f;
    float g1 = cosViewN / (cosViewN * (1.0f - k) + k);
    float g2 = cosThetaP / (cosThetaP * (1.0f - k) + k);
    
    Vector3f fr_specular = (D_val * F * g1 * g2) / (4.0f * cosThetaP * cosViewN + 1e-4f);

    return fr_diffuse + fr_specular;
}

// 评估材质采样的 PDF：当前方向采样到 L 的概率密度
float evaluateBRDFPdf(const Vector3f &N, const Vector3f &V, const Vector3f &L, Material *m) {
    float cosThetaNext = std::max(0.0f, Vector3f::dot(N, L));
    if (cosThetaNext <= 0.0f) return 0.0f;

    float p_diffuse = m->getDiffuseColor().length() / (m->getDiffuseColor().length() + m->getSpecularColor().length() + 1e-5f);
    float p_specular = 1.0f - p_diffuse;

    // 漫反射 PDF (余弦半球采样)
    float pdf_diffuse_eval = cosThetaNext / M_PI;

    // 高光 PDF (GGX)
    Vector3f H = (L + V).normalized();
    float roughness = new_clamp(1.0f - (m->getShininess() / 100.0f), 0.05f, 1.0f);
    float pdf_specular_eval = pdfGGX(N, H, V, roughness); // 你现有的 pdfGGX 函数

    return pdf_diffuse_eval * p_diffuse + pdf_specular_eval * p_specular;
}
Vector3f traceRay(const Ray &ray, float tmin, int depth, int maxDepth, SceneParser &sceneParser, bool isSpecular = true, bool usePathTracing = true)
{
    Hit hit; 
    Group *baseGroup = sceneParser.getGroup();
    Ray currentRay = ray;
    float currentTmin = tmin;
    bool isIntersect = false;
    hit = Hit(); 
    isIntersect = baseGroup->intersect(currentRay, hit, currentTmin);

    if (!isIntersect || hit.getT() > 20.0f)
    {
        return sceneParser.getBackgroundColor();
    }
    if (depth >= maxDepth || depth >= 15) {
        return Vector3f::ZERO;
    }

    thread_local std::random_device rd;
    thread_local std::mt19937 gen(rd());
    thread_local std::uniform_real_distribution<float> dis(0.0f, 1.0f);

    float P_RR = 0.8f;
    float rrScale = 1.0f;
    if (usePathTracing && depth > 0) {
        if (dis(gen) > P_RR) return Vector3f::ZERO;
        rrScale = 1.0f / P_RR;
    }

    Material *m = hit.getMaterial();
    Vector3f N = hit.getNormal().normalized(); 
    Vector3f D = ray.getDirection().normalized(); 
    Vector3f V = -D; 
    Vector3f p = ray.pointAtParameter(hit.getT());
    Vector2f final_uv = hit.getUV();
    float safe_u = fmod(final_uv.x(), 1.0f); if (safe_u < 0.0f) safe_u += 1.0f;
    float safe_v = fmod(final_uv.y(), 1.0f); if (safe_v < 0.0f) safe_v += 1.0f;
    Vector3f emission = m->getEmission();
    Vector3f N_diffuse = N;
    if (Vector3f::dot(D, N_diffuse) > 0.0f) {
        N_diffuse = -N_diffuse; 
    }
    bool hitRealLightPatch = false;
    if (emission.length() > 1e-6f) {
        for (int i = 0; i < sceneParser.getNumLights(); ++i) {
            AreaLight* areaLight = dynamic_cast<AreaLight*>(sceneParser.getLight(i));
            if (areaLight != nullptr) {
                Vector3f p00 = areaLight->getSamplePoint(0, 0);
                Vector3f d1 = areaLight->getSamplePoint(1, 0) - p00;
                Vector3f d2 = areaLight->getSamplePoint(0, 1) - p00;
                
                float u_l = Vector3f::dot(p - p00, d1) / d1.squaredLength();
                float v_l = Vector3f::dot(p - p00, d2) / d2.squaredLength();
                
                if (u_l >= 0.0f && u_l <= 1.0f && v_l >= 0.0f && v_l <= 1.0f) {
                    hitRealLightPatch = true;
                    break;
                }
            }
        }
    }

    if (m->getEmission().length() > 1e-6f) {
        if (hitRealLightPatch) {
            return emission; 
        } else {
            emission = Vector3f::ZERO; 
        }
    }

    Vector3f reflectiveColor = m->getReflectiveColor();
    float refractiveIndex = m->getRefractiveIndex();
    bool isReflective = (reflectiveColor.length() > 1e-6);
    bool isRefractive = (std::abs(refractiveIndex - 1.0f) > 1e-6);

    if (isRefractive) {
        float dotDN = Vector3f::dot(D, N);
        bool leaving = (dotDN > 0.0f); 

        float n_i = leaving ? refractiveIndex : 1.0f;
        float n_t = leaving ? 1.0f : refractiveIndex;
        float ratio = n_i / n_t;

        Vector3f Ng = leaving ? -N : N;
        float cos_i = std::abs(Vector3f::dot(D, N)); 
        float sin2_t = ratio * ratio * (1.0f - cos_i * cos_i);
        float cos_t = (sin2_t <= 1.0f) ? sqrt(1.0f - sin2_t) : 0.0f;
        
        float R0 = powf((n_i - n_t) / (n_i + n_t), 2.0f);
        float Fr = R0 + (1.0f - R0) * powf(1.0f - cos_i, 5.0f);
        Fr = new_clamp(Fr, 0.0f, 1.0f);

        Vector3f transmissionColor = m->getDiffuseColor(safe_u, safe_v);
        float transFactor = m->getTransparency();
        Vector3f L_specular_nee = Vector3f::ZERO;
        for (int i = 0; i < sceneParser.getNumLights(); ++i) {
            Light* light = sceneParser.getLight(i);
            Vector3f L_dir; Vector3f radianceFactor; float dist;
            
            if (light->sampleNEE(p, N, dis(gen), dis(gen), L_dir, radianceFactor, dist)) {
                Vector3f R_dir = (D - 2.0f * Vector3f::dot(D, Ng) * Ng).normalized();
                float specAngle = Vector3f::dot(R_dir, L_dir);
                
                if (specAngle > 0.0f) {
                    Vector3f shadowRayOrigin = p + Ng * 5e-3f; 
                    Vector3f shadowRayDir = L_dir;             
                    Vector3f shadowAttenuation = Vector3f(1.0f); 
                    Vector3f realLightPos = p + L_dir * dist; 
                    
                    while (true) {
                        Ray shadowRay(shadowRayOrigin, shadowRayDir);
                        Hit shadowHit; 
                        bool hasHit = baseGroup->intersect(shadowRay, shadowHit, 1e-4f);
                        float currentDistToLight = (realLightPos - shadowRayOrigin).length() - 1e-3f;
                        
                        if (!hasHit || shadowHit.getT() >= currentDistToLight) break;
                        if (shadowHit.getMaterial()->getEmission().length() > 1e-6f) break;
                        
                        float h_refIdx = shadowHit.getMaterial()->getRefractiveIndex();
                        if (std::abs(h_refIdx - 1.0f) > 1e-6) {
                            shadowAttenuation = shadowAttenuation * shadowHit.getMaterial()->getDiffuseColor();
                            shadowRayOrigin = shadowRay.pointAtParameter(shadowHit.getT() + 5e-3f);
                        }
                        else {
                            shadowAttenuation = Vector3f::ZERO; break;
                        }
                    }
                    
                    if (shadowAttenuation.length() > 1e-5f) {
                        // 🌟 修复：去掉硬 Clamp，确保蒙特卡洛高阶项无偏
                        Vector3f singleSpecularContrib = radianceFactor * shadowAttenuation * powf(specAngle, 50.0f) * Fr;
                        L_specular_nee += singleSpecularContrib;
                        Vector3f transmittedDirect = radianceFactor * shadowAttenuation * (1.0f - Fr) * transmissionColor * transFactor;
                        L_specular_nee += transmittedDirect;
                    }
                }
            }
        }

        if (sin2_t > 1.0f || dis(gen) < Fr) { 
            Vector3f R = (D - 2.0f * Vector3f::dot(D, Ng) * Ng).normalized();
            Vector3f indirect = traceRay(Ray(p + R * 5e-3f, R), 1e-4f, depth + 1, maxDepth, sceneParser, true, usePathTracing);
            return L_specular_nee * reflectiveColor + indirect * reflectiveColor * rrScale;
        } else { 
            Vector3f T = (ratio * D + (ratio * cos_i - cos_t) * Ng).normalized();
            Vector3f indirect = traceRay(Ray(p + T * 5e-3f, T), 1e-4f, depth + 1, maxDepth, sceneParser, true, usePathTracing);
            Vector3f attenuation = transmissionColor * m->getTransparency();
            return indirect * attenuation * rrScale;
        }
    }
    else if (isReflective) {
        Vector3f R = (D - 2.0f * Vector3f::dot(D, N) * N).normalized();
        Vector3f offsetP = p + R * 5e-3f;
        Vector3f indirect = traceRay(Ray(offsetP, R), 1e-4f, depth + 1, maxDepth, sceneParser, true, usePathTracing);
        return indirect * reflectiveColor * rrScale;
    }

    Vector3f L_direct = Vector3f::ZERO;
    float roughness = new_clamp(1.0f - (m->getShininess() / 100.0f), 0.05f, 1.0f);
    float alpha = roughness * roughness;
    float alpha2 = alpha * alpha;

    for (int i = 0; i < sceneParser.getNumLights(); ++i) {
        Light* light = sceneParser.getLight(i);
        Vector3f L_dir; Vector3f radianceFactor; float dist;
        
        if (light->sampleNEE(p, N, dis(gen), dis(gen), L_dir, radianceFactor, dist)) {
            float cosThetaP = Vector3f::dot(N_diffuse, L_dir);
            float cosViewN  = std::max(0.0f, Vector3f::dot(N, V)); 
            
            if (cosThetaP > 0.0f && cosViewN > 0.0f) {
                Vector3f shadowRayOrigin = p + N_diffuse * 5e-3f; 
                Vector3f shadowRayDir = L_dir;             
                Vector3f shadowAttenuation = Vector3f(1.0f); 
                Vector3f realLightPos = p + L_dir * dist; 
                
                while (true) {
                    Ray shadowRay(shadowRayOrigin, shadowRayDir);
                    Hit shadowHit; 
                    bool hasHit = baseGroup->intersect(shadowRay, shadowHit, 1e-4f);
                    float currentDistToLight = (realLightPos - shadowRayOrigin).length() - 1e-3f;
                    
                    if (!hasHit || shadowHit.getT() >= currentDistToLight) break;
                    if (shadowHit.getMaterial()->getEmission().length() > 1e-6f) break;
                    
                    float hitRefractiveIndex = shadowHit.getMaterial()->getRefractiveIndex();
                    if (std::abs(hitRefractiveIndex - 1.0f) > 1e-6) {
                        shadowAttenuation = shadowAttenuation * shadowHit.getMaterial()->getDiffuseColor();
                        shadowRayOrigin = shadowRay.pointAtParameter(shadowHit.getT() + 5e-3f);
                    } else {
                        shadowAttenuation = Vector3f::ZERO; break;
                    }
                }

                if (shadowAttenuation.length() > 1e-5f) {
                    Vector3f fr_diffuse = m->getDiffuseColor(safe_u, safe_v) / M_PI;
                    Vector3f H = (L_dir + V).normalized();
                    float cosHN = std::max(0.0f, Vector3f::dot(H, N));
                    float cosHV = std::max(0.0f, Vector3f::dot(H, V));
                    float denomD = (cosHN * cosHN * (alpha2 - 1.0f) + 1.0f);
                    float D_val = alpha2 / (M_PI * denomD * denomD + 1e-5f);
                    
                    Vector3f F0 = m->getSpecularColor().length() > 1e-5 ? m->getSpecularColor() : Vector3f(0.04f);
                    Vector3f F = F0 + (Vector3f(1.0f) - F0) * powf(1.0f - cosHV, 5.0f);
                    float k = (roughness + 1.0f) * (roughness + 1.0f) / 8.0f;
                    float g1 = cosViewN / (cosViewN * (1.0f - k) + k);
                    float g2 = cosThetaP / (cosThetaP * (1.0f - k) + k);
                    Vector3f fr_specular = (D_val * F * g1 * g2) / (4.0f * cosThetaP * cosViewN + 1e-4f);
                    
                    Vector3f brdf_val = fr_diffuse + fr_specular;

                    float p_diffuse = m->getDiffuseColor().length() / (m->getDiffuseColor().length() + m->getSpecularColor().length() + 1e-5f);
                    float p_specular = 1.0f - p_diffuse;
                    float pdf_diffuse_eval = cosThetaP / M_PI;
                    float pdf_specular_eval = pdfGGX(N, H, V, roughness);
                    float pdf_brdf = pdf_diffuse_eval * p_diffuse + pdf_specular_eval * p_specular;

                    float pdf_light = 0.0f;
                    AreaLight* areaLight = dynamic_cast<AreaLight*>(light);
                    if (areaLight != nullptr) {
                        Vector3f p00 = areaLight->getSamplePoint(0,0);
                        float area = Vector3f::cross(areaLight->getSamplePoint(1,0)-p00, areaLight->getSamplePoint(0,1)-p00).length();
                        Vector3f lightNormal = Vector3f::cross(areaLight->getSamplePoint(1,0)-p00, areaLight->getSamplePoint(0,1)-p00).normalized();
                        float cosThetaLight = std::abs(Vector3f::dot(lightNormal, -L_dir));
                        float safe_dist2 = std::max(dist * dist, 1e-3f);
                        pdf_light = (cosThetaLight > 1e-5f) ? (safe_dist2 / (area * cosThetaLight)) : 0.0f;
                    }

                    float mis_weight = 1.0f;
                    if (pdf_light + pdf_brdf > 1e-6f && areaLight != nullptr) {
                        mis_weight = pdf_light / (pdf_light + pdf_brdf);
                    }

                    // 🌟 修复：去除直接光的内层硬钳制（Clamp）
                    Vector3f singleSampleContrib = radianceFactor * shadowAttenuation * brdf_val * cosThetaP * mis_weight;
                    L_direct += singleSampleContrib;
                }
            }
        }
    }
    
    if (!usePathTracing) {
        return L_direct * rrScale;
    } 
    
    Vector3f L_indirect = Vector3f::ZERO;
    float u1 = dis(gen); float u2 = dis(gen); float selectType = dis(gen); 
    float p_diffuse = m->getDiffuseColor().length() / (m->getDiffuseColor().length() + m->getSpecularColor().length() + 1e-5f);
    float p_specular = 1.0f - p_diffuse;
    
    Vector3f nextDir; 
    if (selectType < p_diffuse) {
        nextDir = sampleHemisphere(N_diffuse); 
    } else {
        Vector3f H_sampled = sampleGGXNormal(N, roughness, u1, u2); 
        nextDir = (V - 2.0f * Vector3f::dot(V, H_sampled) * H_sampled).normalized();
    }

    Vector3f targetNormal = (selectType < p_diffuse) ? N_diffuse : N;
    float cosThetaNext = std::max(0.0f, Vector3f::dot(targetNormal, nextDir));
    float cosViewN_s = std::max(0.0f, Vector3f::dot(targetNormal, V));
    
    if (cosThetaNext > 0.0f && cosViewN_s > 0.0f) {
        float pdf_diffuse_eval = cosThetaNext / M_PI;
        Vector3f H_eval = (nextDir + V).normalized();
        float pdf_specular_eval = pdfGGX(N, H_eval, V, roughness);
        float pdf_val = pdf_diffuse_eval * p_diffuse + pdf_specular_eval * p_specular;
        
        if (pdf_val > 1e-5f) {
            Vector3f brdf_val = Vector3f::ZERO;
            if (selectType < p_diffuse) {
                brdf_val = m->getDiffuseColor(safe_u, safe_v) / M_PI;
            } else {
                float cosHN_s = std::max(0.0f, Vector3f::dot(H_eval, N));
                float cosHV_s = std::max(0.0f, Vector3f::dot(H_eval, V));
                float denomD = (cosHN_s * cosHN_s * (alpha2 - 1.0f) + 1.0f);
                float D_val = alpha2 / (M_PI * denomD * denomD);
                Vector3f F0 = m->getSpecularColor().length() > 1e-5 ? m->getSpecularColor() : Vector3f(0.04f);
                Vector3f F = F0 + (Vector3f(1.0f) - F0) * powf(1.0f - cosHV_s, 5.0f);
                float k = (roughness + 1.0f) * (roughness + 1.0f) / 8.0f;
                float g1 = cosViewN_s / (cosViewN_s * (1.0f - k) + k);
                float g2 = cosThetaNext / (cosThetaNext * (1.0f - k) + k);
                brdf_val = (D_val * F * g1 * g2) / (4.0f * cosViewN_s * cosThetaNext + 1e-4f);
            }

            Ray nextRay(p + targetNormal * 1e-3f, nextDir);
            Hit nextHit;
            bool isNextIntersect = baseGroup->intersect(nextRay, nextHit, 1e-3f);
            
            if (isNextIntersect) {
                Material* nextMat = nextHit.getMaterial();
                Vector3f nextEmission = nextMat->getEmission();
                Vector3f indirect_radiance = traceRay(nextRay, 1e-3f, depth + 1, maxDepth, sceneParser, false, true);
                
                if (nextEmission.length() > 1e-6f) {
                    float pdf_light = 0.0f;
                    for (int l_idx = 0; l_idx < sceneParser.getNumLights(); ++l_idx) {
                        AreaLight* areaLight = dynamic_cast<AreaLight*>(sceneParser.getLight(l_idx));
                        if (areaLight != nullptr) {
                            Vector3f p00 = areaLight->getSamplePoint(0,0);
                            Vector3f d1 = areaLight->getSamplePoint(1,0) - p00;
                            Vector3f d2 = areaLight->getSamplePoint(0,1) - p00;
                            Vector3f hitP = nextRay.pointAtParameter(nextHit.getT());
                            
                            float u_l = Vector3f::dot(hitP - p00, d1) / d1.squaredLength();
                            float v_l = Vector3f::dot(hitP - p00, d2) / d2.squaredLength();
                            
                            if (u_l >= 0.0f && u_l <= 1.0f && v_l >= 0.0f && v_l <= 1.0f) {
                                Vector3f hitN_light = nextHit.getNormal().normalized();
                                float cosAlpha = std::max(0.0f, Vector3f::dot(-nextDir, hitN_light));
                                float lightDist2 = std::max(nextHit.getT() * nextHit.getT(), 1e-3f); 
                                float lightArea = d1.length() * d2.length();
                                pdf_light = (cosAlpha > 1e-5f) ? (lightDist2 / (lightArea * cosAlpha)) : 0.0f;
                                break;
                            }
                        }
                    }
                    float mis_weight = 1.0f;
                    if (pdf_val + pdf_light > 1e-6f && pdf_light > 0.0f) {
                        mis_weight = pdf_val / (pdf_val + pdf_light);
                    }
                    L_indirect = (brdf_val * indirect_radiance * cosThetaNext * mis_weight) / pdf_val;
                } else {
                    L_indirect = (brdf_val * indirect_radiance * cosThetaNext) / pdf_val;
                }
            }
        }
    }
    // 🌟 修复：去除间接光硬钳制（Clamp），改在最外层统一做拟合范围安全限幅
    Vector3f L_ambient = m->getAmbientColor() * 0.15f;
    // 【修复四】rrScale 作用于整个返回值（L_direct + L_indirect + L_ambient）。
    // 路径到达 depth > 0 本身是以概率 P_RR 存活的随机事件，该 depth 的所有出射能量
    // 均需乘 1/P_RR 补偿。depth=0 时 rrScale=1.0，与之前完全等价，无额外影响。
    return (L_direct + L_indirect + L_ambient) * rrScale;
}
// 🌟 严格对齐论文 "First Hit Only / Diffuse Component Only" 规范的亮度追踪器
float traceLuma(const Ray &ray, float tmin, int depth, int maxDepth, SceneParser &sceneParser)
{
    Hit hit; 
    Group *baseGroup = sceneParser.getGroup();
    if (!baseGroup->intersect(ray, hit, tmin)) {
        return 0.0f; // 没击中物体，没有漫反射亮度
    }
    if (depth >= maxDepth || depth >= 10) return 0.0f;

    Material *m = hit.getMaterial();
    Vector3f N = hit.getNormal().normalized(); 
    Vector3f D = ray.getDirection().normalized(); 
    Vector3f V = -D; 
    Vector3f p = ray.pointAtParameter(hit.getT());

    // 轮盘赌
    thread_local std::random_device rd; thread_local std::mt19937 gen(rd());
    thread_local std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    float P_RR = 0.8f; float rrScale = 1.0f;
    if (depth > 0) {
        if (dis(gen) > P_RR) return 0.0f;
        rrScale = 1.0f / P_RR;
    }

    Vector3f reflectiveColor = m->getReflectiveColor();
    float refractiveIndex = m->getRefractiveIndex();
    bool isReflective = (reflectiveColor.length() > 1e-6);
    bool isRefractive = (std::abs(refractiveIndex - 1.0f) > 1e-6);

    // 🔴 理想折射/反射光路：属于纯 Specular 传输，不计入当前面片的漫反射风格化感知亮度
    if (isRefractive) {
        float cos_i = std::abs(Vector3f::dot(D, N)); 
        float R0 = powf((refractiveIndex - 1.0f) / (refractiveIndex + 1.0f), 2.0f);
        float Fr = R0 + (1.0f - R0) * powf(1.0f - cos_i, 5.0f);
        
        if (dis(gen) < Fr) { 
            Vector3f R = (D - 2.0f * Vector3f::dot(D, N) * N).normalized();
            return traceLuma(Ray(p + R * 5e-3f, R), 1e-4f, depth + 1, maxDepth, sceneParser) * rrScale;
        } else { 
            Vector3f T = (D).normalized(); // 简化透明穿透
            return traceLuma(Ray(p + T * 5e-3f, T), 1e-4f, depth + 1, maxDepth, sceneParser) * rrScale;
        }
    }
    else if (isReflective) {
        Vector3f R = (D - 2.0f * Vector3f::dot(D, N) * N).normalized();
        return traceLuma(Ray(p + R * 5e-3f, R), 1e-4f, depth + 1, maxDepth, sceneParser) * rrScale;
    }

    // 🟢 真正的漫反射/Glossy面片：计算它的直接光照和一次间接散射亮度
    float L_diffuse_direct = 0.0f;
    Vector2f final_uv = hit.getUV();
    float safe_u = fmod(final_uv.x(), 1.0f); if (safe_u < 0.0f) safe_u += 1.0f;
    float safe_v = fmod(final_uv.y(), 1.0f); if (safe_v < 0.0f) safe_v += 1.0f;

    // 直接光 NEE 贡献亮度
    for (int i = 0; i < sceneParser.getNumLights(); ++i) {
        Light* light = sceneParser.getLight(i);
        Vector3f L_dir; Vector3f radianceFactor; float dist;
        if (light->sampleNEE(p, N, dis(gen), dis(gen), L_dir, radianceFactor, dist)) {
            float cosThetaP = std::max(0.0f, Vector3f::dot(N, L_dir));
            if (cosThetaP > 0.0f) {
                // 快速阴影筛查
                Ray shadowRay(p + N * 5e-3f, L_dir); Hit shadowHit;
                if (!baseGroup->intersect(shadowRay, shadowHit, 1e-4f) || shadowHit.getT() >= dist - 1e-3f) {
                    Vector3f brdf = evaluateBRDF(N, V, L_dir, m, safe_u, safe_v);
                    Vector3f contrib = radianceFactor * brdf * cosThetaP;
                    L_diffuse_direct += (contrib.x() + contrib.y() + contrib.z()) / 3.0f;
                }
            }
        }
    }

    // 间接光漫反射弹射亮度
    float L_diffuse_indirect = 0.0f;
    Vector3f nextDir = sampleHemisphere(N);
    float cosThetaNext = std::max(0.0f, Vector3f::dot(N, nextDir));
    if (cosThetaNext > 0.0f) {
        Vector3f brdf = m->getDiffuseColor(safe_u, safe_v) / M_PI;
        float pdf = cosThetaNext / M_PI;
        float nextLuma = traceLuma(Ray(p + N * 1e-3f, nextDir), 1e-4f, depth + 1, maxDepth, sceneParser);
        L_diffuse_indirect = (nextLuma * (brdf.x() + brdf.y() + brdf.z()) / 3.0f * cosThetaNext) / (pdf + 1e-5f);
    }

    return L_diffuse_direct + L_diffuse_indirect * rrScale + (m->getAmbientColor().length() * 0.05f);
}
int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        cout << "Usage: ./bin/PA1 <input scene file> <output bmp file>" << endl;
        return 1;
    }
    string outputFile = argv[2];
    
    SceneParser sceneParser = SceneParser(argv[1]);
    Camera *camera = sceneParser.getCamera();
    Image renderedImg(camera->getWidth(), camera->getHeight());

    int ssaaSide = 20; 
    int totalSamples = ssaaSide * ssaaSide; // 100个光线样本

    int K_degree = 8;
    std::vector<float> cheby_alpha;
    computeCelShadingCoefficients(K_degree, cheby_alpha);

    // 🌟 切比雪夫多项式幂次展开矩阵 (严格对应物理定义域 [-1, 4])
    float coeff[9][9] = {
        {1, 0, 0, 0, 0, 0, 0, 0, 0},              // T0
        {0, 1, 0, 0, 0, 0, 0, 0, 0},              // T1
        {-1, 0, 2, 0, 0, 0, 0, 0, 0},             // T2
        {0, -3, 0, 4, 0, 0, 0, 0, 0},             // T3
        {1, 0, -8, 0, 8, 0, 0, 0, 0},             // T4
        {0, 5, 0, -20, 0, 16, 0, 0, 0},           // T5
        {-1, 0, 18, 0, -48, 0, 32, 0, 0},         // T6
        {0, -7, 0, 56, 0, -112, 0, 64, 0},        // T7
        {1, 0, -32, 0, 160, 0, -256, 0, 128}      // T8
    };

    bool useStylizedRendering = true; 

    #pragma omp parallel for schedule(dynamic, 1)
    for (int x = 0; x < camera->getWidth(); ++x)
    {
        for (int y = 0; y < camera->getHeight(); ++y)
        {
            Vector3f rawPixelColor = Vector3f::ZERO;
            float total_weight = 0.0f;
            
            // 【修复一+三】简单幂次矩账本（论文 §4 要求 E[L^k] 的无偏估计）
            // power_sums[0]: 非高光有效样本计数
            // power_sums[k] = Σ t_i^k  (k ≥ 1)，除以 power_sums[0] 即得 E[L^k] 无偏估计
            // 旧代码使用 Girard-Newton 初等对称多项式 e_k/C(N,k)，
            // 其期望值为 (E[L])^k 而非 E[L^k]，实质退化为 m(mean)，丢失了分布信息
            std::vector<double> power_sums(K_degree + 1, 0.0);

            thread_local std::random_device rd;
            thread_local std::mt19937 gen(rd());
            thread_local std::uniform_real_distribution<float> dis(0.0f, 1.0f);
            
            for (int sx = 0; sx < ssaaSide; ++sx) {
                for (int sy = 0; sy < ssaaSide; ++sy) {
                    float rx = 0.25f + dis(gen) * 0.5f;
                    float ry = 0.25f + dis(gen) * 0.5f;
                    float subX = x + (sx + rx) / (float)ssaaSide;
                    float subY = y + (sy + ry) / (float)ssaaSide;

                    Ray camRay = camera->generateRay(Vector2f(subX, subY));
                    
                    // 🔍 先行测试：玻璃（indexOfRefraction 1.5）与完美金属立方体（Material 2）过滤保护
                    Hit testHit;
                    bool isIntersect = sceneParser.getGroup()->intersect(camRay, testHit, 1e-4f);
                    bool isPureSpecular光路 = false;
                    if (isIntersect && testHit.getMaterial() != nullptr) {
                        Material* tm = testHit.getMaterial();
                        if (tm->getRefractiveIndex() > 1.01f || tm->getReflectiveColor().length() > 1e-3f || tm->getTransparency() > 0.9f) {
                            isPureSpecular光路 = true;
                        }
                    }

                    Vector3f rad = traceRay(camRay, 1e-4f, 0, 6, sceneParser, true, true);
                    
                    rawPixelColor += rad;
                    total_weight += 1.0f;

                    // 【修复三】高光/折射/纯反射像素不参与风格化矩统计
                    // 它们最终在像素后处理阶段直接输出 mean_color，
                    // 若混入 power_sums 会以错误的 t_val 污染非高光像素的矩估计
                    if (!isPureSpecular光路) {
                        // 【修复二】使用 traceLuma 获取低方差漫反射亮度，而非从 traceRay 的
                        // 全路径结果提取均值。traceLuma 只追踪第一次漫反射弹射（First Hit /
                        // Diffuse Only），方差远低于含高光、多次弹射的全路径，使矩估计更稳定。
                        // 这符合论文对亮度估计量的 “低方差” 要求。
                        float raw_luma = traceLuma(camRay, 1e-4f, 0, 4, sceneParser);

                        // exposure_scale 控制明暗分界线在原始亮度空间的位置：
                        // 分界线对应 target_style_m 的 u=0.6，即 t_val=0.2，
                        // 故 raw_luma * scale = 0.6 → 分界线在 raw_luma = 0.6/scale = 0.3
                        float exposure_scale = 2.0f;
                        float sample_luma = raw_luma * exposure_scale;

                        // 映射到 Chebyshev 多项式定义域 [-1, 1]
                        float t_val = 2.0f * sample_luma - 1.0f;
                        t_val = std::max(-1.0f, std::min(1.0f, t_val));

                        // 【修复一】简单幂次矩累积：power_sums[k] += t^k
                        // 论文 §4：E[T_i(L)] = Σ_j c_{ij} * E[L^j]，需要 E[L^k] 的无偏估计
                        // 正确估计：(1/N) * Σ t_i^k → E[L^k]
                        double t_pow = 1.0;
                        for (int k = 1; k <= K_degree; ++k) {
                            t_pow *= (double)t_val;
                            power_sums[k] += t_pow;
                        }
                        power_sums[0] += 1.0; // 有效（非高光）样本计数
                    }
                }
            }

            Vector3f finalPixelColor = Vector3f::ZERO;
            Vector3f mean_color = rawPixelColor / total_weight;

            if (!useStylizedRendering) {
                finalPixelColor = mean_color;
            } 
            else {
                // 从 power_sums 恢复 E[L^k] 的无偏估计：E[L^k] ≈ power_sums[k] / N_valid
                // N_valid = 参与矩统计的非高光样本数（power_sums[0]）
                std::vector<float> expected_powers(K_degree + 1, 0.0f);
                expected_powers[0] = 1.0f; // E[L^0] = 1（恒成立）
                double n_valid = power_sums[0];
                if (n_valid > 0.0) {
                    for (int k = 1; k <= K_degree; ++k) {
                        // (1/N_valid) * Σ t_i^k 是 E[L^k] 的无偏估计
                        expected_powers[k] = (float)(power_sums[k] / n_valid);
                    }
                }

                // 普通幂次变 Chebyshev 基底期望
                std::vector<float> E_T(K_degree + 1, 0.0f);
                for (int i = 0; i <= K_degree; ++i) {
                    float sum_t = 0.0f;
                    for (int k = 0; k <= i; ++k) {
                        sum_t += coeff[i][k] * expected_powers[k];
                    }
                    E_T[i] = sum_t;
                }

                float stylized_luma_expectation = 0.0f;
                for (int i = 0; i <= K_degree; ++i) {
                    stylized_luma_expectation += cheby_alpha[i] * E_T[i];
                }
                
                // 🌟 放开波动截断范围至 4.0，给场景灯光的亮部留足完美的能量容纳空间
                stylized_luma_expectation = std::max(0.1f, std::min(0.85f, stylized_luma_expectation));

                float avg_luma = (mean_color.x() + mean_color.y() + mean_color.z()) / 3.0f;

                // 🌟 严格执行论文式(11)进行线性空间辐射度重塑
                if (avg_luma > 1e-4f) {
                    finalPixelColor = mean_color * (stylized_luma_expectation / avg_luma);
                } else {
                    finalPixelColor = Vector3f(stylized_luma_expectation); 
                }
                
                // 后端玻璃/反射材质放行保障
                Hit finalPixelTestHit;
                Ray pixelCenterRay = camera->generateRay(Vector2f(x + 0.5f, y + 0.5f));
                if (sceneParser.getGroup()->intersect(pixelCenterRay, finalPixelTestHit, 1e-4f)) {
                    Material* m_check = finalPixelTestHit.getMaterial();
                    if (m_check && (m_check->getRefractiveIndex() > 1.01f || m_check->getReflectiveColor().length() > 1e-3f || m_check->getTransparency() > 0.9f)) {
                        finalPixelColor = mean_color; 
                    }
                }
            }

            // 色彩空间最末端硬件 Gamma 2.2 输出校正
            finalPixelColor = Vector3f(
                powf(std::max(0.0f, finalPixelColor.x()), 1.0f / 2.2f),
                powf(std::max(0.0f, finalPixelColor.y()), 1.0f / 2.2f),
                powf(std::max(0.0f, finalPixelColor.z()), 1.0f / 2.2f)
            );

            finalPixelColor = Vector3f(
                std::min(1.0f, finalPixelColor.x()),
                std::min(1.0f, finalPixelColor.y()),
                std::min(1.0f, finalPixelColor.z())
            );
            renderedImg.SetPixel(x, y, finalPixelColor);
        }
    }
    renderedImg.SaveImage(outputFile.c_str());
    return 0;
}
// if (x % 40 == 0) {
//     printf("Rendering Progress: %.1f%%\n", (float)x / camera->getWidth() * 100.0f);
// }