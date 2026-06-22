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

Vector3f traceRay(const Ray &ray, float tmin, int depth, int maxDepth, SceneParser &sceneParser, bool isSpecular = true, bool usePathTracing = true)
{
    Hit hit; 
    Group *baseGroup = sceneParser.getGroup();
    Ray currentRay = ray;
    float currentTmin = tmin;
    bool isIntersect = false;

    // 💡 1. 健壮穿透求交循环：完美过滤半透明/空白外壳
    while (true) {
        hit = Hit(); 
        isIntersect = baseGroup->intersect(currentRay, hit, currentTmin);
        if (!isIntersect) break; 

        Material *m = hit.getMaterial();
        Vector2f raw_uv = hit.getUV();

        // 内部防越界安全卡位测试
        float su = fmod(raw_uv.x(), 1.0f); if (su < 0.0f) su += 1.0f;
        float sv = fmod(raw_uv.y(), 1.0f); if (sv < 0.0f) sv += 1.0f;
        

        Vector3f texColor = m->getDiffuseColor(su, sv);

        // 透明判定守卫（Alpha Cutout）：如果材质全透，或者贴图踩中纯黑/纯白纯留白
        bool isTransparentPatch = (m->getTransparency() > 0.8f) || 
                                  (texColor.length() < 1e-3f) || 
                                  ((Vector3f(1.0f) - texColor).length() < 1e-3f);

        if (isTransparentPatch) {
            currentTmin = 5e-3f;
            currentRay = Ray(currentRay.pointAtParameter(hit.getT()), currentRay.getDirection());
            continue; 
        }
        break; 
    }

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
    
    // ===================================================================
    // 💡 核心修复：对最终传递给整个 PBR 管线的 UV 坐标进行强制安全防护绑定
    // ===================================================================
    Vector2f final_uv = hit.getUV();
    float safe_u = fmod(final_uv.x(), 1.0f); if (safe_u < 0.0f) safe_u += 1.0f;
    float safe_v = fmod(final_uv.y(), 1.0f); if (safe_v < 0.0f) safe_v += 1.0f;
    Vector3f emission = m->getEmission();
    Vector3f N_diffuse = N;
    if (Vector3f::dot(D, N_diffuse) > 0.0f) {
        N_diffuse = -N_diffuse; // 只在此变量上应用双面迎向翻转，使双面都能进行漫反射和自发光染色
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

        // 💡 升级：传入全安全的归一化循环 UV 坐标
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
                        Vector3f singleSpecularContrib = radianceFactor * shadowAttenuation * powf(specAngle, 50.0f) * Fr;
                        float maxClamp = 2.5f; 
                        singleSpecularContrib = Vector3f(
                            std::min(maxClamp, singleSpecularContrib.x()),
                            std::min(maxClamp, singleSpecularContrib.y()),
                            std::min(maxClamp, singleSpecularContrib.z())
                        );
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
            return (L_specular_nee * reflectiveColor + indirect * reflectiveColor) * rrScale;
        } else { 
            Vector3f T = (ratio * D + (ratio * cos_i - cos_t) * Ng).normalized();
            Vector3f indirect = traceRay(Ray(p + T * 5e-3f, T), 1e-4f, depth + 1, maxDepth, sceneParser, true, usePathTracing);
            Vector3f attenuation = transmissionColor * m->getTransparency();
            return (indirect * attenuation) * rrScale;
        }
    }
    else if (isReflective) {
        Vector3f R = (D - 2.0f * Vector3f::dot(D, N) * N).normalized();
        Vector3f offsetP = p + R * 5e-3f;
        Vector3f indirect = traceRay(Ray(offsetP, R), 1e-4f, depth + 1, maxDepth, sceneParser, true, usePathTracing);
        return indirect * reflectiveColor * rrScale;
    }

    // ===================================================================
    // 3. 漫反射 / 粗糙微表面（Glossy）着色计算
    // ===================================================================
    Vector3f L_direct = Vector3f::ZERO;
    float roughness = new_clamp(1.0f - (m->getShininess() / 100.0f), 0.05f, 1.0f);
    float alpha = roughness * roughness;
    float alpha2 = alpha * alpha;
    
    int lightSamplesX = 4;
    int lightSamplesY = 4;
    float totalLightSamples = (float)(lightSamplesX * lightSamplesY);

    for (int i = 0; i < sceneParser.getNumLights(); ++i) {
        Light* light = sceneParser.getLight(i);
        Vector3f L_direct_accum = Vector3f::ZERO;

        for (int lx = 0; lx < lightSamplesX; ++lx) {
            for (int ly = 0; ly < lightSamplesY; ++ly) {
                float u_s = ((float)lx + dis(gen)) / (float)lightSamplesX;
                float v_s = ((float)ly + dis(gen)) / (float)lightSamplesY;
                
                Vector3f L_dir; Vector3f radianceFactor; float dist;
                
                if (light->sampleNEE(p, N, u_s, v_s, L_dir, radianceFactor, dist)) {
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
                            }
                            else {
                                shadowAttenuation = Vector3f::ZERO; break;
                            }
                        }

                        if (shadowAttenuation.length() > 1e-5f) {
                            // 💡 升级：直接光照漫反射采集使用全安全归一化坐标 safe_u, safe_v
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
                            float G = g1 * g2;
                            Vector3f fr_specular = (D_val * F * G) / (4.0f * cosThetaP * cosViewN + 1e-4f);
                            
                            Vector3f singleSampleContrib = radianceFactor * shadowAttenuation * (fr_diffuse + fr_specular) * cosThetaP;
                            float maxClamp = 2.5f;
                            singleSampleContrib = Vector3f(
                                std::min(maxClamp, singleSampleContrib.x()),
                                std::min(maxClamp, singleSampleContrib.y()),
                                std::min(maxClamp, singleSampleContrib.z())
                            );
                            L_direct_accum += singleSampleContrib;
                        }
                    }
                }
            }
        }
        L_direct += L_direct_accum / totalLightSamples; 
    }
    
    if (!usePathTracing) {
        return L_direct * rrScale;
    } 
    else { 
        float u1 = dis(gen); float u2 = dis(gen); float selectType = dis(gen); 
        
        float p_diffuse = m->getDiffuseColor().length() / (m->getDiffuseColor().length() + m->getSpecularColor().length() + 1e-5f);
        float p_specular = 1.0f - p_diffuse;
        
        Vector3f nextDir; 
        float pdf_val = 0.0f;
        Vector3f brdf_val = Vector3f::ZERO;

        if (selectType < p_diffuse) {
            nextDir = sampleHemisphere(N_diffuse); 
        } 
        else {
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
            
            pdf_val = pdf_diffuse_eval * p_diffuse + pdf_specular_eval * p_specular;
            if (selectType < p_diffuse) {
                // 💡 升级：间接漫反射反射分子采集同样使用全安全坐标 safe_u, safe_v
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
        }

        Vector3f L_indirect = Vector3f::ZERO;
        if (pdf_val > 1e-5f && cosThetaNext > 0.0f) {
            Ray nextRay(p + targetNormal * 1e-3f, nextDir);
            Hit nextHit;
            bool isNextIntersect = baseGroup->intersect(nextRay, nextHit, 1e-3f);
            
            if (isNextIntersect) {
                Material* nextMat = nextHit.getMaterial();
                Vector3f nextEmission = nextMat->getEmission();
                
                Vector3f indirect_radiance = traceRay(nextRay, 1e-3f, depth + 1, maxDepth, sceneParser, false, true);
                
                if (nextEmission.length() > 1e-6f && sceneParser.getNumLights() > 0) {
                    AreaLight* areaLight = dynamic_cast<AreaLight*>(sceneParser.getLight(0));
                    float mis_weight = 1.0f;
                    
                    if (areaLight != nullptr) {
                        Vector3f hitN_light = nextHit.getNormal().normalized();
                        float cosAlpha = std::max(0.0f, Vector3f::dot(-nextDir, hitN_light));
                        float lightDist2 = nextHit.getT() * nextHit.getT();
                        
                        Vector3f p00 = areaLight->getSamplePoint(0,0);
                        float lightArea = (areaLight->getSamplePoint(1,0) - p00).length() * (areaLight->getSamplePoint(0,1) - p00).length();
                        float pdf_light = (cosAlpha > 1e-5f) ? (lightDist2 / (lightArea * cosAlpha)) : 0.0f;
                        
                        if (pdf_val + pdf_light > 1e-6f) {
                            mis_weight = pdf_val / (pdf_val + pdf_light);
                        }
                    }
                    L_indirect = (brdf_val * indirect_radiance * cosThetaNext * mis_weight) / pdf_val;
                } else {
                    L_indirect = (brdf_val * indirect_radiance * cosThetaNext) / pdf_val;
                }
            }
        }
        Vector3f L_ambient = m->getAmbientColor() * 0.15f; 
        // 联合结算：物理光路辐射度 + 动漫环境固有底色
        return (L_direct + L_indirect) * rrScale + L_ambient;
    }
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

    int ssaaSide = 10; 
    int totalSamples = ssaaSide * ssaaSide;

    // 💡 核心优化 2：引入 OpenMP 动态多线程调度加速！将所有 CPU 核心的算力彻底释放
    #pragma omp parallel for schedule(dynamic, 1)
    for (int x = 0; x < camera->getWidth(); ++x)
    {
        for (int y = 0; y < camera->getHeight(); ++y)
        {
            Vector3f pixelColor = Vector3f::ZERO;
            
            // 💡 为了多线程安全，主循环的随机数也升级为 thread_local
            thread_local std::random_device rd;
            thread_local std::mt19937 gen(rd());
            thread_local std::uniform_real_distribution<float> dis(0.0f, 1.0f);

            for (int sx = 0; sx < ssaaSide; ++sx) {
                for (int sy = 0; sy < ssaaSide; ++sy) {
                    float rx = dis(gen);
                    float ry = dis(gen);
                    float subX = x + (sx + rx) / (float)ssaaSide;
                    float subY = y + (sy + ry) / (float)ssaaSide;
                    
                    Ray camRay = camera->generateRay(Vector2f(subX, subY));
                    // 💡 maxDepth 显式传入 6 层
                    pixelColor += traceRay(camRay, 1e-4f, 0, 6, sceneParser,true,true);
                }
            }
            
            Vector3f finalPixelColor = pixelColor / (float)totalSamples;
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
        // 打印单线程进度条提示（可选）
        if (x % 40 == 0) {
            printf("Rendering Progress: %.1f%%\n", (float)x / camera->getWidth() * 100.0f);
        }
    }

    renderedImg.SaveImage(outputFile.c_str());
    cout << "Hello! Computer Graphics! Path Tracer Done!" << endl;
    return 0;
}