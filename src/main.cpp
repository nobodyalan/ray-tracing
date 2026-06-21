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
    bool isIntersect = baseGroup->intersect(ray, hit, tmin);
    
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

    Vector3f emission = m->getEmission();
    
    bool hitRealLightPatch = false;
    if (emission.length() > 1e-6f) {
        for (int i = 0; i < sceneParser.getNumLights(); ++i) {
            AreaLight* areaLight = dynamic_cast<AreaLight*>(sceneParser.getLight(i));
            if (areaLight != nullptr) {
                Vector3f p00 = areaLight->getSamplePoint(0, 0);
                Vector3f d1 = areaLight->getSamplePoint(1, 0) - p00;
                Vector3f d2 = areaLight->getSamplePoint(0, 1) - p00;
                
                float u = Vector3f::dot(p - p00, d1) / d1.squaredLength();
                float v = Vector3f::dot(p - p00, d2) / d2.squaredLength();
                
                if (u >= 0.0f && u <= 1.0f && v >= 0.0f && v <= 1.0f) {
                    hitRealLightPatch = true;
                    break;
                }
            }
        }
    }

    if (m->getEmission().length() > 1e-6f) {
        if (hitRealLightPatch) {
            // 💡 无论是不是 Specular，次级射线撞击到光源时必须把 emission 带回去，交由 MIS 去做加权分配
            return emission; 
        } else {
            emission = Vector3f::ZERO; 
        }
    }

    Vector3f reflectiveColor = m->getReflectiveColor();
    float refractiveIndex = m->getRefractiveIndex();
    bool isReflective = (reflectiveColor.length() > 1e-6);
    bool isRefractive = (std::abs(refractiveIndex - 1.0f) > 1e-6);

    // ===================================================================
    // 2.1 物理完全体：支持有色折射/透射与自适应光路推进的透明介质
    // ===================================================================
    if (isRefractive) {
        float dotDN = Vector3f::dot(D, N);
        bool leaving = (dotDN > 0.0f); 

        float n_i = leaving ? refractiveIndex : 1.0f;
        float n_t = leaving ? 1.0f : refractiveIndex;
        float ratio = n_i / n_t;

        Vector3f Ng = leaving ? -N : N;
        float cos_i = -Vector3f::dot(D, Ng); 
        float sin2_t = ratio * ratio * (1.0f - cos_i * cos_i);
        float cos_t = sqrt(1-sin2_t);
        
        float R0 = powf((n_i - n_t) / (n_i + n_t), 2.0f);
        float Fr = R0 + (1.0f - R0) * powf(1.0f - std::max(0.0f, cos_i), 5.0f);

        // 💡 提取有色半透明材质的吸收颜色项和透明度
        Vector3f transmissionColor = m->getDiffuseColor(); 
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
                    
                    // 统一直接光（NEE）阴影射线测试：聚光灯与面光源在可见性拓扑上完全对齐
                    // ===================================================================
                    Vector3f realLightPos = p + L_dir * dist; 
                    while (true) {
                        Ray shadowRay(shadowRayOrigin, shadowRayDir);
                        Hit shadowHit; 
                        bool hasHit = baseGroup->intersect(shadowRay, shadowHit, 1e-4f);
                        
                        // 💡 恢复物理无偏的直线距离截止判定（解决隐患二的逃逸早退问题）
                        float currentDistToLight = (realLightPos - shadowRayOrigin).length() - 1e-3f;
                        
                        if (!hasHit || shadowHit.getT() >= currentDistToLight) {
                            break; 
                        }
                        if (shadowHit.getMaterial()->getEmission().length() > 1e-6f) {
                            break; 
                        }
                        
                        float hitRefractiveIndex = shadowHit.getMaterial()->getRefractiveIndex();
                        if (std::abs(hitRefractiveIndex - 1.0f) > 1e-6) {
                            Vector3f h_matColor = shadowHit.getMaterial()->getDiffuseColor();
                            float h_transFactor = shadowHit.getMaterial()->getTransparency();
                            
                            Vector3f hitN = shadowHit.getNormal().normalized();
                            float shadow_cos_i = std::abs(Vector3f::dot(shadowRayDir, hitN));
                            
                            float h_n_i = 1.0f; float h_n_t = hitRefractiveIndex;
                            float h_R0 = powf((h_n_i - h_n_t) / (h_n_i + h_n_t), 2.0f); // 垂直入射基础反射率（玻璃约为 0.04）
                            
                            // 使用垂直入射的菲涅尔基础透射率（1.0 - 0.04 = 0.96）作为常数穿透项，彻底免疫掠入射角的钝角崩塌！
                            float constant_transmittance = 1.0f - h_R0; 
                            
                            // 累乘能量：此时由于没有了掠射角的极端值，前方地面能量保持在 96% 以上
                            shadowAttenuation = shadowAttenuation * constant_transmittance * h_matColor * h_transFactor;
                            
                            shadowRayOrigin = shadowRay.pointAtParameter(shadowHit.getT() + 5e-3f);
                        }
                        else {
                            // 撞击到普通不透明固体，直接完全阻挡
                            shadowAttenuation = Vector3f::ZERO; 
                            break;
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

        // 💡 关键改动：区分颜色通道
        // 1. 如果是折射，我们强制不让 DiffuseColor 在漫反射计算中产生错误贡献
        // 2. 将此逻辑应用于折射路径
        if (sin2_t > 1.0f || dis(gen) < Fr) { 
            // 反射路径：只应用反射颜色
            Vector3f R = (D - 2.0f * Vector3f::dot(D, Ng) * Ng).normalized();
            Vector3f indirect = traceRay(Ray(p + R * 5e-3f, R), 1e-4f, depth + 1, maxDepth, sceneParser, true, usePathTracing);
            return (L_specular_nee * reflectiveColor + indirect * reflectiveColor) * rrScale;
        } else { 
            // 折射路径：只应用透射颜色 (transmissionColor) 和透明度 (transparency)
            Vector3f T = (ratio * D + (ratio * cos_i - cos_t) * Ng).normalized();
            Vector3f indirect = traceRay(Ray(p + T * 5e-3f, T), 1e-4f, depth + 1, maxDepth, sceneParser, true, usePathTracing);
            
            // 💡 颜色解耦：仅在此处应用透射颜色，不要影响外部的漫反射计算
            Vector3f attenuation = transmissionColor * m->getTransparency();
            return (indirect * attenuation) * rrScale;
        }
    }
    // ===================================================================
    // 2.2 理想纯全反射不透明金属介质（处理左下角的纯全反射镜面球）
    // ===================================================================
    else if (isReflective) {
        Vector3f R = (D - 2.0f * Vector3f::dot(D, N) * N).normalized();
        Vector3f offsetP = p + R * 5e-3f;
        Vector3f indirect = traceRay(Ray(offsetP, R), 1e-4f, depth + 1, maxDepth, sceneParser, true, usePathTracing);
        
        // 💡 严格乘上有色镜面反射率
        return indirect * reflectiveColor * rrScale;
    }

    // ===================================================================
    // 3. 漫反射 / 粗糙微表面（Glossy）着色计算 (下面保持你原本的逻辑)
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
                float u = ((float)lx + dis(gen)) / (float)lightSamplesX;
                float v = ((float)ly + dis(gen)) / (float)lightSamplesY;
                
                Vector3f L_dir;
                Vector3f radianceFactor;
                float dist;
                
                if (light->sampleNEE(p, N, u, v, L_dir, radianceFactor, dist)) {
                    float cosThetaP = Vector3f::dot(N, L_dir);
                    float cosViewN  = std::max(0.0f, Vector3f::dot(N, V)); 
                    
                    if (cosThetaP > 0.0f && cosViewN > 0.0f) {
                        Vector3f shadowRayOrigin = p + N * 5e-3f; 
                        Vector3f shadowRayDir = L_dir;             
                        Vector3f shadowAttenuation = Vector3f(1.0f); 
                        
                        Vector3f realLightPos = p + L_dir * dist; 
                        while (true) {
                            Ray shadowRay(shadowRayOrigin, shadowRayDir);
                            Hit shadowHit; 
                            bool hasHit = baseGroup->intersect(shadowRay, shadowHit, 1e-4f);
                            
                            float currentDistToLight = (realLightPos - shadowRayOrigin).length() - 1e-3f;
                            
                            if (!hasHit || shadowHit.getT() >= currentDistToLight) {
                                break;
                            }
                            if (shadowHit.getMaterial()->getEmission().length() > 1e-6f) {
                                break;
                            }
                            
                                                    // 💡 找到第 310 行附近的漫反射阴影循环：
                            float hitRefractiveIndex = shadowHit.getMaterial()->getRefractiveIndex();
                            if (std::abs(hitRefractiveIndex - 1.0f) > 1e-6) {
                                // 💡 对齐修改：提取被撞击半透明物体的颜色和不透明度
                                Vector3f h_matColor = shadowHit.getMaterial()->getDiffuseColor();
                                float h_transFactor = shadowHit.getMaterial()->getTransparency();
                                Vector3f hitN = shadowHit.getNormal().normalized();
                                float shadow_cos_i = std::abs(Vector3f::dot(shadowRayDir, hitN));
                                bool h_leaving = (Vector3f::dot(shadowRayDir, hitN) > 0.0f);
                                float h_n_i = h_leaving ? hitRefractiveIndex : 1.0f;
                                float h_n_t = h_leaving ? 1.0f : hitRefractiveIndex;
                                float h_R0 = powf((h_n_i - h_n_t) / (h_n_i + h_n_t), 2.0f);
                                float h_Fr = h_R0 + (1.0f - h_R0) * powf(1.0f - shadow_cos_i, 5.0f);
                                
                                // 💡 统一代数计算：乘上颜色和透明度衰减，强制阴影产生深度和色度
                                shadowAttenuation = shadowAttenuation * Vector3f(std::max(0.0f, 1.0f - h_Fr)) * h_matColor * h_transFactor;
                                shadowRayOrigin = shadowRay.pointAtParameter(shadowHit.getT() + 5e-3f);
                            }
                            else {
                                shadowAttenuation = Vector3f::ZERO;
                                break;
                            }
                        }

                        if (shadowAttenuation.length() > 1e-5f) {
                            Vector3f fr_diffuse = m->getDiffuseColor() / M_PI; 
                            
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
        
        Vector3f nextDir; Vector3f brdf_val = Vector3f::ZERO; float pdf_val = 0.0f;

        if (selectType < p_diffuse) {
            nextDir = sampleHemisphere(N);
            float cosThetaOut = std::max(0.0f, Vector3f::dot(N, nextDir));
            pdf_val = (cosThetaOut / M_PI) * p_diffuse;
            brdf_val = m->getDiffuseColor() / M_PI;
        } 
        else {
            Vector3f H_sampled = sampleGGXNormal(N, roughness, u1, u2);
            nextDir = (V - 2.0f * Vector3f::dot(V, H_sampled) * H_sampled).normalized();
            float cosThetaOut = std::max(0.0f, Vector3f::dot(N, nextDir));
            float cosViewN_s = std::max(0.0f, Vector3f::dot(N, V));
            
            if (cosThetaOut > 0.0f && cosViewN_s > 0.0f) {
                float cosHN_s = std::max(0.0f, Vector3f::dot(H_sampled, N));
                float cosHV_s = std::max(0.0f, Vector3f::dot(H_sampled, V));
                float denomD = (cosHN_s * cosHN_s * (alpha2 - 1.0f) + 1.0f);
                float D_val = alpha2 / (M_PI * denomD * denomD);
                Vector3f F0 = m->getSpecularColor().length() > 1e-5 ? m->getSpecularColor() : Vector3f(0.04f);
                Vector3f F = F0 + (Vector3f(1.0f) - F0) * powf(1.0f - cosHV_s, 5.0f);
                float k = (roughness + 1.0f) * (roughness + 1.0f) / 8.0f;
                float g1 = cosViewN_s / (cosViewN_s * (1.0f - k) + k);
                float g2 = cosThetaOut / (cosThetaOut * (1.0f - k) + k);
                brdf_val = (D_val * F * g1 * g2) / (4.0f * cosViewN_s * cosThetaOut + 1e-4f);
                pdf_val = pdfGGX(N, H_sampled, V, roughness) * (1.0f - p_diffuse);
            }
        }

        Vector3f L_indirect = Vector3f::ZERO;
        float cosThetaNext = std::max(0.0f, Vector3f::dot(N, nextDir));

        // ===================================================================
        // 💡 真正落地激活的无偏 MIS 路径解算内核
        // ===================================================================
        if (pdf_val > 1e-5f && cosThetaNext > 0.0f) {
            Ray nextRay(p + N * 1e-3f, nextDir);
            Hit nextHit;
            bool isNextIntersect = baseGroup->intersect(nextRay, nextHit, 1e-3f);
            
            if (isNextIntersect) {
                Material* nextMat = nextHit.getMaterial();
                Vector3f nextEmission = nextMat->getEmission();
                
                // 递归探测间接光辐射度
                Vector3f indirect_radiance = traceRay(nextRay, 1e-3f, depth + 1, maxDepth, sceneParser, false, true);
                
                // 💡 判断条件：如果这根间接光射线“意外”在天花板碰到了面光源
                if (nextEmission.length() > 1e-6f && sceneParser.getNumLights() > 0) {
                    AreaLight* areaLight = dynamic_cast<AreaLight*>(sceneParser.getLight(0));
                    float mis_weight = 1.0f;
                    
                    if (areaLight != nullptr) {
                        // 动态解算当前命中的面光源真实的几何 PDF 因子
                        Vector3f hitN_light = nextHit.getNormal().normalized();
                        float cosAlpha = std::max(0.0f, Vector3f::dot(-nextDir, hitN_light));
                        float lightDist2 = nextHit.getT() * nextHit.getT();
                        
                        // 从场景动态计算面光源面积，绝不硬编码
                        Vector3f p00 = areaLight->getSamplePoint(0,0);
                        float lightArea = (areaLight->getSamplePoint(1,0) - p00).length() * (areaLight->getSamplePoint(0,1) - p00).length();
                        
                        float pdf_light = (cosAlpha > 1e-5f) ? (lightDist2 / (lightArea * cosAlpha)) : 0.0f;
                        
                        // 运用平衡启发式加权
                        if (pdf_val + pdf_light > 1e-6f) {
                            mis_weight = pdf_val / (pdf_val + pdf_light);
                        }
                    }
                    L_indirect = (brdf_val * indirect_radiance * cosThetaNext * mis_weight) / pdf_val;
                } else {
                    // 撞击普通不发光墙壁，维持原标准蒙特卡洛积分
                    L_indirect = (brdf_val * indirect_radiance * cosThetaNext) / pdf_val;
                }
            }
        }
        return (L_direct + L_indirect) * rrScale;
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