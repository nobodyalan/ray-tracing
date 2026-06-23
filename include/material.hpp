// #ifndef MATERIAL_H
// #define MATERIAL_H

// #include <cassert>
// #include <vecmath.h>
// #include "ray.hpp"
// #include "hit.hpp"
// #include "image.hpp" // 💡 新增：引入大作业自带的 Image 图像库
// #include <iostream>

// class Material
// {
// public:
//     explicit Material(const Vector3f &d_color, const Vector3f &s_color = Vector3f::ZERO, float s = 0,
//                       const Vector3f &r_color = Vector3f::ZERO, float i = 1, const Vector3f &e_color = Vector3f::ZERO, 
//                       float t = 1.0f, float _roughness = 0.3f) : 
//                       diffuseColor(d_color), specularColor(s_color), shininess(s), 
//                       reflectiveColor(r_color), indexOfRefraction(i), emissionColor(e_color), transparency(t), roughness(_roughness),
//                       texture(nullptr), has_texture(false) // 💡 增量初始化
//     {
//     }
    
//     // 💡 默认构造函数，供解析 .mtl 时动态创建并用 set 赋值
//     Material() {
//         diffuseColor = Vector3f::ZERO;
//         specularColor = Vector3f::ZERO;
//         shininess = 0.0f;
//         indexOfRefraction = 1.0f; 
//         transparency = 0.0f;
//         emissionColor = Vector3f::ZERO;
//         texture = nullptr;      // 💡 增量初始化
//         has_texture = false;    // 💡 增量初始化
//     }
    
//     virtual ~Material() {
//         if (texture) {
//             delete texture;    // 💡 妥善释放绑定的纹理图片内存
//         }
//     }
    
//     virtual float getTransparency() const { return transparency; }
//     virtual Vector3f getEmission() const { return emissionColor; }
    
//     // 💡 保持原汁原味：完全不影响你现有的无参数常数单色获取
//     virtual Vector3f getDiffuseColor() const { return diffuseColor; }
    

// // 方法定义
//     void setAmbientColor(const Vector3f& ka) { ambientColor = ka; }
//     Vector3f getAmbientColor() const { return ambientColor; }
//     // 💡 新增：针对纹理映射的重载函数。若该材质加载了贴图，则根据 UV 进行像素级采样
//     // virtual Vector3f getDiffuseColor(float u, float v) const {
//     //     if (has_texture && texture != nullptr) {
//     //         // 将 UV 映射到图片的 Width 和 Height 像素坐标，并做取模以支持平铺（Tiling）防止数组越界
//     //         int w = (int)(u * texture->Width()) % texture->Width();
//     //         int h = (int)((1.0f - v) * texture->Height()) % texture->Height(); // 💡 统一图形学 Y/V 轴反转规范
//     //         if (w < 0) w += texture->Width();
//     //         if (h < 0) h += texture->Height();
//     //         return texture->GetPixel(w, h);
//     //     }
//     //     return diffuseColor; // 兜底返回解析出的常数 Kd
//     // }
//     Vector3f getDiffuseColor(float u, float v) const {
//     if (texture != nullptr) {
//         float safe_u = u; if (safe_u < 0.0f) safe_u = 0.0f; if (safe_u >= 1.0f) safe_u = 0.9999f;
//         float safe_v = v; if (safe_v < 0.0f) safe_v = 0.0f; if (safe_v >= 1.0f) safe_v = 0.9999f;

//         int x = (int)(safe_u * texture->Width());
//         int y = (int)(safe_v * texture->Height());

//         if (x >= 0 && x < texture->Width() && y >= 0 && y < texture->Height()) {
//             Vector3f srgbColor = texture->GetPixel(x, y);
            
//             // 💡 逆 Gamma 操作 (Linearize)：将贴图颜色从 sRGB 转换到线性空间
//             return Vector3f(
//                 powf(srgbColor.x(), 2.2f),
//                 powf(srgbColor.y(), 2.2f),
//                 powf(srgbColor.z(), 2.2f)
//             );
//         }
//     }
//     // 如果没有纹理，降级返回常数底色，常数固有色通常在建模软件里也是线性空间的
//     return diffuseColor; 
// }
    
//     virtual float getRoughness() const { return roughness; }
//     virtual Vector3f getSpecularColor() const { return specularColor; }
//     virtual float getShininess() const { return shininess; }
//     virtual Vector3f getReflectiveColor() const { return reflectiveColor; }
//     virtual float getRefractiveIndex() const { return indexOfRefraction; }

//     void setDiffuseColor(const Vector3f &color) { diffuseColor = color; }
//     void setSpecularColor(const Vector3f &color) { specularColor = color; }
//     void setShininess(float s) { shininess = s; }
//     void setRefractiveIndex(float r_idx) { indexOfRefraction = r_idx; }
//     void setTransparency(float t) { transparency = t; }
//     void setEmission(const Vector3f &e) { emissionColor = e; }
//     void loadTexture(const std::string& path) {
//         texture = Image::LoadTGA(path.c_str()); 
//         if (texture != nullptr) {
//             has_texture = true;
//             std::cout << "Successfully loaded TGA texture: " << path << std::endl;
//         } else {
//             std::cout << "Warning: TGA texture failed to load: " << path << std::endl;
//         }
//     }

// protected:
//     Vector3f diffuseColor;
//     Vector3f specularColor;
//     float shininess;
//     Vector3f reflectiveColor;
//     float indexOfRefraction;
//     Vector3f emissionColor;
//     float transparency;
//     float roughness;
//     Vector3f ambientColor = Vector3f::ZERO;
//     Image* texture;        // 💡 新增：持有的纹理类指针
//     bool has_texture;      // 💡 新增：标记当前材质是否被贴图覆盖
// };
// #endif

#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>
#include "ray.hpp"
#include "hit.hpp"
#include "image.hpp" // 💡 新增：引入大作业自带的 Image 图像库
#include <iostream>

class Material
{
public:
    explicit Material(const Vector3f &d_color, const Vector3f &s_color = Vector3f::ZERO, float s = 0,
                      const Vector3f &r_color = Vector3f::ZERO, float i = 1, const Vector3f &e_color = Vector3f::ZERO, 
                      float t = 1.0f, float _roughness = 0.3f,std::string _matname = "defaultmatname",Material* _secondary = nullptr) : 
                      diffuseColor(d_color), specularColor(s_color), shininess(s), 
                      reflectiveColor(r_color), indexOfRefraction(i), emissionColor(e_color), transparency(t), roughness(_roughness),
                      texture(nullptr), has_texture(false),matName(_matname),secondary_mat(_secondary) // 💡 增量初始化
    {
    }
    // 💡 默认构造函数，供解析 .mtl 时动态创建并用 set 赋值
    Material() {
        diffuseColor = Vector3f::ZERO;
        specularColor = Vector3f::ZERO;
        shininess = 0.0f;
        indexOfRefraction = 1.0f; 
        transparency = 0.0f;
        emissionColor = Vector3f::ZERO;
        texture = nullptr;      // 💡 增量初始化
        has_texture = false;    // 💡 增量初始化
        matName = "defaultmatname";
        secondary_mat = nullptr;
    }
    void setSecondaryMaterial(Material* secondary) {
        // 如果传入的副材质就是自己，或者已经有副材质了，直接拒绝，死锁最大 2 层
        if (secondary == nullptr || secondary == this || this->secondary_mat != nullptr) {
            return;
        }
        this->secondary_mat = secondary;
    }

    // 💡 核心新增：获取二级副材质指针的接口
    Material* getSecondaryMaterial() const {
        return secondary_mat;
    }
    virtual ~Material() {
        if (texture) {
            delete texture;    // 💡 妥善释放绑定的纹理图片内存
        }
    }
    
    virtual float getTransparency() const { return transparency; }
    virtual Vector3f getEmission() const { return emissionColor; }
    std::string getName() const {
        return matName;
    }

    void setName(const std::string& name) {
        matName = name;
    }
    
    // 💡 保持原汁原味：完全不影响你现有的无参数常数单色获取
    virtual Vector3f getDiffuseColor() const { return diffuseColor; }
    

// 方法定义
    void setAmbientColor(const Vector3f& ka) { ambientColor = ka; }
    Vector3f getAmbientColor() const { return ambientColor; }
    Vector3f getDiffuseColor(float u, float v) const {
    if (texture != nullptr) {
        float safe_u = u; if (safe_u < 0.0f) safe_u = 0.0f; if (safe_u >= 1.0f) safe_u = 0.9999f;
        float safe_v = v; if (safe_v < 0.0f) safe_v = 0.0f; if (safe_v >= 1.0f) safe_v = 0.9999f;

        int x = (int)(safe_u * texture->Width());
        int y = (int)(safe_v * texture->Height());
        Vector3f srgbColor = diffuseColor; // 先初始化为主材质的基础颜色
        if (x >= 0 && x < texture->Width() && y >= 0 && y < texture->Height()) {
            srgbColor = texture->GetPixel(x, y);
            if (secondary_mat != nullptr && secondary_mat->texture != nullptr) {
                // 如果存在副材质且副材质有贴图，则进行叠加采样
                float safe_u_sub = fmod(u, 1.0f); if (safe_u_sub < 0.0f) safe_u_sub += 1.0f;
                float safe_v_sub = fmod(v, 1.0f); if (safe_v_sub < 0.0f) safe_v_sub += 1.0f;
                int sx = (int)(safe_u_sub * secondary_mat->texture->Width());
                int sy = (int)(safe_v_sub * secondary_mat->texture->Height());
                if (sx >= 0 && sx < secondary_mat->texture->Width() && sy >= 0 && sy < secondary_mat->texture->Height()) {
                    Vector3f sub_color = secondary_mat->texture->GetPixel(sx, sy);
                    // 简单的线性叠加，权重可以根据需要调整

                    bool isMainBlack = srgbColor.length() < 0.02f;
                    bool isSubBlack = sub_color.length() < 0.02f;
                    if(isMainBlack && !isSubBlack)
                    {
                        srgbColor = sub_color;
                    }
                    else if(!isMainBlack && !isSubBlack)
                    {
                        srgbColor = 0.7*srgbColor + 0.5*sub_color;
                    }
                }
            }
            // 💡 逆 Gamma 操作 (Linearize)：将贴图颜色从 sRGB 转换到线性空间
            return Vector3f(
                powf(srgbColor.x(), 2.2f),
                powf(srgbColor.y(), 2.2f),
                powf(srgbColor.z(), 2.2f)
            );
        }
    }
    // 如果没有纹理，降级返回常数底色，常数固有色通常在建模软件里也是线性空间的
    return diffuseColor; 
}
    
    
    virtual float getRoughness() const { return roughness; }
    virtual Vector3f getSpecularColor() const { return specularColor; }
    virtual float getShininess() const { return shininess; }
    virtual Vector3f getReflectiveColor() const { return reflectiveColor; }
    virtual float getRefractiveIndex() const { return indexOfRefraction; }

    void setDiffuseColor(const Vector3f &color) { diffuseColor = color; }
    void setSpecularColor(const Vector3f &color) { specularColor = color; }
    void setShininess(float s) { shininess = s; }
    void setRefractiveIndex(float r_idx) { indexOfRefraction = r_idx; }
    void setTransparency(float t) { transparency = t; }
    void setEmission(const Vector3f &e) { emissionColor = e; }
    void loadTexture(const std::string& path) {
        texture = Image::LoadTGA(path.c_str()); 
        if (texture != nullptr) {
            has_texture = true;
            std::cout << "Successfully loaded TGA texture: " << path << std::endl;
        } else {
            std::cout << "Warning: TGA texture failed to load: " << path << std::endl;
        }
    }

protected:
    Vector3f diffuseColor;
    Vector3f specularColor;
    float shininess;
    Vector3f reflectiveColor;
    float indexOfRefraction;
    Vector3f emissionColor;
    float transparency;
    float roughness;
    Vector3f ambientColor = Vector3f::ZERO;
    Image* texture;        // 💡 新增：持有的纹理类指针
    bool has_texture;      // 💡 新增：标记当前材质是否被贴图覆盖
    std::string matName;
    Material* secondary_mat = nullptr;
};

#endif // MATERIAL_H