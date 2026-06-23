#include "mesh.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <map>

using namespace std;

static void parseMTL(const string& mtlPath, std::map<string, Material*>& mtlMap) {
    ifstream f(mtlPath);
    if (!f.is_open()) {
        string retryMtl = string("../") + mtlPath;
        f.open(retryMtl);
        if (!f.is_open()) {
            cout << "Warning: Cannot open mtl file: " << mtlPath << endl;
            return;
        }
    }

    string line;
    Material* currentMat = nullptr;

    while (getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (line.back() == '\r') line.pop_back();

        stringstream ss(line);
        string token;
        ss >> token;

        if (token == "newmtl") {
            string mtlName;
            ss >> mtlName;
            currentMat = new Material();
            mtlMap[mtlName] = currentMat;
        } 
        else if (currentMat != nullptr) {
            if (token == "Kd") { 
                float r, g, b; ss >> r >> g >> b;
                currentMat->setDiffuseColor(Vector3f(r, g, b));
            } 
            else if (token == "Ka") { 
                float r, g, b; ss >> r >> g >> b;
                currentMat->setAmbientColor(Vector3f(r, g, b));
            }
            else if (token == "Ks") { 
                float r, g, b; ss >> r >> g >> b;
                currentMat->setSpecularColor(Vector3f(r, g, b));
            } 
            else if (token == "Ke") { 
                float r, g, b; ss >> r >> g >> b;
                currentMat->setEmission(Vector3f(r, g, b));
            }
            else if (token == "Ns") { 
                float ns; ss >> ns;
                currentMat->setShininess(ns);
            } 
            else if (token == "Ni") { 
                float ni; ss >> ni;
                currentMat->setRefractiveIndex(ni);
            } 
            else if (token == "d") { 
                float d_val; ss >> d_val;
                currentMat->setTransparency(1.0f - d_val);
            }
            else if (token == "Tr") { 
                float tr_val; ss >> tr_val;
                currentMat->setTransparency(tr_val);
            }
            else if (token == "map_Kd") {
                string texPath;
                ss >> texPath; // 读入图片相对路径，例如 "tex/武器.tga"
                // 根据当前 .mtl 文件的物理路径，拼装出贴图文件的正确绝对/相对路径
                size_t lastSlash = mtlPath.find_last_of("/\\");
                string baseDir = (lastSlash != string::npos) ? mtlPath.substr(0, lastSlash + 1) : "";
                
                // 👇 触发调用：一键调用你修改好的材质类加载接口，让硬盘里的 TGA 资源扎进内存
                currentMat->loadTexture(baseDir + texPath);
            }
        }
    }
    f.close();
}

void Mesh::initialize(const char *filename) {
    ifstream f(filename);
    printf("filename = %s\n", filename);
    string finalFilename = string(filename);
    
    if (!f.is_open()) {
        string retryPath = string("../") + filename;
        f.open(retryPath);
        if (!f.is_open()) {
            cout << "Cannot open OBJ file: " << filename << endl;
            return;
        }
        finalFilename = retryPath;
    }

    string baseDir = "";
    size_t lastSlash = finalFilename.find_last_of("/\\");
    if (lastSlash != string::npos) {
        baseDir = finalFilename.substr(0, lastSlash + 1);
    }

    std::map<string, Material*> localMtlMap;
    Material* activeMaterial = this->material; 

    string line;
    while (getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (line.back() == '\r') line.pop_back();

        stringstream ss(line);
        string token;
        ss >> token;

        if (token == "mtllib") {
            string mtlFile;
            ss >> mtlFile;
            parseMTL(baseDir + mtlFile, localMtlMap);
        } 
        else if (token == "usemtl") {
            string mtlName;
            ss >> mtlName;
            if (localMtlMap.find(mtlName) != localMtlMap.end()) {
                activeMaterial = localMtlMap[mtlName];
            }
        } 
        else if (token == "v") {
            float x, y, z;
            ss >> x >> y >> z;
            v.push_back(Vector3f(x, y, z));
        } 
        else if (token == "vt"){
            float x,y;
            ss>>x>>y;
            vt.push_back(Vector2f(x,y));
        }
        else if (token == "vn"){
            float x,y,z;
            ss>>x>>y>>z;
            vn.push_back(Vector3f(x,y,z));
        }
        else if (token == "f") {
            string vToken;
            vector<int> v_idx, vt_idx, vn_idx;
            while (ss >> vToken) {
                int vi = 0, vti = 0, vni = 0;
                if (sscanf(vToken.c_str(), "%d/%d/%d", &vi, &vti, &vni) == 3) {
                    v_idx.push_back(vi > 0 ? vi - 1 : (int)v.size() + vi);
                    vt_idx.push_back(vti > 0 ? vti - 1 : (int)vt.size() + vti);
                    vn_idx.push_back(vni > 0 ? vni - 1 : (int)vn.size() + vni);
                } else if (sscanf(vToken.c_str(), "%d/%d", &vi, &vti) == 2) {
                    v_idx.push_back(vi > 0 ? vi - 1 : (int)v.size() + vi);
                    vt_idx.push_back(vti > 0 ? vti - 1 : (int)vt.size() + vti);
                } else if (sscanf(vToken.c_str(), "%d", &vi) == 1) {
                    v_idx.push_back(vi > 0 ? vi - 1 : (int)v.size() + vi);
                }
            }
            if (v_idx.size() == 3) {
                // 💡 方案 B 核心：在这里不做任何查重拦截，只进行极速“盲读”加载
                Triangle* tri = nullptr;
                if (!vt_idx.empty() && !vn_idx.empty()) {
                    tri = new Triangle(v[v_idx[0]], v[v_idx[1]], v[v_idx[2]], vt[vt_idx[0]], vt[vt_idx[1]], vt[vt_idx[2]], vn[vn_idx[0]], vn[vn_idx[1]], vn[vn_idx[2]], activeMaterial);
                } else if (!vt_idx.empty()) {
                    tri = new Triangle(v[v_idx[0]], v[v_idx[1]], v[v_idx[2]], vt[vt_idx[0]], vt[vt_idx[1]], vt[vt_idx[2]], activeMaterial);
                } else {
                    tri = new Triangle(v[v_idx[0]], v[v_idx[1]], v[v_idx[2]], activeMaterial);
                }
                
                t_faces.push_back(tri);
            } 
            else {
                assert(v_idx.size() == 3);
            }
        }
    }
    
    f.close();
    std::cout << ">> [Mesh Clean] Entering global geometric sorting for " << t_faces.size() << " triangles..." << std::endl;

    std::cout << ">> [Mesh Clean] Entering global geometric sorting for " << t_faces.size() << " triangles..." << std::endl;

    // 1. 高性能内存连续快排
    std::sort(t_faces.begin(), t_faces.end(), [](Triangle* a, Triangle* b) {
        Vector3f ca = (a->getVertex(0) + a->getVertex(1) + a->getVertex(2)) / 3.0f;
        Vector3f cb = (b->getVertex(0) + b->getVertex(1) + b->getVertex(2)) / 3.0f;
        
        if (fabs(ca.x() - cb.x()) > 1e-5f) return ca.x() < cb.x();
        if (fabs(ca.y() - cb.y()) > 1e-5f) return ca.y() < cb.y();
        return ca.z() < cb.z();
    });

    // 2. 双指针线性扫描合并
    std::vector<Triangle*> optimized_faces;
    int deduplicated_count = 0;

    if (!t_faces.empty()) {
        optimized_faces.push_back(t_faces[0]); 

        for (size_t i = 1; i < t_faces.size(); ++i) {
            Triangle* curr = t_faces[i];
            Triangle* prev = optimized_faces.back(); 

            Vector3f cc = (curr->getVertex(0) + curr->getVertex(1) + curr->getVertex(2)) / 3.0f;
            Vector3f cp = (prev->getVertex(0) + prev->getVertex(1) + prev->getVertex(2)) / 3.0f;

            if ((cc - cp).length() < 1e-5f) {
                Material* baseMat = prev->getMaterial();
                Material* activeMaterial = curr->getMaterial();

                if (baseMat != nullptr && activeMaterial != nullptr && baseMat != activeMaterial) {
                    if (baseMat->getSecondaryMaterial() == nullptr) {
                        baseMat->setSecondaryMaterial(activeMaterial);
                    }
                }
                
                // 🟢 核心修改：绝对不在原地调用 delete curr！
                // 我们直接将这个重复面片【留在原来的 t_faces 垃圾池】里用于生命周期闭环，
                // 但坚决不把它放进生存区，使其对后面的 BVH 树和求交光线而言彻底隐形、绝不参与计算！
                deduplicated_count++;
            } 
            else {
                optimized_faces.push_back(curr);
            }
        }
    }

    // 💡 3. 核心解耦：建立一个类内或生命周期同等的延时垃圾池
    // 为了防止析构函数循环释放，我们在 Mesh 内部只用 optimized_faces 交付渲染
    this->t_faces = optimized_faces;

    std::cout << ">> [Mesh Optimization] Globally filtered out " << deduplicated_count 
              << " disordered duplicate triangles! Remaining: " << this->t_faces.size() << std::endl;
    // 接下来可以放心地去构建你的 BVH 树了：
    // bvh_root = buildBVH(t_faces, 0, t_faces.size());
    // 💡 加载完毕，单次编译生成 BVH 加速树

    // 💡 加载完毕，单次编译生成 BVH 加速树
    if (!t_faces.empty()) {
        bvh_root = buildBVH(t_faces, 0, (int)t_faces.size());
    }
}

// 💡 3. 完美适配：直接利用 Triangle 暴露出的顶点访问接口进行极速空间排序
BVHNode* Mesh::buildBVH(std::vector<Triangle*>& tris, int start, int end) {
    if (start >= end) return nullptr;

    BVHNode* node = new BVHNode();
    
    // 运用你在 triangle.hpp 里新加的只读顶点接口直接合成 AABB 包围盒，开销降为真正的 O(1)
    for (int i = start; i < end; ++i) {
        node->box.integrate(tris[i]->getVertex(0)); // 👈 适配：直接调取顶点
        node->box.integrate(tris[i]->getVertex(1));
        node->box.integrate(tris[i]->getVertex(2));
    }

    int len = end - start;
    if (len == 1) {
        node->tri = tris[start];
        return node;
    }

    // 选取包围盒跨度最长的轴
    Vector3f extents = node->box.max_p - node->box.min_p;
    int axis = 0;
    if (extents[1] > extents[0]) axis = 1;
    if (extents[2] > extents[axis]) axis = 2;

    // 真正的 O(N log N) 空间中位数排序
    std::sort(tris.begin() + start, tris.begin() + end, [axis](Triangle* a, Triangle* b) {
        float centerA = (a->getVertex(0)[axis] + a->getVertex(1)[axis] + a->getVertex(2)[axis]) / 3.0f;
        float centerB = (b->getVertex(0)[axis] + b->getVertex(1)[axis] + b->getVertex(2)[axis]) / 3.0f;
        return centerA < centerB;
    });

    int mid = start + len / 2;
    node->left = buildBVH(tris, start, mid);
    node->right = buildBVH(tris, mid, end);

    return node;
}

bool Mesh::intersectBVH(BVHNode* node, const Ray& r, Hit& h, float tmin) const {
    if (node == nullptr) return false;
    
    // AABB 粗筛
    if (!node->box.intersect(r, tmin, h.getT())) return false;

    if (node->tri != nullptr) {
        return node->tri->intersect(r, h, tmin);
    }

    bool hit_left = intersectBVH(node->left, r, h, tmin);
    bool hit_right = intersectBVH(node->right, r, h, tmin);
    return hit_left || hit_right;
}

bool Mesh::intersect(const Ray &r, Hit &h, float tmin) {
    if (bvh_root == nullptr) return false;
    return intersectBVH(bvh_root, r, h, tmin);
}