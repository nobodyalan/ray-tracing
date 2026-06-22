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
        else if (token == "f") {
            string vertexToken;
            vector<int> faceVertices;

            while (ss >> vertexToken) {
                size_t firstSlash = vertexToken.find('/');
                int raw_idx = 0;
                if (firstSlash == string::npos) {
                    raw_idx = atoi(vertexToken.c_str());
                } else {
                    raw_idx = atoi(vertexToken.substr(0, firstSlash).c_str());
                }

                int final_idx = 0;
                if (raw_idx > 0) {
                    final_idx = raw_idx - 1; 
                } else if (raw_idx < 0) {
                    final_idx = (int)v.size() + raw_idx; 
                }
                faceVertices.push_back(final_idx);
            }

            for (size_t i = 1; i < faceVertices.size() - 1; ++i) {
                TriangleIndexed t;
                t.v[0] = faceVertices[0];
                t.v[1] = faceVertices[i];
                t.v[2] = faceVertices[i + 1];
                v_indices.push_back(t);

                Triangle* tri = new Triangle(v[t.v[0]], v[t.v[1]], v[t.v[2]], activeMaterial);
                t_faces.push_back(tri);
            }
        }
    }
    f.close();

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