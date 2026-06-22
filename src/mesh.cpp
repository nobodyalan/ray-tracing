#include "mesh.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <map>

using namespace std;

// 💡 辅助解析流：就地解析对应的 .mtl 文件，动态填充局部材质映射表
static void parseMTL(const string& mtlPath, std::map<string, Material*>& mtlMap) {
    ifstream f;
    f.open(mtlPath);
    if (!f.is_open()) {
        // 自适应脚本路径重试 1：向前回溯一级探测
        string retryMtl = string("../") + mtlPath;
        f.open(retryMtl);
        if (!f.is_open()) {
            // 自适应脚本路径重试 2：针对不 cd build 直接运行 sh run_all.sh 的情况，强行命中 testcases/
            string scriptRetryMtl = string("testcases/") + mtlPath;
            f.open(scriptRetryMtl);
            if (!f.is_open()) {
                cout << "Warning: Cannot open mtl file: " << mtlPath << endl;
                return;
            }
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
    ifstream f;
    string finalFilename = string(filename);
    f.open(filename);
    if (!f.is_open())
    {
        std::cout << "Cannot open " << filename << "\n";
        return;
    }
    // 从最终成功打开的路径句柄中，动态、精准剥离当前网格物体所在的实际物理基准目录
    string baseDir = "";
    size_t lastSlash = finalFilename.find_last_of("/\\");
    if (lastSlash != string::npos) {
        baseDir = finalFilename.substr(0, lastSlash + 1);
    }

    std::map<string, Material*> localMtlMap;
    // 💡 核心修复：此时 this->material 已在构造函数中被安全赋值，activeMaterial 获得了完美的场景兜底色
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

                // 材质精准初次对齐绑定
                Triangle* tri = new Triangle(v[t.v[0]], v[t.v[1]], v[t.v[2]], activeMaterial);
                t_faces.push_back(tri);
            }
        }
    }
    f.close();
}

bool Mesh::intersect(const Ray &r, Hit &h, float tmin) {
    bool result = false;
    for (int triIndex = 0; triIndex < (int)t_faces.size(); ++triIndex) {
        result |= t_faces[triIndex]->intersect(r, h, tmin);
    }
    return result;
}