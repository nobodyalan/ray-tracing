# 💡 优化 1：强制删除旧的 build 缓存目录，确保多线程 CMakeLists.txt 干净完整地重新配置
rm -rf build

cmake -DCMAKE_BUILD_TYPE=Release -B build
# 💡 优化 2：加上 -j 参数（比如 -j8），开启多核同时编译代码，让编译速度提高数倍
cmake --build build -j

# Run all testcases. 
# You can comment some lines to disable the run of specific examples.
mkdir -p output/niko
build/PA1 testcases/scene05.txt output/niko/scene01.bmp
