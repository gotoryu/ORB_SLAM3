echo "Configuring and building ORB_SLAM3 ..."

rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j$(nproc)

# Keep compile_commands.json at workspace root for tools that do not auto-detect build/.
ln -sf build/compile_commands.json compile_commands.json
