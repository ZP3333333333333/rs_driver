cmake_minimum_required(VERSION 3.5)

# 注意：这里不要再写 project(...)，作为子目录被上层工程包含即可

message(=============================================================)
message("-- Ready to compile rs_pcd_saver shared library")
message(=============================================================)

# 生成动态库
add_library(rs_pcd_saver SHARED
  src/rs_pcd_saver.cpp
)

# 输出名：librs_pcd_saver.so
set_target_properties(rs_pcd_saver PROPERTIES
  OUTPUT_NAME "rs_pcd_saver"
  POSITION_INDEPENDENT_CODE ON
)

# 头文件路径：本库的 include + 你工程已有的 DRIVER_INCLUDE_DIRS
target_include_directories(rs_pcd_saver
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${DRIVER_INCLUDE_DIRS}
)

# 编译选项（可选，但建议）
# 如果你的工程已全局设置了 C++ 标准，这里可以不写
target_compile_features(rs_pcd_saver PRIVATE cxx_std_11)

# 链接依赖：直接复用你原来可执行工具的依赖集合
# 这样不会漏掉 rs_driver / pcap / pthread 等
target_link_libraries(rs_pcd_saver
  PRIVATE
    ${EXTERNAL_LIBS}
)

# 可选：安装到系统目录（你如果不需要安装可以删掉这段）
# install(TARGETS rs_pcd_saver
#         LIBRARY DESTINATION /usr/lib
#         ARCHIVE DESTINATION /usr/lib
#         RUNTIME DESTINATION /usr/bin)
