cmake_minimum_required(VERSION 3.10)

project(lineplz-sample VERSION 1)
add_executable(sample 
	samples/lnpz-sample.cpp 
	lnpz/lnpz_fieldquadtree.cpp
	lnpz/lnpz_util.cpp
	lnpz/lnpz.cpp)

target_include_directories(sample PUBLIC "lnpz/include/")
target_compile_features(sample PUBLIC cxx_std_17)
set_target_properties(sample PROPERTIES CXX_EXTENSIONS OFF)
