cmake_minimum_required(VERSION 3.10)

project(lineplz-sample VERSION 1)

set(CMAKE_CXX_STANDARD 17)

if (MSVC)
    # Set compiler flags to generate PDB files in Release mode
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DEBUG /OPT:REF /OPT:ICF")
endif()

add_subdirectory(lnpz/Clipper2/CPP)

add_executable(
	sample 
	samples/lnpz-sample.cpp 
	lnpz/lnpz_fieldquadtree.cpp
	lnpz/lnpz_util.cpp
	lnpz/lnpz2d.cpp
	#lnpz/clipper.cpp
)
target_link_libraries(sample Clipper2)
target_include_directories(sample PUBLIC "lnpz/include/")
set_target_properties(sample PROPERTIES CXX_EXTENSIONS OFF)
if (MSVC)
set_target_properties(sample PROPERTIES LINK_FLAGS "/PROFILE")
endif()
#
#Testing
#

include(FetchContent)
FetchContent_Declare(
	googletest
	URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
	unittester
	test/test.cpp
	lnpz/lnpz_fieldquadtree.cpp
	lnpz/lnpz_util.cpp
	lnpz/lnpz2d.cpp
	
)

target_include_directories(unittester PUBLIC "lnpz/include/")

target_link_libraries(
	unittester	
	gtest_main
	Clipper2
)

include(GoogleTest)
gtest_discover_tests(unittester)


