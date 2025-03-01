# find_package(OpenCV 4 REQUIRED)

set(CMAKE_PREFIX_PATH "/usr/local/include/opencv4/")
set(OpenCV_DIR /usr/local/include/opencv4/)
find_package(OpenCV 4 REQUIRED)

# 添加海康相机SDK
find_package(MVS REQUIRED)

include_directories(
        include
        ../utils/include
        ../armor_detector/include
        ${OpenCV_INCLUDE_DIRS}
        ${MVS_INCLUDE_DIRS}  # 添加海康相机头文件
)

add_library(driver
        src/SerialPort.cpp
        src/VideoCapture.cpp
        src/GxCamera.cpp
        src/VideoSaver.cpp
        src/Picture.cpp
        src/DaHen.cpp
        src/Video.cpp
        src/HikCamera.cpp  # 添加海康相机源文件
)

target_link_libraries(driver
        utils
        -lpthread
        -lboost_system
        libgxiapi.so
        ${OpenCV_LIBS}
        ${MVS_LIBS}  # 添加海康相机库
)