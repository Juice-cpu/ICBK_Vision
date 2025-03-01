#include "HikCamera.h"
#include "Log.h"

namespace ly {

HikCamera::HikCamera() {
    handle = nullptr;
    pData = nullptr;
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
}

HikCamera::~HikCamera() {
    // 停止取流
    if (handle) {
        MV_CC_StopGrabbing(handle);
    }
    
    // 关闭设备
    if (handle) {
        MV_CC_DestroyHandle(handle);
        handle = nullptr;
    }

    // 释放缓存
    if (pData) {
        free(pData);
        pData = nullptr;
    }
}

void HikCamera::open() {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Enum devices fail! nRet [" << nRet << "]";
        return;
    }

    if (stDeviceList.nDeviceNum == 0) {
        LOG(ERROR) << "No camera found!";
        return;
    }

    // 创建句柄
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Create handle fail! nRet [" << nRet << "]";
        return;
    }

    // 打开设备
    nRet = MV_CC_OpenDevice(handle);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Open device fail! nRet [" << nRet << "]";
        return;
    }

    // 设置触发模式为off
    if (!setTriggerMode(false)) {
        return;
    }

    // 设置曝光时间
    if (!setExposureTime(CameraParam::exposure_time)) {
        return;
    }

    // 设置增益
    if (!setGain(CameraParam::gain)) {
        return;
    }

    // 设置帧率
    if (!setFrameRate(210.0f)) {
        return;
    }

    // 设置ROI
    if (!setROI(offset_x, offset_y, width, height)) {
        return;
    }

    // 开始取流
    if (!startGrabbing()) {
        return;
    }

    // 分配数据缓存
    pData = (unsigned char*)malloc(sizeof(unsigned char) * width * height * 3);
}

void HikCamera::startCapture(Params_ToVideo &params) {
    // params out
    _video_thread_params.frame_pp = params.frame_pp;

    int id = 0;
    constexpr int size = 10;
    Image frame[size];
    for(auto& m: frame) m.mat = new cv::Mat(cv::Size(width, height), CV_8UC3);

    std::chrono::steady_clock::time_point start, end;

    while (true) {
        std::unique_lock<std::mutex> umtx_video(Thread::mtx_image);
        while(Thread::image_is_update) {
            Thread::cond_is_process.wait(umtx_video);
        }

        start = std::chrono::steady_clock::now();

        // 获取一帧图像
        MV_FRAME_OUT stOutFrame = {0};
        int nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 1000);
        if (nRet == MV_OK) {
            memcpy(pData, stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3);
            
            // 转换为OpenCV格式
            cv::Mat src(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, 
                       CV_8UC3, pData);
            src.copyTo(*frame[id].mat);

            // 释放图像缓存
            nRet = MV_CC_FreeImageBuffer(handle, &stOutFrame);
            if (nRet != MV_OK) {
                LOG(ERROR) << "Free image buffer fail! nRet [" << nRet << "]";
            }
        } else {
            LOG(ERROR) << "Get image failed! nRet [" << nRet << "]";
            continue;
        }

        end = std::chrono::steady_clock::now();
        
        frame[id].time_stamp = start + (end-start) / 2;
        frame[id].imu_data = SerialParam::recv_data;
        *_video_thread_params.frame_pp = &frame[id];
        
        Thread::image_is_update = true;
        Thread::cond_is_update.notify_one();
        umtx_video.unlock();
        
        id = (id+1) % size;
        
        LOG_IF(ERROR, (*_video_thread_params.frame_pp)->mat->empty()) << "get empty picture mat!";
    }
}

bool HikCamera::setTriggerMode(bool enable) {
    int nRet = MV_CC_SetEnumValue(handle, "TriggerMode", enable ? 1 : 0);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Set trigger mode fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

bool HikCamera::setExposureTime(float exposure_time) {
    int nRet = MV_CC_SetFloatValue(handle, "ExposureTime", exposure_time);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Set exposure time fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

bool HikCamera::setGain(float gain) {
    int nRet = MV_CC_SetFloatValue(handle, "Gain", gain);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Set gain fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

bool HikCamera::setFrameRate(float frame_rate) {
    int nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", frame_rate);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Set frame rate fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

bool HikCamera::setROI(int offsetX, int offsetY, int width, int height) {
    int nRet;
    nRet = MV_CC_SetIntValue(handle, "Width", width);
    nRet |= MV_CC_SetIntValue(handle, "Height", height);
    nRet |= MV_CC_SetIntValue(handle, "OffsetX", offsetX);
    nRet |= MV_CC_SetIntValue(handle, "OffsetY", offsetY);
    
    if (nRet != MV_OK) {
        LOG(ERROR) << "Set ROI fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

bool HikCamera::startGrabbing() {
    int nRet = MV_CC_StartGrabbing(handle);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Start grabbing fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

bool HikCamera::stopGrabbing() {
    int nRet = MV_CC_StopGrabbing(handle);
    if (nRet != MV_OK) {
        LOG(ERROR) << "Stop grabbing fail! nRet [" << nRet << "]";
        return false;
    }
    return true;
}

} 