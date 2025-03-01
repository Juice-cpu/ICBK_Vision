#ifndef AUTOAIM_HIKCAMERA_H
#define AUTOAIM_HIKCAMERA_H

#include "MvCameraControl.h"
#include "VideoCapture.h"
#include <opencv2/opencv.hpp>

namespace ly {

class HikCamera : public VideoCapture {
public:
    explicit HikCamera();
    ~HikCamera() override;
    void open() override;
    void startCapture(Params_ToVideo &params) override;

private:
    void* handle;                      // 相机句柄
    unsigned char* pData;              // 图像数据缓存
    MV_FRAME_OUT_INFO_EX stImageInfo;  // 图像信息
    
    // 相机参数配置
    bool setTriggerMode(bool enable);
    bool setExposureTime(float exposure_time);
    bool setGain(float gain);
    bool setFrameRate(float frame_rate);
    bool setROI(int offsetX, int offsetY, int width, int height);
    bool startGrabbing();
    bool stopGrabbing();
};

}

#endif //AUTOAIM_HIKCAMERA_H 