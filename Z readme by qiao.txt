 二、硬件配置清单
1. 图像采集设备：
大恒工业相机（MER-500-14U3M-L，推荐型号）
海康威视工业相机（MV-CH050-10UM，备选方案）
支持USB3.0接口，需配备C/CS接口镜头（推荐焦距8mm）

计算设备：
NVIDIA Jetson Xavier NX 或 AGX Xavier
或Intel NUC i7 + NVIDIA GTX 1650以上显卡
内存：建议8GB以上

通信设备：
USB转TTL串口模块（推荐CH340G芯片）
或STM32F4系列开发板（用于协议转换）

机械结构：
云台机构（需支持PWM/PPM控制）
双轴稳定云台（俯仰轴+偏航轴）

三、软件环境要求
操作系统：
Ubuntu 18.04/20.04 LTS
Linux内核版本4.15+

核心依赖库：
OpenCV 4.5+（需编译contrib模块）
Eigen3 3.3.7+
Ceres Solver 2.0.0+
GLOG 0.4.0+

深度学习框架：
OpenVINO 2021.4+
ONNX Runtime 1.8+
PyTorch 1.8+（仅训练需要）

4. 其他工具：
CMake 3.16+
Boost 1.71+
ROS Noetic（可选）
 
 
 
 训练相关文件
trainSVM.h & trainSVM.cpp
这两个文件用于训练装甲板分类器：
用于收集装甲板数据并训练SVM分类器
训练数据存放在 utils/data/pictures/armors 目录下
训练好的模型保存在 utils/tools/svm_numbers.xml
使用方法：
# 1. 收集装甲板图片数据到指定目录
# 2. 编译运行训练程序
./trainSVM


主程序文件
AutoAim.h & AutoAim.cpp
主程序入口，负责初始化和管理各个模块：
int main(int argc, char** argv) {
    // 1. 初始化日志系统
    auto log = new Log();
    log->init(argv[0]);

    // 2. 加载配置文件
    auto config = new Config(confog_file_path);
    config->parse();

    // 3. 初始化串口通信
    auto serial_port = new SerialPort();
    
    // 4. 初始化相机
    ly::VideoCapture* video;
    video->chooseCameraType(video);
    
    // 5. 初始化检测器
    auto detector = new Detector();
}


 功能模块文件
Config.h
配置文件解析类：
class Config {
public:
    void parse();  // 解析json配置文件
    void chooseCameraType(VideoCapture*& video);  // 选择相机类型
};
使用方法：
// 在init.json中配置参数
{
    "camera": {
        "device_type": 0,  // 相机类型
        "exposure_time": 5000  // 曝光时间
    }
}


Bumper.hpp
状态缓冲器，用于平滑检测结果：
enum DETECT_MODE {
    NOT_GET_TARGET = 0,       // 未获得目标
    CONTINOUS_GET_TARGET = 1, // 连续获得目标
    LOST_BUMP = 2,           // 缓冲阶段
    DETECT_BUMP = 3          // 进入连续识别状态的缓冲
};
使用方法：
Bumper bumper;
int mode = bumper.getDetectMode(target_class, last_frame_class);


ArmorFinder.h
装甲板检测的核心类：
class ArmorFinder {
public:
    bool judgeArmor(const ArmorBlob&);  // 判断是否为装甲板
    bool matchTwoLightBar(const RotatedRect&, const RotatedRect&);  // 匹配灯条
    bool getArmor(const RotatedRect&, const RotatedRect&, ArmorBlob& armor);  // 获取装甲板信息
};
使用流程：
检测灯条
匹配灯条对
提取装甲板
分类识别


运动控制实现
PID参数整定步骤：
1. 先调P：逐渐增大比例系数直到云台出现等幅振荡
再调D：增加微分系数抑制超调（典型值：P的1/10~1/5）
最后调I：消除静差（积分时间设为系统周期的3-5倍）
// 增量式PID实现
typedef struct {
    float Kp, Ki, Kd;
    float error[3]; // 当前、前一次、前两次误差
    float output;
} PID_Controller;

float PID_Update(PID_Controller* pid, float setpoint, float measurement) {
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = setpoint - measurement;

    float delta = pid->Kp * (pid->error[0] - pid->error[1]) 
                + pid->Ki * pid->error[0]
                + pid->Kd * (pid->error[0] - 2*pid->error[1] + pid->error[2]);
    
    pid->output += delta;
    return pid->output;
}


联合调试
典型问题排查表：
| 现象 | 可能原因 | 排查工具 | 解决方法 |
|-----------------------|---------------------------|------------------------|------------------------------|
| 数据断续 | 波特率不匹配 | 示波器 | 检查双方波特率设置 |
| 云台抖动 | PID参数过冲 | 数据记录仪 | 降低P值，增加D值 |
| 延迟超过50ms | 串口缓冲区溢出 | 逻辑分析仪 | 优化协议格式，减少数据量 |
| 控制量反向 | 电机极性接反 | 万用表 | 调换电机线序 |
| 上电复位异常 | 电源浪涌 | 电流探头 | 增加缓启动电路 |


操作流程
1. 启动顺序：
# 1. 启动相机驱动
./bin/camera_driver -c config/camera_params.yaml

# 2. 启动主识别程序
./bin/auto_aim -m models/armor.onnx

# 3. 启动调试界面（可选）
./bin/debug_ui --ros

2. 调试工具使用：
按F1显示帮助信息
Ctrl+鼠标左键框选ROI区域
W/S调整曝光时间
R键重置跟踪器

故障排查指南
常见问题：
相机连接失败：检查USB3.0接口供电，运行lsusb确认设备枚举
串口通信异常：检查波特率设置，使用minicom测试串口收发
识别率低：调整armor_detector/params/ThresholdParams.h中的颜色阈值
2. 日志查看：
tail -f logs/main.log
# 调试级别日志
export GLOG_v=2