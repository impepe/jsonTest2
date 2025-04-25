#include <chrono>
#include <memory>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <sys/select.h>
#include <boost/bimap.hpp>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "my_interface/msg/motor_drive.hpp"
#include "my_interface/msg/motor_feedback.hpp"
#include "nlohmann/json.hpp"

using namespace std::chrono_literals;
using namespace my_interface::msg;
using json = nlohmann::json;

static const int baudrate = B1000000;
static const std::string port = "/dev/ttyACM3";
static const int serialCheckFreq = 300;

// 串口功能类
class serialClass
{
    public:
        // 指向串口
        int serial;
        serialClass()
        {
            //非阻塞模式打开串口
            serial = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (serial == -1) {
                std::cout << "无法打开串口" << port << std::endl;
                std::exit(0);
            }
            else{
                std::cout << "打开串口" << port << std::endl;
            }
            //获取串口配置
            struct termios options;
            tcgetattr(serial, &options);
            // 自定义设置波特率
            cfsetispeed(&options, baudrate);
            cfsetospeed(&options, baudrate);
            //options.c_cflag |= (CLOCAL | CREAD); //无需硬件流控制，启用串口接收器
            // 设置数据位、停止位和校验位
            options.c_cflag &= ~PARENB; // 无校验位
            options.c_cflag &= ~CSTOPB; // 1 位停止位
            options.c_cflag &= ~CSIZE;  // 清除数据位大小的设置
            options.c_cflag |= CS8;     // 设置8位数据位
            // 设置为原始模式
            // options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            // options.c_oflag &= ~OPOST;
            // 配置生效
            tcsetattr(serial, TCSANOW, &options);
        }
        
        ~serialClass()
        {
            // 关闭串口
            close(serial);
            std::cout << "已关闭串口" << port << std::endl;
        }

        static bool writeToSerial(const int serial, const char* data_charPtr) {
        // 静态函数，被发送节点调用
        // 向串口发送字符串
            // 检查空指针
            if (data_charPtr == nullptr) {
                std::cerr << "错误：发送数据指针为空！" << std::endl;
                return false;
            }
            // 向串口发送数据
            size_t datalen = strlen(data_charPtr);
            ssize_t bytesWritten = write(serial, data_charPtr, datalen);
            // 检查发送是否成功
            if (bytesWritten == -1) {
                std::cerr << "错误：串口发送失败！" << std::endl;
                return false;
            }
            return true;
        }

        static bool readJsonFromSerial(const int serial, std::string& fbkStr){
        // 静态函数，被接收节点调用
        // 读取串口json字符串，以}为结束符
            char byte;
            // 获取缓冲区数据长度
            // 备注：不可使用read来判断是否串口有数据，否则后续新数据会被破坏
            int bytesAvailable;
            if (ioctl(serial, FIONREAD, &bytesAvailable) == -1){
                std::cerr << "错误：串口缓冲区数据长度获取失败！" << std::endl;
                std::exit(0);
            }
            if (bytesAvailable == 0)  return false;
            // 有数据可读
            else {
                // 对下述代码进行计时
                // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                // 超时判断，用于防止单片机死机或手动断电时陷入死循环
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                // 循环读取直到}结束符
                while (byte != 0x7d){
                    if (read(serial, &byte, 1) == 1){
                        fbkStr += byte;
                        t1 = std::chrono::steady_clock::now();
                    }
                    t2 = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(
                             t2 - t1).count() > 50){
                                std::cerr << "串口反馈死机！" << std::endl;
                                return false;
                             }
                }
                // std::cout << "读取耗时： " << std::chrono::duration_cast<std::chrono::microseconds>(
                //     std::chrono::steady_clock::now() - begin).count() << "us" << std::endl;
            }
            return true;
        }

    private:

};

// 串口实例化
serialClass mySerial;

// 关节名称和电机id双向查找表
typedef boost::bimap<std::string, int> stringIntBiMap;
static stringIntBiMap jointIdMap;
static void biMapInit(){
// 初始化关节id查找表
    jointIdMap.insert(stringIntBiMap::value_type("bottom",1));
    jointIdMap.insert(stringIntBiMap::value_type("arm",2));
    jointIdMap.insert(stringIntBiMap::value_type("lift",3));
}

// 订阅电机命令节点类
class driveNodeClass : public rclcpp::Node
{
    public:
        driveNodeClass()
        : Node("driveNode")
        {
            // ros2订阅者节点，用来接收电机命令
            driveCmd_subscriptor = this->create_subscription<MotorDrive>("MotorDrive_topic", 10,
                                    std::bind(&driveNodeClass::drive_callback, this, std::placeholders::_1));
        }

    private:
        // ros2订阅对象指针
        rclcpp::Subscription<MotorDrive>::SharedPtr driveCmd_subscriptor;
        // ros2订阅回调函数
        void drive_callback(const MotorDrive::SharedPtr driveCmdMsg) const
        {
        // 将ros2消息转化为json格式并调用方法发送数据
            std::cout << "接收到命令" << std::endl;

            // 将关节名称转化为电机id
            std::vector<std::int32_t> motorId;
            for (const auto& joint : driveCmdMsg->joint)
            {
                auto it = jointIdMap.left.find(joint);
                if (it != jointIdMap.left.end())    motorId.push_back(it->second);
                else {
                    RCLCPP_ERROR(this->get_logger(), "错误：关节名称不存在！");
                    return ;
                }                       
            }
            // 将ros2消息转化为json数据
            json driveCmdJson;
            driveCmdJson["id"] = motorId;
            driveCmdJson["md"] = driveCmdMsg->mode;
            driveCmdJson["ps"] = driveCmdMsg->pos;
            driveCmdJson["vl"] = driveCmdMsg->vel;
            driveCmdJson["tq"] = driveCmdMsg->tq;
            // driveCmdJson["kp"] = driveCmdMsg->kp;
            // driveCmdJson["kd"] = driveCmdMsg->kd;
            std::string driveCmdJson_Str = driveCmdJson.dump();
            
            // 串口下发电机命令，用字节指针更安全
            char* driveCmdJson_charPtr = const_cast<char*>(driveCmdJson_Str.c_str());
            static_cast<void>(serialClass::writeToSerial(mySerial.serial, driveCmdJson_charPtr));
            // 如果为单次运动，而不是控制模式，则重发数据
            // while ((rst == false) && (driveCmdMsg->type == "single"))
            // {
            //     rst = serialClass::writeToSerial(driveCmdJson_charPtr);
            // }
        }
};

// 发布电机状态节点类
class fbkNodeClass : public rclcpp::Node
{
    public:
        fbkNodeClass()
        : Node("fbkNode")
        {
            if (serialCheckFreq <= 0){
                std::cerr << "错误: 频率必须大于0!" << std::endl;
                std::exit(0);
            }
            // ros2发布者节点，用来发布电机状态
            motorFbk_publisher = this->create_publisher<MotorFeedback>("motorFbk_topic", 10);
            // 三倍于电机反馈频率
            timer = this->create_wall_timer(std::chrono::microseconds(1000000/serialCheckFreq),
                                            std::bind(&fbkNodeClass::timer_callback, this));
        }

    private:
        // ros2发布对象指针
        rclcpp::Publisher<MotorFeedback>::SharedPtr motorFbk_publisher;
        // ros2定时器指针
        rclcpp::TimerBase::SharedPtr timer;
        // 自定义消息
        MotorFeedback motorFbkMsg;
        // 串口反馈字符串
        std::string fbkStr = "";
        // json
        json motorFbkJson;
        void timer_callback()
        // 定时非阻塞读取串口数据；阻塞读取会导致程序难以退出
        // 若成功读取，则发布数据
        // 若超时，则终止并反馈给上层
        {
            static int count = 0;
            bool rst;
            // 调用读取串口方法
            fbkStr = "";
            rst = serialClass::readJsonFromSerial(mySerial.serial, fbkStr);
            if (rst == true)
            {
                //std::cout << "读取到串口数据:\n" << fbkStr << std::endl;
                std::cout << "读取到串口数据" << std::endl;
                //尝试解析串口数据
                try{ 
                    motorFbkJson = json::parse(fbkStr);
                    std::cout << "json解析成功" << std::endl;
                } catch (const json::parse_error& e){
                    count++;
                    RCLCPP_WARN(this->get_logger(), "警告: 无法解析json数据,合计失败次数: %d", count);
                    std::cout << fbkStr << std::endl;
                    return ;
                }
                std::cout << "合计失败次数: " << count << std::endl;
                //将电机id转化为关节名
                std::vector<std::string> joint;
                for (const auto& id : motorFbkJson["id"])
                {
                    auto it = jointIdMap.right.find(id);
                    if (it != jointIdMap.right.end())    joint.push_back(it->second);
                    else                                 joint.push_back(" ");
                }
                //将数据转化为自定义消息并发布
                motorFbkMsg.joint = joint;
                try{
                    motorFbkMsg.pos   = motorFbkJson["ps"].get<std::vector<int32_t>>();
                    motorFbkMsg.vel   = motorFbkJson["vl"].get<std::vector<int32_t>>();
                    motorFbkMsg.tq    = motorFbkJson["tq"].get<std::vector<int32_t>>();
                    motorFbkMsg.err   = motorFbkJson["err"].get<std::vector<int32_t>>();
                }
                catch (const std::exception& e){
                    RCLCPP_ERROR(this->get_logger(), "错误: json 数据转化失败！");
                    std::cout << motorFbkJson << std::endl;
                    return ;
                }
                //发布
                motorFbk_publisher->publish(motorFbkMsg);
            }
            else{}
        }
};

int main(int argc, char * argv[])
{
    // 初始化关节id双向查找表
    biMapInit();
    // ros2初始化并创建收发节点
    rclcpp::init(argc, argv);
    auto driveNode = std::make_shared<driveNodeClass>();
    auto fbkNode   = std::make_shared<fbkNodeClass>();  
    // 创建多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(driveNode);
    executor.add_node(fbkNode);
    // 多线程运行节点
    executor.spin();
    // 结束程序
    executor.cancel();
    rclcpp::shutdown();

    return 0;
}
