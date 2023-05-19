//
// Created by Quanyi on 2022/12/3.
//
// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <iostream>
#include <unistd.h>          // Unix 标准函数定义
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>           // 文件控制定义
#include <termios.h>         // PPSIX 终端控制定义
#include <string.h>
#include <thread>
#include "../include/serial/serial.h"

using namespace std;
/*************************************************
Function:       SerialPort
Description:    构造函数
Input:
Output:
Return:
Others:         初始化变量 创建接收线程
*************************************************/
SerialPort::SerialPort(const char* id, const int speed, const std::string& name) : Node(name)
{
    RCLCPP_INFO(this -> get_logger(),"This is %s.",name.c_str());
    RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");
    
    devices = id;
    baudrate = speed;
    for (int i = 0; i < COM_BUFF_LEN; i++)
    {
        buff_w_[i] = 0;
        buff_r_[i] = 0;
    }
    // serial_publisher = this->create_publisher<my_interfaces::msg::RobotStatus>("/serial/robotinfo",10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&SerialPort::receive_thread, this));
    this->PortInit();
    data_sub_= this->create_subscription<auto_aim_interfaces::msg::Target>("/tracker/target", rclcpp::SensorDataQoS(),
     std::bind(&SerialPort::data_send, this, std::placeholders::_1));
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));
    latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

    /**
        aiming_point
    */
    aiming_point_.header.frame_id = "odom";
    aiming_point_.ns = "aiming_point";
    aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
    aiming_point_.action = visualization_msgs::msg::Marker::ADD;
    aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
    aiming_point_.color.r = 1.0;
    aiming_point_.color.g = 1.0;
    aiming_point_.color.b = 1.0;
    aiming_point_.color.a = 1.0;
    aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);
    // data_sub_ = this->create_subscription<my_interfaces::msg::Target>(
    //     "/processor/target", rclcpp::SensorDataQoS(),
    // std::bind(&SerialPort::data_send, this, std::placeholders::_1));
}
/*************************************************
Function:       OpenDev
Description:    打开串口
Input:          device
Output:
Return:         fd or -1
Others:         LINUX中 open 函数作用：打开和创建文件
                O_RDONLY 只读打开  O_WRONLY 只写打开  O_RDWR 读，写打开
                对于串口的打开操作，必须使用O_NOCTTY参数，它表示打开的是一个终端设备，程序不会成为该端口的控制终端。如果不使用此标志，任务的一个输入(比如键盘终止信号等)都会影响进程
                O_NDELAY表示不关心DCD信号所处的状态（端口的另一端是否激活或者停止）
                O_NONBLOCK 设置为非阻塞模式，在read时不会阻塞住，在读的时候将read放在while循环中
*************************************************/
int SerialPort::OpenDev(const char *dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);//|O_NONBLOCK);
    if (fd == -1)
        cout << "Open serial error!!!" << endl;
    else
        tcflush(fd, TCIOFLUSH);   // 清空输入输出缓存

    return (fd);
}

/*************************************************
Function:       SetSpeed
Description:    设置波特率
Input:          fd speed
Output:
Return:         true or false
Others:
*************************************************/
unsigned int speed_arr[] = {B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
unsigned int name_arr[] = {115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300};
bool SerialPort::SetSpeed(int fd, int speed)
{
    int status;
    struct termios options;
    tcgetattr(fd, &options);
    for (int i= 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&options, speed_arr[i]);//输入波特率
            cfsetospeed(&options, speed_arr[i]);//输出波特率
            options.c_cflag |= (CLOCAL | CREAD);//本地控制模式 //保证程序不会成为端的占有者//使端口能读取输入的数据
            status = tcsetattr(fd, TCSANOW, &options);//设置串口文件
            if (status != 0)
            {
                cout << "波特率设置失败" << endl;
                return false;
            }
        }
    }
    return true;
}
/*************************************************
Function:       SetParity
Description:    设置串口数据位，效验和停止位
Input:          fd         类型 int  打开的串口文件句柄
                data_bits  类型 int  数据位 取值 为 7 或者 8
                parity     类型 char 效验类型 取值为 N, E, O, S
                stop_bits  类型 int  停止位 取值为 1 或者 2
Output:
Return:
Others:
*************************************************/
bool SerialPort::SetParity(int &fd, int data_bits, char parity, int stop_bits)
{
    struct termios options;
    /*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关错误信息*/
    if (tcgetattr(fd, &options) != 0)
    {
        cout << "串口设置失败" << endl;
        return false;
    }
    /*options.c_cflag &= ~ CSIZE;
    switch (data_bits)
    {
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            cout << "不支持的数据位" << endl;
            return false;
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB; // Clear parity enable
            options.c_iflag &= ~INPCK; // Enable parity checking
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB); // 设置为奇效验
            options.c_iflag |= INPCK; // Disable parity checking
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB; // Enable parity
            options.c_cflag &= ~PARODD; // 转换为偶效验
            options.c_iflag |= INPCK; // Disable parity checking
            break;
        case 's':
        case 'S': // as no parity
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            cout << "不支持的校验类型" << endl;
            return false;
    }
    switch (stop_bits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            cout << "不支持的停止位" << endl;
            return false;
    }
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 设置为原始模式
    options.c_oflag &= ~ OPOST;*/
/*  直接设置
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_cflag = 2237; // 6322-115200 2237-9600
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0; // 等待时间
    options.c_cc[VMIN] = 0; // 最小接收字符
*/  //fcntl(fd,F_SETFL,FNDELAY);
    //直接设置
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    if (tcsetattr(fd, TCSANOW, &options) != 0) // Update the options and do it NOW
    {
        cout << "串口错误" << endl;
        return false;
    }

    cout << options.c_iflag << endl;
    cout << options.c_oflag << endl;
    cout << options.c_cflag << endl;
    cout << options.c_lflag << endl;
    cout << options.c_ispeed << endl;
    cout << options.c_ospeed << endl;
    return true;
}
/*************************************************
Function:       PortInit
Description:    初始化串口
Input:          串口号 波特率
Output:
Return:         true or false
Others:         bool
*************************************************/
bool SerialPort::PortInit()
{
    fd_ = OpenDev(devices);
    if (fd_ == -1){
        return false;
    }

    if (!SetSpeed(fd_, baudrate)){
        return false;
    }

    // 数据位 8  校验 无  停止位 1
    if (!SetParity(fd_, 8, 'N', 1)){
        return false;
    }

    return true;
}
/*************************************************
Function:       Read
Description:    串口读数据
Input:          buff length
Output:
Return:         true or false
Others:         bool；read()默认为阻塞模式，没有读到数据会阻塞住；若在前面设置为非阻塞模式，没有读到数据会返回-1
*************************************************/
int SerialPort::Read(char *r_buff, size_t length)
{
    size_t byte_read = 0;
    byte_read = read(fd_, r_buff, length);

    // if(byte_read == 0){ PortInit(0, 115200);}
    tcflush(fd_, TCIFLUSH); // 清空输入队列  TCIFLUSH 输入队列  TCOFLUSH 输出队列  TCIOFLUSH 输入输出队列
    return byte_read;
}
/*************************************************
Function:       Write
Description:    串口写数据
Input:          buff length
Output:
Return:         true or false
Others:         bool
*************************************************/
bool SerialPort::Write(char *w_buff, size_t length)
{
    //cout << length << endl;
    write(fd_, w_buff, length);
    //tcflush(fd_, TCOFLUSH); // 清空输出队列  TCIFLUSH 输入队列  TCOFLUSH 输出队列  TCIOFLUSH 输入输出队列
    return true;
}
/*************************************************
Function:       ISO14443AAppendCRCA
Description:    CRC16 打包
Input:          buff byte_count
Output:
Return:
Others:
*************************************************/
void SerialPort::ISO14443AAppendCRCA(void* buffer, unsigned short byte_count)
{
    unsigned short check_sum = 0x6363;   //1101111011
    unsigned char* data = (unsigned char*)buffer;

    while (byte_count --)
    {
        unsigned char byte = *data++;

        byte ^= (unsigned char)(check_sum & 0x00ff);
        byte ^= byte << 4;

        check_sum = (check_sum >> 8) ^ ((unsigned short)byte << 8) ^ ((unsigned short)byte << 3) ^((unsigned short)byte >> 4);
    }
    *data++ = (check_sum >> 0) & 0x00ff;
    *data = (check_sum >> 8) & 0x00ff;
}
/*************************************************
Function:       ISO14443ACheckCRCA
Description:    CRC16 解包
Input:          buff byte_count
Output:
Return:
Others:
*************************************************/
unsigned char SerialPort::ISO14443ACheckCRCA(void* buffer, unsigned short byte_count)
{
    unsigned short check_sum = 0x6363;
    unsigned char* data = (unsigned char*)buffer;

    while (byte_count --)
    {
        unsigned char byte = *data++;

        byte ^= (unsigned char)(check_sum & 0x00ff);
        byte ^= byte << 4;

        check_sum = (check_sum >> 8) ^ ((unsigned short)byte << 8) ^ ((unsigned short)byte << 3) ^((unsigned short)byte >> 4);
    }
    //cout << ((data[0] == ((check_sum >> 0) & 0xff)) && (data[1] == ((check_sum >> 8) & 0xff))) << endl;
    return (data[0] == ((check_sum >> 0) & 0xff)) && (data[1] == ((check_sum >> 8) & 0xff));
}
/*************************************************
Function:       ISO14443AAppendCRCA
Description:    CRC16 数据长度检验
Input:          buff
Output:
Return:
Others:
*************************************************/

bool SerialPort::ISO14443ACheckLen(unsigned  char* buffer)
{
    if ((buffer[2] + buffer[3] == 0xff))
        return true;
    else
        return false;
}
/*************************************************
Function:       SendBuff
Description:    发送数据包
Input:          buff length
Output:
Return:         true or false
Others:         bool
*************************************************/
bool SerialPort::SendBuff(char command, char *data, unsigned short length)
{
    unsigned char buffer[COMMAND_BUFF_LEN]; //50
    buffer[0] = 0x55; // 帧头
    buffer[1] = command; // 命令
    buffer[2] = length; // 数据长度
    buffer[3] = 0xff - length; // 数据长度取反
    memcpy(buffer + HEAD_LEN, data, DATA_LEN); // 拷贝数据   从data中复制DATA_LEN个字节到第一个参数
    ISO14443AAppendCRCA(buffer, length + HEAD_LEN); // CRC16 打包

    if (Write((char*)buffer, length + HEAD_LEN + 2))
        return true;
    else
        return false;
}
/*************************************************
Function:       ReceiveBuff
Description:    读取数据包
Input:          src_buff dst_buff
Output:
Return:         true or false
Others:         bool
*************************************************/
int SerialPort::ReceiveBuff()
{
    //NULL;
    //usleep(20000);
    size_t read_length = Read(buff_l_, COM_BUFF_LEN);
     //std::clog<<"llllllllllength "<<read_length<<std::endl;
    if (read_length == 0){return -1;}
    //cout << src_buff[0] <<endl;
    if (buff_l_[0] == 0x55)
    {
        //cout<<"[info ] receive check head success "<<endl;
        if(ISO14443ACheckLen((unsigned char *)(buff_l_))){
            //cout<<"[info ] receive check len success "<<endl;
            if (ISO14443ACheckCRCA(buff_l_, (unsigned short)(buff_l_[2] + HEAD_LEN))){
                //cout<< "[info ] receive check crc success " <<endl;
                //cout << (int8_t *)buff_l_ <<endl;
                
                for (int i = 0; i < DATA_LEN; i++) {
                    buff_r_[i] = buff_l_[i];
                    //cout << buff_l_[i] <<"     "<<i<<endl;
                }
                int16_t buff[4];

                buff[0] = ((uint16_t)buff_r_[5]<<8)|uint8_t (buff_r_[6]);
                buff[1] = ((uint16_t)buff_r_[7]<<8)|uint8_t (buff_r_[8]);

                double pitch_ptz = double(buff[0]) /1000;
                double yaw_ptz = double (buff[1]) /1000;
                //  std::cout<<"pitch"<<pitch_ptz<<std::endl;
                //  std::cout<<"yaw"<<yaw_ptz<<std::endl;
//                buff[2] =  ((uint16_t)buff_r_[9]<<8)| uint8_t (buff_r_[10]);
//                buff[3] = ((uint16_t)buff_r_[11]<<8)| uint8_t (buff_r_[12]);
//                double roll_ptz = double(buff[2])/1000;
//                double speed_d = double(buff[3]);
                // double pitch_w = double(buff[2]) / 1000 ; //角速度
                // double yaw_w = double(buff[3]) / 1000 ;

                /**
                    joint_states
                */
                sensor_msgs::msg::JointState joint_state;
                joint_state.header.stamp = this->now();
                joint_state.name.push_back("pitch_joint");
                joint_state.name.push_back("yaw_joint");
                joint_state.position.push_back(pitch_ptz);
                joint_state.position.push_back(yaw_ptz);
                joint_state_pub_->publish(joint_state);

                /**
                    获取到电控信息后宝宝自己把数据发到marker_pub
                */
                double aim_x = 0.0;
                double aim_y = 0.0;
                double aim_z = 0.0;
                if (aim_x > 0.01) {
                    aiming_point_.header.stamp = this->now();
                    aiming_point_.pose.position.x = aim_x;
                    aiming_point_.pose.position.y = aim_y;
                    aiming_point_.pose.position.z = aim_z;
                    marker_pub_->publish(aiming_point_);
                }


                //std::cout<<"bit8 "<<hex<<uint8_t(dst_buff[8])<<" "<<"bit9"<<hex<<uint8_t(dst_buff[9])<<std::endl;
                // auto robot_status = my_interfaces::msg::RobotStatus();
                // // robot_status = {'a', pitch_ptz, yaw_ptz, double(speed_d)};
                // robot_status.header.stamp = this->get_clock()->now();
                // robot_status.color = buff_r_[4];
                // robot_status.pitch = pitch_ptz;
                // robot_status.yaw = yaw_ptz;
                // robot_status.roll = roll_ptz;
                // robot_status.bullet_speed = speed_d;
                // serial_publisher->publish(robot_status);
                // robotInfo_ = 
//                cout<<"port buff: "
//                    <<" pitch_ptz: "<<pitch_ptz
//                    <<", yaw_ptz: "<<yaw_ptz
//                    <<", speed_d: "<<speed_d
//                    <<", pitch_w: "<<pitch_w  //抬头负，低头正
//                    <<", yaw_w: "<<yaw_w
//                    <<endl;  //顺时针负，逆时针正
                return 1;
            }else{
                //cout<< "[error ] receive check crc error " <<endl;
                for (int i = 0; i < DATA_LEN; i++) {
                    buff_r_[i] = 0;
                }
                return 0;
            }

        }else{
            //cout<<"[error ] receive check len error"<<endl;
            for (int i = 0; i < DATA_LEN; i++) {
                buff_r_[i] = 0;
            }
            return 0;
        }
    }
    else
    {
        //cout<<"[error ] receive check head error "<<endl;
        //cout << "SERIAL error1" << endl;
        for (int i = 0; i < DATA_LEN; i++){
            buff_r_[i] = 0;
            //receive[i] = 0;
        }
        // infor_clear();
        return 0;
    }
}

void SerialPort::receive_thread() {
    // std::cout<<"serial thread create success :"<<std::endl;
    // while(!this->PortInit());
       

    // while(1){
        int status = this->ReceiveBuff();
        //std::cout<<"ssssssssssss"<<status<<std::endl;
        if(status == -1){   //serial offline test
            while(!this->PortInit());
        // }
    }
}

void SerialPort::data_send(const auto_aim_interfaces::msg::Target::SharedPtr data_msg_){
    // double pitch = data_msg_->pitch;
    // double yaw = data_msg_->yaw;
    // double dis = data_msg_->dis;
    //std::cout<<"x:**********************************************"<<data_msg_->position.x<<std::endl;
    std::cout<<"position.x:  "<<data_msg_->position.x<<std::endl;
    char* send_data = new char[6];
    cmd = 0x30;
    if(data_msg_-> tracking) {cmd = 0x31;}
    pitch=0,yaw=0,dis=0,yaw1=0,pitch1=0;
    pitch1=atan(data_msg_->position.z/sqrt(data_msg_->position.x*data_msg_->position.x+data_msg_->position.y*data_msg_->position.y));
    yaw1=atan(data_msg_->position.y/data_msg_->position.x);
    yaw=data_msg_->yaw;
    // pitch=data_msg_->pitch;
    dis=sqrt(data_msg_->position.x*data_msg_->position.x+data_msg_->position.y*data_msg_->position.y+data_msg_->position.z*data_msg_->position.z);
    // std::cout<<"x:"<<data_msg_->position.x<<std::endl;
    // std::cout<<"x1:"<<send_data[0]<<std::endl;
    // std::cout<<"y:"<<data_msg_->position.y<<std::endl;
    // std::cout<<"z:"<<data_msg_->position.z<<std::endl;
    send_data[0] =int16_t (1000*(pitch1));
    send_data[1] =int16_t (1000*(pitch1)) >> 8;
    send_data[2] =int16_t (1000*yaw);
    send_data[3] =int16_t (1000*yaw) >> 8;
    send_data[4] =int16_t (10*dis);
    send_data[5] =int16_t (10*dis) >> 8;
    bool status = this->SendBuff(cmd, send_data, 6);
    delete[] send_data;
    //  std::cout<<"pitch:"<<pitch<<std::endl;
    //std::cout<<"x1:"<<send_data[0]<<std::endl;
    // std::cout<<"yaw:"<<yaw<<std::endl;
    // std::cout<<"yaw1:"<<yaw1<<std::endl;
    // std::cout<<"dis:"<<dis<<std::endl;
    pitch1=0;yaw=0;dis=0;
    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - data_msg_->header.stamp).seconds() * 1000.0;
    RCLCPP_INFO_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialPort>("/dev/ttyACM0", 115200, "serial_node");

    rclcpp::spin(node);
    rclcpp::shutdown();

}
