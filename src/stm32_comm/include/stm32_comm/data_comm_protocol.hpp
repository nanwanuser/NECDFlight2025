/**
  ******************************************************************************
  * @file    data_comm_protocol.hpp
  * @brief   ROS端数据通信协议库头文件
  * @version V1.0.0
  * @date    2025-07-26
  ******************************************************************************
  */

#ifndef DATA_COMM_PROTOCOL_HPP
#define DATA_COMM_PROTOCOL_HPP

#include <string>
#include <vector>
#include <functional>
#include <cstdint>
#include <memory>

// 协议定义（必须与STM32端一致！）
#define FRAME_HEADER    0xAA55    // 帧头
#define FRAME_END       0x55AA    // 帧尾
#define USE_CRC16       1         // 是否使用CRC校验
#define MAX_DATA_LENGTH 256       // 最大数据长度

// 前向声明
class SerialPort;

/**
 * @brief 数据通信协议类
 */
class DataCommProtocol {
public:
    // 数据包回调函数类型
    using PacketCallback = std::function<void(uint8_t cmd, const std::vector<uint8_t>& data)>;
    
    /**
     * @brief 构造函数
     * @param port 串口设备路径
     * @param baudrate 波特率
     */
    DataCommProtocol(const std::string& port, unsigned int baudrate);
    
    /**
     * @brief 析构函数
     */
    ~DataCommProtocol();
    
    /**
     * @brief 发送数据包
     * @param cmd 命令字节
     * @param data 数据载荷
     * @return 是否发送成功
     */
    bool send(uint8_t cmd, const std::vector<uint8_t>& data = {});
    
    /**
     * @brief 发送数据包（带日志输出）
     * @param cmd 命令字节
     * @param data 数据载荷
     * @param show_packet 是否显示数据包内容
     * @return 是否发送成功
     */
    bool sendWithLog(uint8_t cmd, const std::vector<uint8_t>& data = {}, bool show_packet = true);
    
    /**
     * @brief 获取最后发送的数据包
     * @return 最后发送的数据包内容
     */
    std::vector<uint8_t> getLastSentPacket() const;
    
    /**
     * @brief 设置数据包接收回调
     * @param callback 回调函数
     */
    void setPacketCallback(const PacketCallback& callback);
    
    /**
     * @brief 处理接收数据（非阻塞）
     * @return 处理的字节数
     */
    size_t processReceive();
    
    /**
     * @brief 检查串口是否打开
     * @return 是否打开
     */
    bool isOpen() const;
    
private:
    // 解析状态机状态
    enum ParseState {
        STATE_WAIT_HEADER1,
        STATE_WAIT_HEADER2,
        STATE_WAIT_LENGTH_HIGH,
        STATE_WAIT_LENGTH_LOW,
        STATE_WAIT_CMD,
        STATE_READ_DATA,
        STATE_WAIT_CRC1,
        STATE_WAIT_CRC2,
        STATE_WAIT_END1,
        STATE_WAIT_END2
    };
    
    // 解析上下文
    struct ParseContext {
        ParseState state;
        uint16_t data_index;
        uint16_t pkg_length;
        uint8_t cmd;
        std::vector<uint8_t> data;
        uint16_t recv_crc;
        std::vector<uint8_t> crc_buffer;
        
        ParseContext() : state(STATE_WAIT_HEADER1), data_index(0), 
                        pkg_length(0), cmd(0), recv_crc(0) {
            data.reserve(MAX_DATA_LENGTH);
            crc_buffer.reserve(MAX_DATA_LENGTH + 3);
        }
    };
    
    /**
     * @brief CRC16-CCITT计算
     * @param data 数据缓冲区
     * @param len 数据长度
     * @return CRC16值
     */
    static uint16_t crc16_ccitt(const uint8_t* data, size_t len);
    
    /**
     * @brief 解析接收到的单个字节
     * @param byte 接收到的字节
     */
    void parseByte(uint8_t byte);
    
    /**
     * @brief 重置解析器状态
     */
    void resetParser();
    
private:
    std::unique_ptr<SerialPort> serial_;    // 串口对象
    ParseContext ctx_;                      // 解析上下文
    PacketCallback packet_callback_;        // 数据包回调
    std::vector<uint8_t> last_sent_packet_; // 最后发送的数据包
};

#endif // DATA_COMM_PROTOCOL_HPP