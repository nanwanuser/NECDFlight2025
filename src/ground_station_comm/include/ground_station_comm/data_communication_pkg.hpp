/**
  ******************************************************************************
  * @file    data_communication_pkg.hpp
  * @brief   通用数据通信协议库 C++版本
  * @version V2.1.0
  * @date    2025-07-31
  ******************************************************************************
  */

#ifndef DATA_COMMUNICATION_PKG_HPP
#define DATA_COMMUNICATION_PKG_HPP

#include <cstdint>
#include <vector>
#include <functional>

namespace DataComm {

/* 协议参数配置 */
constexpr uint16_t FRAME_HEADER = 0xAA55;      // 帧头
constexpr uint16_t FRAME_END = 0x55AA;         // 帧尾
constexpr uint16_t MAX_DATA_LENGTH = 256;      // 最大数据长度
constexpr bool USE_CRC16 = true;               // 启用CRC16校验

/* 数据包状态枚举 */
enum class PkgStatus {
    OK = 0,
    HEADER_ERR,
    LENGTH_ERR,
    CRC_ERR,
    END_ERR
};

/* 回调函数类型定义 */
using TransmitCallback = std::function<void(const uint8_t*, uint16_t)>;
using PacketCallback = std::function<void(uint8_t, const uint8_t*, uint16_t)>;

/**
 * @brief 数据通信协议类
 */
class DataCommProtocol {
private:
    /* 解析状态机状态定义 */
    enum class ParseState {
        WAIT_HEADER1,       // 等待帧头第1字节
        WAIT_HEADER2,       // 等待帧头第2字节
        WAIT_LENGTH_HIGH,   // 等待长度高字节
        WAIT_LENGTH_LOW,    // 等待长度低字节
        WAIT_CMD,           // 等待命令字节
        READ_DATA,          // 读取数据载荷
        WAIT_CRC1,          // 等待CRC高字节
        WAIT_CRC2,          // 等待CRC低字节
        WAIT_END1,          // 等待帧尾第1字节
        WAIT_END2           // 等待帧尾第2字节
    };

    /* 解析上下文结构体 */
    struct ParseContext {
        ParseState state;                    // 当前解析状态
        uint16_t data_index;                 // 数据索引
        uint16_t pkg_length;                 // 数据包长度（CMD+DATA）
        uint8_t cmd;                         // 命令字节
        std::vector<uint8_t> data;           // 数据缓冲区
        uint16_t recv_crc;                   // 接收到的CRC
        std::vector<uint8_t> crc_buffer;     // CRC计算缓冲区
        uint16_t crc_index;                  // CRC缓冲区索引

        ParseContext() : state(ParseState::WAIT_HEADER1),
                        data_index(0),
                        pkg_length(0),
                        cmd(0),
                        recv_crc(0),
                        crc_index(0) {
            data.reserve(MAX_DATA_LENGTH);
            crc_buffer.reserve(MAX_DATA_LENGTH + 3);
        }
    };

    ParseContext ctx_;
    TransmitCallback transmit_callback_;
    PacketCallback packet_callback_;

    /* 私有函数 */
    uint16_t crc16_ccitt(const uint8_t* data, uint16_t len);

public:
    DataCommProtocol();
    ~DataCommProtocol() = default;

    /* 设置回调函数 */
    void setTransmitCallback(TransmitCallback callback) { transmit_callback_ = callback; }
    void setPacketCallback(PacketCallback callback) { packet_callback_ = callback; }

    /* API函数接口 */
    void init();
    uint16_t send(uint8_t cmd, const uint8_t* data, uint16_t len);
    void parseByte(uint8_t byte);
};

} // namespace DataComm

#endif /* DATA_COMMUNICATION_PKG_HPP */