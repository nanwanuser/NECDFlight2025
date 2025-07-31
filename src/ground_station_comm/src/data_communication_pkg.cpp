/**
  ******************************************************************************
  * @file    data_communication_pkg.cpp
  * @brief   通用数据通信协议库 C++版本实现
  * @version V2.1.0
  * @date    2025-07-31
  ******************************************************************************
  */

#include "ground_station_comm/data_communication_pkg.hpp"
#include <cstring>

namespace DataComm {

/**
 * @brief  构造函数
 */
DataCommProtocol::DataCommProtocol() {
    init();
}

/**
 * @brief  CRC16-CCITT计算
 * @param  data : 数据缓冲区
 * @param  len  : 数据长度
 * @retval CRC16值
 * @note   多项式: 0x1021, 初始值: 0xFFFF
 */
uint16_t DataCommProtocol::crc16_ccitt(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    uint16_t i;

    while (len--) {
        crc ^= static_cast<uint16_t>(*data++) << 8;
        for (i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief  初始化通信协议模块
 */
void DataCommProtocol::init() {
    ctx_ = ParseContext();
}

/**
 * @brief  打包并发送数据
 * @param  cmd  : 命令字节（1字节）
 * @param  data : 数据载荷指针
 * @param  len  : 数据载荷长度（0~MAX_DATA_LENGTH）
 * @retval 实际发送的字节数
 */
uint16_t DataCommProtocol::send(uint8_t cmd, const uint8_t* data, uint16_t len) {
    static uint8_t buffer[MAX_DATA_LENGTH + 10]; // 最大帧长度
    uint16_t index = 0;
    uint16_t total_len;

    /* 参数检查 */
    if (len > MAX_DATA_LENGTH) {
        return 0;
    }

    /* 1. 帧头（2字节） */
    buffer[index++] = (FRAME_HEADER >> 8) & 0xFF;
    buffer[index++] = FRAME_HEADER & 0xFF;

    /* 2. 长度字段（2字节，表示CMD+DATA的总长度） */
    total_len = len + 1;  // +1 for CMD
    buffer[index++] = (total_len >> 8) & 0xFF;
    buffer[index++] = total_len & 0xFF;

    /* 3. 命令字节（1字节） */
    buffer[index++] = cmd;

    /* 4. 数据载荷（len字节） */
    if (data && len > 0) {
        memcpy(&buffer[index], data, len);
        index += len;
    }

    if (USE_CRC16) {
        /* 5. CRC16校验（2字节）- 从长度字段开始计算 */
        uint16_t crc = crc16_ccitt(&buffer[2], index - 2);
        buffer[index++] = (crc >> 8) & 0xFF;
        buffer[index++] = crc & 0xFF;
    }

    /* 6. 帧尾（2字节） */
    buffer[index++] = (FRAME_END >> 8) & 0xFF;
    buffer[index++] = FRAME_END & 0xFF;

    /* 调用用户发送函数 */
    if (transmit_callback_) {
        transmit_callback_(buffer, index);
    }

    return index;
}

/**
 * @brief  解析接收到的单个字节
 * @param  byte : 接收到的字节
 */
void DataCommProtocol::parseByte(uint8_t byte) {
    switch (ctx_.state) {
        case ParseState::WAIT_HEADER1:
            if (byte == ((FRAME_HEADER >> 8) & 0xFF)) {
                ctx_.state = ParseState::WAIT_HEADER2;
            }
            break;

        case ParseState::WAIT_HEADER2:
            if (byte == (FRAME_HEADER & 0xFF)) {
                ctx_.state = ParseState::WAIT_LENGTH_HIGH;
                ctx_.crc_buffer.clear();
                ctx_.crc_index = 0;
            } else {
                ctx_.state = ParseState::WAIT_HEADER1;
            }
            break;

        case ParseState::WAIT_LENGTH_HIGH:
            ctx_.pkg_length = byte << 8;
            ctx_.crc_buffer.push_back(byte);
            ctx_.crc_index++;
            ctx_.state = ParseState::WAIT_LENGTH_LOW;
            break;

        case ParseState::WAIT_LENGTH_LOW:
            ctx_.pkg_length |= byte;
            ctx_.crc_buffer.push_back(byte);
            ctx_.crc_index++;

            /* 长度检查 */
            if (ctx_.pkg_length == 0 || ctx_.pkg_length > (MAX_DATA_LENGTH + 1)) {
                ctx_.state = ParseState::WAIT_HEADER1;
                return;
            }
            ctx_.state = ParseState::WAIT_CMD;
            break;

        case ParseState::WAIT_CMD:
            ctx_.cmd = byte;
            ctx_.crc_buffer.push_back(byte);
            ctx_.crc_index++;
            ctx_.data_index = 0;
            ctx_.data.clear();

            /* 如果只有命令字节，没有数据 */
            if (ctx_.pkg_length == 1) {
                ctx_.state = USE_CRC16 ? ParseState::WAIT_CRC1 : ParseState::WAIT_END1;
            } else {
                ctx_.state = ParseState::READ_DATA;
            }
            break;

        case ParseState::READ_DATA:
            ctx_.data.push_back(byte);
            ctx_.crc_buffer.push_back(byte);
            ctx_.crc_index++;
            ctx_.data_index++;

            /* 数据接收完成 */
            if (ctx_.data_index >= (ctx_.pkg_length - 1)) {
                ctx_.state = USE_CRC16 ? ParseState::WAIT_CRC1 : ParseState::WAIT_END1;
            }
            break;

        case ParseState::WAIT_CRC1:
            if (USE_CRC16) {
                ctx_.recv_crc = byte << 8;
                ctx_.state = ParseState::WAIT_CRC2;
            }
            break;

        case ParseState::WAIT_CRC2:
            if (USE_CRC16) {
                ctx_.recv_crc |= byte;

                /* CRC校验 */
                uint16_t calc_crc = crc16_ccitt(ctx_.crc_buffer.data(), ctx_.crc_index);
                if (calc_crc != ctx_.recv_crc) {
                    /* CRC错误，重新开始 */
                    ctx_.state = ParseState::WAIT_HEADER1;
                    return;
                }
            }
            ctx_.state = ParseState::WAIT_END1;
            break;

        case ParseState::WAIT_END1:
            if (byte == ((FRAME_END >> 8) & 0xFF)) {
                ctx_.state = ParseState::WAIT_END2;
            } else {
                ctx_.state = ParseState::WAIT_HEADER1;
            }
            break;

        case ParseState::WAIT_END2:
            if (byte == (FRAME_END & 0xFF)) {
                /* 完整数据包接收成功，调用用户处理函数 */
                if (packet_callback_) {
                    packet_callback_(ctx_.cmd, ctx_.data.data(), ctx_.data.size());
                }
            }
            ctx_.state = ParseState::WAIT_HEADER1;
            break;

        default:
            ctx_.state = ParseState::WAIT_HEADER1;
            break;
    }
}

} // namespace DataComm