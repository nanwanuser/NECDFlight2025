/**
  ******************************************************************************
  * @file    data_comm_protocol.cpp
  * @brief   ROS端数据通信协议库实现
  * @version V1.0.0
  * @date    2025-07-26
  ******************************************************************************
  */

#include "stm32_comm/data_comm_protocol.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <ros/ros.h>

/**
 * @brief 串口通信类
 */
class SerialPort {
public:
    SerialPort(const std::string& port, unsigned int baudrate) : fd_(-1) {
        open(port, baudrate);
    }
    
    ~SerialPort() {
        close();
    }
    
    bool open(const std::string& port, unsigned int baudrate) {
        // 打开串口
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ < 0) {
            ROS_ERROR("Failed to open serial port: %s", port.c_str());
            return false;
        }
        
        // 设置非阻塞模式
        fcntl(fd_, F_SETFL, FNDELAY);
        
        // 配置串口参数
        struct termios options;
        tcgetattr(fd_, &options);
        
        // 设置波特率
        speed_t baud = B9600;
        switch(baudrate) {
            case 9600:   baud = B9600;   break;
            case 19200:  baud = B19200;  break;
            case 38400:  baud = B38400;  break;
            case 57600:  baud = B57600;  break;
            case 115200: baud = B115200; break;
            case 230400: baud = B230400; break;
            case 460800: baud = B460800; break;
            default:
                ROS_WARN("Unsupported baudrate %d, using 115200", baudrate);
                baud = B115200;
        }
        
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);
        
        // 8位数据位，无校验，1位停止位
        options.c_cflag &= ~PARENB;  // 无校验
        options.c_cflag &= ~CSTOPB;  // 1位停止位
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;      // 8位数据位
        options.c_cflag |= CLOCAL | CREAD;
        
        // 原始输入模式
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        
        // 设置超时
        options.c_cc[VTIME] = 0;
        options.c_cc[VMIN] = 0;
        
        // 应用设置
        tcflush(fd_, TCIFLUSH);
        tcsetattr(fd_, TCSANOW, &options);
        
        ROS_INFO("Serial port opened: %s @ %d baud", port.c_str(), baudrate);
        return true;
    }
    
    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
    
    bool isOpen() const {
        return fd_ >= 0;
    }
    
    ssize_t write(const uint8_t* data, size_t len) {
        if (fd_ < 0) return -1;
        return ::write(fd_, data, len);
    }
    
    ssize_t read(uint8_t* buffer, size_t max_len) {
        if (fd_ < 0) return -1;
        return ::read(fd_, buffer, max_len);
    }
    
private:
    int fd_;
};

// ========================= DataCommProtocol 实现 =========================

DataCommProtocol::DataCommProtocol(const std::string& port, unsigned int baudrate)
    : serial_(std::make_unique<SerialPort>(port, baudrate)) {
    resetParser();
}

DataCommProtocol::~DataCommProtocol() = default;

bool DataCommProtocol::send(uint8_t cmd, const std::vector<uint8_t>& data) {
    if (!serial_->isOpen()) {
        ROS_ERROR("Serial port not open");
        return false;
    }
    
    // 参数检查
    if (data.size() > MAX_DATA_LENGTH) {
        ROS_ERROR("Data too long: %zu > %d", data.size(), MAX_DATA_LENGTH);
        return false;
    }
    
    std::vector<uint8_t> buffer;
    buffer.reserve(MAX_DATA_LENGTH + 10);
    
    // 1. 帧头（2字节）
    buffer.push_back((FRAME_HEADER >> 8) & 0xFF);
    buffer.push_back(FRAME_HEADER & 0xFF);
    
    // 2. 长度字段（2字节，表示CMD+DATA的总长度）
    uint16_t total_len = data.size() + 1;  // +1 for CMD
    buffer.push_back((total_len >> 8) & 0xFF);
    buffer.push_back(total_len & 0xFF);
    
    // 3. 命令字节（1字节）
    buffer.push_back(cmd);
    
    // 4. 数据载荷
    if (!data.empty()) {
        buffer.insert(buffer.end(), data.begin(), data.end());
    }
    
#if USE_CRC16
    // 5. CRC16校验（2字节）- 从长度字段开始计算
    uint16_t crc = crc16_ccitt(&buffer[2], buffer.size() - 2);
    buffer.push_back((crc >> 8) & 0xFF);
    buffer.push_back(crc & 0xFF);
#endif
    
    // 6. 帧尾（2字节）
    buffer.push_back((FRAME_END >> 8) & 0xFF);
    buffer.push_back(FRAME_END & 0xFF);
    
    // 保存最后发送的数据包
    last_sent_packet_ = buffer;
    
    // 发送数据
    ssize_t written = serial_->write(buffer.data(), buffer.size());
    if (written != static_cast<ssize_t>(buffer.size())) {
        ROS_ERROR("Failed to send complete packet: %zd/%zu", written, buffer.size());
        return false;
    }
    
    return true;
}

bool DataCommProtocol::sendWithLog(uint8_t cmd, const std::vector<uint8_t>& data, bool show_packet) {
    bool result = send(cmd, data);
    
    if (show_packet && !last_sent_packet_.empty()) {
        // 显示实际发送的数据包
        std::stringstream ss;
        ss << std::hex << std::setfill('0');
        for (size_t i = 0; i < last_sent_packet_.size(); ++i) {
            if (i > 0) ss << " ";
            ss << std::setw(2) << static_cast<int>(last_sent_packet_[i]);
        }
        
        ROS_INFO("=== ACTUAL SENT PACKET ===");
        ROS_INFO("Raw bytes [%zu]: %s", last_sent_packet_.size(), ss.str().c_str());
        
        // 解析显示
        if (last_sent_packet_.size() >= 10) {
            ROS_INFO("Header: 0x%02X%02X", last_sent_packet_[0], last_sent_packet_[1]);
            ROS_INFO("Length: %d", (last_sent_packet_[2] << 8) | last_sent_packet_[3]);
            ROS_INFO("Command: 0x%02X", last_sent_packet_[4]);
            
            if (data.size() > 0) {
                std::stringstream data_ss;
                data_ss << std::hex << std::setfill('0');
                for (size_t i = 0; i < data.size(); ++i) {
                    if (i > 0) data_ss << " ";
                    data_ss << std::setw(2) << static_cast<int>(last_sent_packet_[5 + i]);
                }
                ROS_INFO("Data: %s", data_ss.str().c_str());
            }
            
            size_t crc_pos = last_sent_packet_.size() - 4;
            ROS_INFO("CRC: 0x%02X%02X", last_sent_packet_[crc_pos], last_sent_packet_[crc_pos + 1]);
            ROS_INFO("Tail: 0x%02X%02X", last_sent_packet_[crc_pos + 2], last_sent_packet_[crc_pos + 3]);
        }
        ROS_INFO("==========================");
    }
    
    return result;
}

std::vector<uint8_t> DataCommProtocol::getLastSentPacket() const {
    return last_sent_packet_;
}

void DataCommProtocol::setPacketCallback(const PacketCallback& callback) {
    packet_callback_ = callback;
}

size_t DataCommProtocol::processReceive() {
    if (!serial_->isOpen()) {
        return 0;
    }
    
    uint8_t buffer[256];
    ssize_t bytes_read = serial_->read(buffer, sizeof(buffer));
    
    if (bytes_read > 0) {
        for (ssize_t i = 0; i < bytes_read; ++i) {
            parseByte(buffer[i]);
        }
    }
    
    return bytes_read > 0 ? bytes_read : 0;
}

bool DataCommProtocol::isOpen() const {
    return serial_->isOpen();
}

uint16_t DataCommProtocol::crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    
    for (size_t j = 0; j < len; ++j) {
        crc ^= static_cast<uint16_t>(data[j]) << 8;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

void DataCommProtocol::parseByte(uint8_t byte) {
    switch (ctx_.state) {
        case STATE_WAIT_HEADER1:
            if (byte == ((FRAME_HEADER >> 8) & 0xFF)) {
                ctx_.state = STATE_WAIT_HEADER2;
            }
            break;
            
        case STATE_WAIT_HEADER2:
            if (byte == (FRAME_HEADER & 0xFF)) {
                ctx_.state = STATE_WAIT_LENGTH_HIGH;
                ctx_.crc_buffer.clear();
            } else {
                ctx_.state = STATE_WAIT_HEADER1;
            }
            break;
            
        case STATE_WAIT_LENGTH_HIGH:
            ctx_.pkg_length = byte << 8;
            ctx_.crc_buffer.push_back(byte);
            ctx_.state = STATE_WAIT_LENGTH_LOW;
            break;
            
        case STATE_WAIT_LENGTH_LOW:
            ctx_.pkg_length |= byte;
            ctx_.crc_buffer.push_back(byte);
            
            // 长度检查
            if (ctx_.pkg_length == 0 || ctx_.pkg_length > (MAX_DATA_LENGTH + 1)) {
                ROS_WARN("Invalid packet length: %d", ctx_.pkg_length);
                resetParser();
                return;
            }
            ctx_.state = STATE_WAIT_CMD;
            break;
            
        case STATE_WAIT_CMD:
            ctx_.cmd = byte;
            ctx_.crc_buffer.push_back(byte);
            ctx_.data_index = 0;
            ctx_.data.clear();
            
            // 如果只有命令字节，没有数据
            if (ctx_.pkg_length == 1) {
#if USE_CRC16
                ctx_.state = STATE_WAIT_CRC1;
#else
                ctx_.state = STATE_WAIT_END1;
#endif
            } else {
                ctx_.state = STATE_READ_DATA;
            }
            break;
            
        case STATE_READ_DATA:
            ctx_.data.push_back(byte);
            ctx_.crc_buffer.push_back(byte);
            ctx_.data_index++;
            
            // 数据接收完成
            if (ctx_.data_index >= (ctx_.pkg_length - 1)) {
#if USE_CRC16
                ctx_.state = STATE_WAIT_CRC1;
#else
                ctx_.state = STATE_WAIT_END1;
#endif
            }
            break;
            
#if USE_CRC16
        case STATE_WAIT_CRC1:
            ctx_.recv_crc = byte << 8;
            ctx_.state = STATE_WAIT_CRC2;
            break;
            
        case STATE_WAIT_CRC2: {
            ctx_.recv_crc |= byte;
            
            // CRC校验
            uint16_t calc_crc = crc16_ccitt(ctx_.crc_buffer.data(), ctx_.crc_buffer.size());
            if (calc_crc != ctx_.recv_crc) {
                ROS_WARN("CRC error: calc=0x%04X, recv=0x%04X", calc_crc, ctx_.recv_crc);
                resetParser();
                return;
            }
            ctx_.state = STATE_WAIT_END1;
            break;
        }
#endif
            
        case STATE_WAIT_END1:
            if (byte == ((FRAME_END >> 8) & 0xFF)) {
                ctx_.state = STATE_WAIT_END2;
            } else {
                ROS_WARN("Invalid frame end 1: 0x%02X", byte);
                resetParser();
            }
            break;
            
        case STATE_WAIT_END2:
            if (byte == (FRAME_END & 0xFF)) {
                // 完整数据包接收成功
                if (packet_callback_) {
                    packet_callback_(ctx_.cmd, ctx_.data);
                }
                ROS_DEBUG("Packet received: cmd=0x%02X, len=%zu", ctx_.cmd, ctx_.data.size());
            } else {
                ROS_WARN("Invalid frame end 2: 0x%02X", byte);
            }
            resetParser();
            break;
            
        default:
            resetParser();
            break;
    }
}

void DataCommProtocol::resetParser() {
    ctx_.state = STATE_WAIT_HEADER1;
    ctx_.data_index = 0;
    ctx_.pkg_length = 0;
    ctx_.cmd = 0;
    ctx_.recv_crc = 0;
    ctx_.data.clear();
    ctx_.crc_buffer.clear();
}