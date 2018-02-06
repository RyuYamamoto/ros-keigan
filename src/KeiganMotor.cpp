#include "dg_sbr_driver/KeiganMotor.h"

KeiganMotor::KeiganMotor(string portname, int baudrate)
    : port(io_srv)
{
    port.open(portname.c_str());

    port.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
	port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
	port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
	port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
}

KeiganMotor::~KeiganMotor()
{

}

// モータの駆動を許可
void KeiganMotor::enable()
{
    char byte[5];
    
    byte[0] = (unsigned char)0x51;
    byte[1] = (unsigned char)0x00;
    byte[2] = (unsigned char)0x00;
    byte[3] = (unsigned char)0x00;
    byte[4] = (unsigned char)0x00;

    port.write_some(boost::asio::buffer(byte, sizeof(byte)));
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
}

// モータの駆動を不許可
void KeiganMotor::disable()
{
    char byte[5];
    
    byte[0] = (unsigned char)0x50;
    byte[1] = (unsigned char)0x00;
    byte[2] = (unsigned char)0x00;
    byte[3] = (unsigned char)0x00;
    byte[4] = (unsigned char)0x00;

    port.write_some(boost::asio::buffer(byte, sizeof(byte)));
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
}

// 16進数をバイト列に変換
void BytesFromHexString(unsigned char *data, const char *string) {
    int len = (int)strlen(string);
	for (int i=0; i<len; i+=2) {
        unsigned int x;
        sscanf((char *)(string + i), "%02x", &x);
        data[i/2] = x;
    }
}

// モータのスピードを設定
void KeiganMotor::speed(float speed)
{
    char byte[9], str[8];
    unsigned char data[6];
    union {float f; int i;} a;

	a.f = speed;

    // float型を16進数(IEEE754準拠)に変換
	sprintf(str,"%08X",a.i);
    // 16進数をバイト列に変換
    BytesFromHexString(data, str);

    byte[0] = (unsigned char)0x58;
    byte[1] = (unsigned char)0x00;
    byte[2] = (unsigned char)0x01;
    byte[3] = (unsigned char)data[0];
    byte[4] = (unsigned char)data[1];
    byte[5] = (unsigned char)data[2];
    byte[6] = (unsigned char)data[3];
    byte[7] = (unsigned char)0x00;
    byte[8] = (unsigned char)0x00;

    port.write_some(boost::asio::buffer(byte, sizeof(byte)));
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
}

// モータを正回転
void KeiganMotor::runForward()
{
    char byte[5];
    
    byte[0] = (unsigned char)0x60;
    byte[1] = (unsigned char)0x00;
    byte[2] = (unsigned char)0x00;
    byte[3] = (unsigned char)0x00;
    byte[4] = (unsigned char)0x00;

    port.write_some(boost::asio::buffer(byte, sizeof(byte)));
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
}

// モータを逆回転
void KeiganMotor::runBackward()
{
    char byte[5];
    
    byte[0] = (unsigned char)0x61;
    byte[1] = (unsigned char)0x00;
    byte[2] = (unsigned char)0x00;
    byte[3] = (unsigned char)0x00;
    byte[4] = (unsigned char)0x00;

    port.write_some(boost::asio::buffer(byte, sizeof(byte)));
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
}

// モータの動作を停止
void KeiganMotor::stop()
{
    char byte[5];
    
    byte[0] = (unsigned char)0x6D;
    byte[1] = (unsigned char)0x00;
    byte[2] = (unsigned char)0x00;
    byte[3] = (unsigned char)0x00;
    byte[4] = (unsigned char)0x00;

    port.write_some(boost::asio::buffer(byte, sizeof(byte)));
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
}
