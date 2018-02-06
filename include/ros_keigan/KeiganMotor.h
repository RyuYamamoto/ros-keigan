#ifndef _KEIGANMOTOR_H_
#define _KEIGANMOTOR_H_

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <boost/asio.hpp>

using namespace std;

class KeiganMotor
{
    public:
        KeiganMotor(string portname, int baudrate);
        ~KeiganMotor();
        void speed(float speed);
        void torque(float torque);
        void enable();
        void disable();
        void runForward();
        void runBackward();
        void stop();
    private:
        boost::asio::io_service io_srv;
        boost::asio::serial_port port;
};

#endif