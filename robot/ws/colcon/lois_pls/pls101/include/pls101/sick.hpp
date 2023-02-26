#ifndef SICK_H
#define SICK_H

#include <fcntl.h>
#include <linux/serial.h>
#include <math.h>
#include <mutex>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <thread>
#include <time.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#define MAXSICKDATA 2048
#define MAXBAUDTRIES 2
#define DIFFBAUDRATES 5
#define DELAYTIME 25000

#define CRC16_GEN_POL 0x8005
#define MKSHORT(a, b) ((unsigned short)(a) | ((unsigned short)(b) << 8))

using namespace std::chrono_literals;

void SICK_Ready();

class Sick : public rclcpp::Node
{
  public:
    Sick();
    ~Sick();
    Sick(const Sick &) = delete;
    Sick(Sick &&) = delete;
    Sick &operator=(const Sick &) = delete;
    Sick &operator=(Sick &&) = delete;
    bool process();
    bool isAvailable();
    sensor_msgs::msg::LaserScan getNextFrame();

  private:

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pubLaser;
    
    std::thread thread;
    bool running;
    std::mutex sickMutex;
    void worker();

    float maxRange;

    unsigned int createCRC(unsigned char *data, unsigned int len);
    void addCRC(char *data, unsigned int len);
    void changeBaud(int baud);
    void changeModePW(char mode, char *password);
    void changeMode(char mode);
    void reset();
    void fillFrame(std::string id, float angle_rad, float maxRange_m, float *data_m, int len);

    int sickFd;

    int foundfirst;
    int sickLen;
    int sickDataLen;
    int sickStatus;
    float sickData[MAXSICKDATA];
    unsigned char crcData[MAXSICKDATA];
    int crcCounter;
    int size;
    char buffer[256];
    unsigned char data;
    int initbaudrate;
    std::string port;
    int baudrate;
    int checkrates[5] = {9600, 19200, 38400, 57600, 115200};
    time_t rtimer, modetimer;
    struct timeval timestamp;
    char timerbuf[26];
    char stringbuf[100];
    unsigned short crcSick, crcCalculated;
    bool sickCrcOk;
    int sickCMD;

    int state;
    int mode;
    int framecounter;
    int i, j, k;

    int searchsick;

    std::string frame;
    float data_m[361];
    sensor_msgs::msg::LaserScan currentFrame;

    bool available;
};
#endif
