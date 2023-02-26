#include <pls101/sick.hpp>

__attribute__((weak)) void SICK_Ready() {}

Sick::Sick()
    : Node("pls_101"), size(0), initbaudrate(9600), rtimer(0), modetimer(0), sickCrcOk(0), state(0),
      mode(0x24), framecounter(0), i(0), j(0), k(0), searchsick(1), foundfirst(0), sickLen(0),
      sickDataLen(0), sickStatus(0), sickCMD(0)
{
    declare_parameter("port", std::string("/dev/ttyUSB0"));
    port = get_parameter("port").as_string();
    declare_parameter("baud", 115200);
    baudrate = get_parameter("baud").as_int();
    declare_parameter("range_m", 10.0);
    maxRange = static_cast<float>(get_parameter("range_m").as_double());
    declare_parameter("frame", std::string("lidar_link"));
    frame = get_parameter("frame").as_string();
    
    pubLaser = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 1);
    
    sickFd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (sickFd == -1)
    {
        RCLCPP_ERROR(get_logger(), "failed to open port %s\n", port.c_str());
        exit(1);
    }
    for (i = 0; i < MAXSICKDATA; i++)
    {
        sickData[i] = 0;
        crcData[i] = 0;
    }

    if (searchsick == 1) initbaudrate = 9600;
    changeBaud(initbaudrate);
    usleep(DELAYTIME);

    if (initbaudrate != baudrate)
        state = 31;
    else
        state = 11;
    if (initbaudrate == 9600) state = 0;
    if (searchsick == 1) state = 20;
    j = 0;
    k = 0;

    time(&modetimer);
    RCLCPP_INFO(get_logger(), "start receiving\n");

    available = false;

    running = true;
    thread = std::thread(&Sick::worker, this);
};

Sick::~Sick()
{
    running = false;
    thread.join();

    close(sickFd);
};

void Sick::worker()
{
    thread.detach();
    while (running)
    {
        process();
    }
}

bool Sick::process()
{
    bool newFrame = false;
    if (state == 0)
    {
        RCLCPP_INFO(get_logger(), "reset sick\n");
        reset();
        time(&modetimer);
        state = 1;
    }
    if (state == 1)
    {
        foundfirst = 0;
        state = 2;
    }
    if (state == 11)
    {
        RCLCPP_INFO(get_logger(), "change sick mode to 0x%02x\n", mode);
        changeMode(mode);
        time(&modetimer);
        foundfirst = 0;
        state = 12;
    }
    if (state == 20)
    {
        if (k >= MAXBAUDTRIES)
        {
            k = 0;
            j++;
            if (j >= DIFFBAUDRATES) j = 0;
            changeBaud(checkrates[j]);
            usleep(DELAYTIME);
        }
        k++;
        RCLCPP_INFO(get_logger(), "search sick: try to reset sick at %i\n", checkrates[j]);
        reset();
        time(&modetimer);
        state = 1;
    }
    if (state == 31)
    {
        time(&modetimer);
        RCLCPP_INFO(get_logger(), "change sick to mode 0x00\n");
        changeModePW(0, "LASERFUN"); // erst in mode 0, dann baudrate aendern
        foundfirst = 0;
        state = 32;
    }
    if (state == 33)
    {
        time(&modetimer);
        RCLCPP_INFO(get_logger(), "change sick to mode (baudrate=%i)\n", baudrate);
        if (baudrate == 38400) changeMode(0x40);
        if (baudrate == 19200) changeMode(0x41);
        if (baudrate == 9600) changeMode(0x42);
        if (baudrate == 57600) changeMode(0x43);  //  58000 baud
        if (baudrate == 115200) changeMode(0x44); // 111111 baud
        if (baudrate == 111111) changeMode(0x44);
        if (baudrate == 200000) changeMode(0x45);
        if (baudrate == 250000) changeMode(0x46);
        if (baudrate == 333333) changeMode(0x47);
        if (baudrate == 500000) changeMode(0x48);
        foundfirst = 0;
        state = 34;
    }
    if ((state > 0) && (state < 40))
    {
        if (difftime(time(NULL), modetimer) > 5)
        {
            RCLCPP_WARN(get_logger(), "mode change timeout!!!\n");
            if (initbaudrate != baudrate)
                state = 31;
            else
                state = 11;
            //                if(initbaudrate==9600) state=0;  // timeout soll versuchen, dass der
            //                mode gewechselt wird
            if (searchsick == 1) state = 20;
        }
    }

    // receiving
    if ((foundfirst > 0) && (difftime(time(NULL), rtimer) > 3))
    {
        RCLCPP_WARN(get_logger(), "Timeout!! sicklen=%i datalen=%i\n", sickLen, crcCounter);
        foundfirst = 0;
    }
    size = read(sickFd, buffer, 1);
    if (size > 0)
    {
        data = static_cast<unsigned char>(buffer[0]);
        if (foundfirst == 4)
        {
            crcData[crcCounter] = data;
            crcCounter++;
            if (crcCounter >= MAXSICKDATA)
                foundfirst =
                    0; // received too much data; start from beginning to prevent buffer overrun
            if ((crcCounter - 4) == sickLen) // all bytes received
            {
                sickCMD = crcData[4];
                sickStatus = crcData[sickLen + 1];
                crcSick = ((static_cast<unsigned short>(crcData[sickLen + 2]) & 0xff)) |
                          ((static_cast<unsigned short>(crcData[sickLen + 3]) & 0x00ff) << 8);
                crcCalculated = createCRC(crcData, crcCounter - 2);
                if (crcSick == crcCalculated)
                    sickCrcOk = 1;
                else
                    sickCrcOk = 0;
                gettimeofday(&timestamp, NULL);
                ctime_r(static_cast<const time_t *>(&timestamp.tv_sec), timerbuf);
                timerbuf[strlen(timerbuf) - 1] = 0;
                if ((sickCMD == 0xb0) && (sickCrcOk == 1))
                {
                    framecounter++;
                    gettimeofday(&timestamp, NULL);
                    sickDataLen = crcData[1 + 4] + 256 * crcData[2 + 4];
                    int dat = 0;
                    for (i = 0; i < sickDataLen; i++)
                    {
                        dat = crcData[i * 2 + 7] + 256 * crcData[i * 2 + 8];
                        dat = dat & 0x3FFF;
                        float meters = static_cast<float>(dat) * 0.01;
                        if (meters > maxRange)
                        {
                            meters = maxRange;
                        }
                        sickData[i] = meters;
                    }
                    fillFrame(frame, 3.14, maxRange, sickData, sickDataLen);
                    newFrame = true;
                }
                else if ((sickCMD == 0x90) && (sickCrcOk == 1))
                {
                    for (i = 1; i < sickLen - 2; i++)
                        stringbuf[i - 1] = crcData[i + 4];
                    RCLCPP_INFO(get_logger(), "Power On: %s \n", stringbuf);
                    if ((state == 2) && (baudrate != initbaudrate))
                        state = 31;
                    else
                        state = 11;
                    searchsick = 0;
                }
                else if ((sickCMD == 0x91) && (sickCrcOk == 1))
                {
                    RCLCPP_INFO(get_logger(), "Software Reset confirm\n");
                }
                else if ((sickCMD == 0xa0) && (sickCrcOk == 1))
                {
                    if (crcData[5] == 0)
                    {
                        if (state == 32)
                            state = 33;
                        else if (state == 34)
                        {
                            changeBaud(baudrate);
                            usleep(DELAYTIME);
                            state = 11;
                        }
                        else
                            state = 40;
                        RCLCPP_INFO(get_logger(), "Mode change performed successfully\n");
                    }
                    if (crcData[5] == 1)
                        RCLCPP_ERROR(get_logger(),
                                     "Mode change not possible - password incorrect\n");
                    if (crcData[5] == 2)
                        RCLCPP_ERROR(get_logger(), "Mode change not possible - LSI FAULT\n");
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "rawlen=%i:\n", sickLen);
                    for (i = 0; i < sickLen; i++)
                        RCLCPP_INFO(get_logger(), "%02x-", crcData[i + 4]);
                    RCLCPP_INFO(get_logger(), "\n");
                }
                foundfirst = 0;
            }
        }
        else if (foundfirst == 3) // Len high byte
        {
            sickLen = sickLen + data * 256 + 2;
            foundfirst = 4;
            crcData[crcCounter] = data;
            crcCounter++;
        }
        else if (foundfirst == 2) // Len low byte
        {
            sickLen = data;
            foundfirst = 3;
            crcData[crcCounter] = data;
            crcCounter++;
        }
        else if ((data == 0x80) && (foundfirst == 1)) // address of subscriber = 0x80
        {
            foundfirst = 2;
            crcData[crcCounter] = data;
            crcCounter++;
        }
        else if (data == 0x02)
        { // start byte
            foundfirst = 1;
            crcCounter = 0;
            crcData[crcCounter] = data;
            crcCounter++;
            time(&rtimer);
        }
        else
        {
            foundfirst = 0;
        }
    }
    else
    {
        usleep(1);
    }
    return newFrame;
}

void Sick::fillFrame(std::string id, float angle_rad, float maxRange_m, float *data_m, int len)
{
    std::lock_guard<std::mutex> lock(sickMutex);

    available = true;
    static uint64_t lastTime = get_clock()->now().nanoseconds();
    uint64_t time = get_clock()->now().nanoseconds();
    currentFrame.header.frame_id = id;
    currentFrame.header.stamp = get_clock()->now();
    currentFrame.angle_min = -0.5 * angle_rad;
    currentFrame.angle_max = 0.5 * angle_rad;
    currentFrame.angle_increment = angle_rad / len;
    currentFrame.time_increment = static_cast<float>((time - lastTime) / len) / 10E9;
    currentFrame.scan_time = static_cast<float>(time - lastTime);
    lastTime = time;
    currentFrame.range_min = 0.0;
    currentFrame.range_max = maxRange_m;
    currentFrame.ranges.resize(len);

    for (int i = 0; i < len; i++)
    {
        currentFrame.ranges[i] = data_m[i];
        float x = cos(-0.5 * M_PI + static_cast<float>(i) * currentFrame.angle_increment) *
                  currentFrame.ranges[i];
        float y = sin(-0.5 * M_PI + static_cast<float>(i) * currentFrame.angle_increment) *
                  currentFrame.ranges[i];
    }
    pubLaser->publish(currentFrame);
    SICK_Ready();
}

sensor_msgs::msg::LaserScan Sick::getNextFrame()
{
    std::lock_guard<std::mutex> lock(sickMutex);
    sensor_msgs::msg::LaserScan frame = currentFrame;

    return frame;
}

bool Sick::isAvailable()
{
    std::lock_guard<std::mutex> lock(sickMutex);
    bool avail = available;
    available = false;

    return avail;
}

unsigned int Sick::createCRC(unsigned char *data, unsigned int len)
{
    unsigned short uCrc16;
    unsigned char abData[2];
    uCrc16 = 0;
    abData[0] = 0;
    while (len--)
    {
        abData[1] = abData[0];
        abData[0] = *data++;
        if (uCrc16 & 0x8000)
        {
            uCrc16 = (uCrc16 & 0x7fff) << 1;
            uCrc16 ^= CRC16_GEN_POL;
        }
        else
        {
            uCrc16 <<= 1;
        }
        uCrc16 ^= MKSHORT(abData[0], abData[1]);
    }
    return (uCrc16);
};

void Sick::addCRC(char *data, unsigned int len)
{
    unsigned short crc;
    crc = createCRC((unsigned char*)data, len);

    data[len] = crc & 0x00ff;
    data[len + 1] = (crc >> 8) & 0x00ff;
};

void Sick::changeBaud(int baud)
{
    struct termios term;
    struct serial_struct serial;

    RCLCPP_INFO(get_logger(), "setTerminalBaud: trying to set baudrate to %i\n", baud);

    /* If seeting baud to 500k */
    if ((baud == 111111) || (baud == 200000) || (baud == 250000) || (baud == 333333) ||
        (baud == 500000))
    {
        /* Get serial attributes */
        if (ioctl(sickFd, TIOCGSERIAL, &serial) < 0)
        {
            RCLCPP_ERROR(get_logger(), "setTerminalBaud: ioctl() failed!\n");
        }

        /* Set the custom devisor */
        serial.flags &= ~ASYNC_SPD_MASK;
        serial.flags |= ASYNC_SPD_CUST;

        /* Set the new attibute values */
        if (ioctl(sickFd, TIOCSSERIAL, &serial) < 0)
        {
            RCLCPP_ERROR(get_logger(), "Sick::setTerminalBaud: ioctl() failed!\n");
        }
    }
    else
    { /* Using a standard baud rate */
        /* We let the next few errors slide in case USB adapter is being used */
        if (ioctl(sickFd, TIOCGSERIAL, &serial) < 0)
        {
            RCLCPP_WARN(get_logger(),
                        "Sick::changeBaud: ioctl() failed while trying to get serial port info!\n");
            RCLCPP_WARN(get_logger(), "NOTE: This is normal when connected via USB!\n");
        }

        serial.custom_divisor = 0;
        serial.flags &= ~ASYNC_SPD_MASK;
        serial.flags &= ~ASYNC_SPD_CUST;

        if (ioctl(sickFd, TIOCSSERIAL, &serial) < 0)
        {
            RCLCPP_WARN(get_logger(),
                        "Sick::changeBaud ioctl() failed while trying to set serial port info!\n");
            RCLCPP_WARN(get_logger(), "\tNOTE: This is normal when connected via USB!\n");
        }
    }

    /* Attempt to acquire device attributes */
    if (tcgetattr(sickFd, &term) < 0)
    {
        RCLCPP_ERROR(get_logger(), "Sick::setTerminalBaud: Unable to get device attributes!\n");
    }

    /* Switch on the baud rate */
    switch (baud)
    {
    case 9600:
    {
        cfmakeraw(&term);
        cfsetispeed(&term, B9600);
        cfsetospeed(&term, B9600);
        break;
    }
    case 19200:
    {
        cfmakeraw(&term);
        cfsetispeed(&term, B19200);
        cfsetospeed(&term, B19200);
        break;
    }
    case 38400:
    {
        cfmakeraw(&term);
        cfsetispeed(&term, B38400);
        cfsetospeed(&term, B38400);
        break;
    }
    case 57600:
    {
        cfmakeraw(&term);
        cfsetispeed(&term, B57600);
        cfsetospeed(&term, B57600);
        break;
    }
    case 115200:
    {
        cfmakeraw(&term);
        cfsetispeed(&term, B115200);
        cfsetospeed(&term, B115200);
        break;
    }
    case 111111:
    case 200000:
    case 250000:
    case 333333:
    case 500000:
    {
        cfmakeraw(&term);
        cfsetispeed(&term, B38400);
        cfsetospeed(&term, B38400);
        break;
    }
    default:
        RCLCPP_ERROR(get_logger(), "Sick::setTerminalBaud: Unknown baud rate!\n");
    }

    term.c_cflag |= (PARENB); // enable even parity

    /* Attempt to set the device attributes */
    if (tcsetattr(sickFd, TCSAFLUSH, &term) < 0)
    {
        RCLCPP_ERROR(get_logger(), "Sick::setTerminalBaud: Unable to set device attributes!\n");
    }

    /* Attempt to flush the I/O buffers */
    if (tcflush(sickFd, TCIOFLUSH) != 0)
    {
        RCLCPP_ERROR(get_logger(), "Sick::setTerminalBaud: tcflush() failed!\n");
    }
};

void Sick::changeModePW(char mode, char *password)
{
    char commdata[20];
    int i;

    commdata[0] = 0x02;
    commdata[1] = 0x00;
    commdata[2] = 0x0a;
    commdata[3] = 0x00;
    commdata[4] = 0x20;
    commdata[5] = mode;

    for (i = 0; i < 8; i++)
        commdata[i + 6] = password[i];
    addCRC(commdata, 14);
    write(sickFd, commdata, 16);
    usleep(DELAYTIME);
};

void Sick::changeMode(char mode)
{
    char commdata[10];

    commdata[0] = 0x02;
    commdata[1] = 0x00;
    commdata[2] = 0x02;
    commdata[3] = 0x00;
    commdata[4] = 0x20;
    commdata[5] = mode;
    addCRC(commdata, 6);
    write(sickFd, commdata, 8);
    usleep(DELAYTIME);
};

void Sick::reset()
{
    char commdata[10];

    commdata[0] = 0x02;
    commdata[1] = 0x00;
    commdata[2] = 0x01;
    commdata[3] = 0x00;
    commdata[4] = 0x10;
    addCRC(commdata, 5);
    write(sickFd, commdata, 7);
    usleep(DELAYTIME);
};
