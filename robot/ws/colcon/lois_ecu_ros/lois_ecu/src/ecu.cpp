#include <lois_ecu/ecu.hpp>

ECU::ECU() : Node("lois_ecu"), running(true), emergencyStop(false)
{
    declare_parameter("port", std::string("/dev/ttyUSB0"));
    port = get_parameter("port").as_string();
    declare_parameter("baud", 115200);
    baud = get_parameter("baud").as_int();
    declare_parameter("wheel_width_m", 0.5);
    wheelWidth = static_cast<float>(get_parameter("wheel_width_m").as_double());
    declare_parameter("wheel_radius_m", 0.125);
    wheelRadius = static_cast<float>(get_parameter("wheel_radius_m").as_double());
    declare_parameter("encoder_steps", 60);
    encoderSteps = get_parameter("encoder_steps").as_int();
    declare_parameter("configfile", std::string("runtimeparameters.ini"));
    runtimeParametersIni = get_parameter("configfile").as_string();
    declare_parameter("odom_parent", std::string("odom"));
    odomParent = get_parameter("odom_parent").as_string();
    declare_parameter("odom_child", std::string("base_link"));
    odomChild = get_parameter("odom_child").as_string();
    pubRPMRecord = create_publisher<std_msgs::msg::UInt16MultiArray>("/rpm_record", 1);
    pubTorqueRecord = create_publisher<std_msgs::msg::UInt16MultiArray>("/torque_record", 1);
    pubHeadingRecord = create_publisher<std_msgs::msg::Int16MultiArray>("/heading_record", 1);
    pubEmergencyStop =
        create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/emergency_stop", 1);
    pubKVH = create_publisher<std_msgs::msg::Int32>("/kvh_heading", 1);
    pubGPS = create_publisher<sensor_msgs::msg::NavSatFix>("/gps_data", 1);
    pubOdometry = create_publisher<nav_msgs::msg::Odometry>("/odom_wheel", 1);
    pubRuntimeParameters = create_publisher<lois_ecu::msg::RuntimeParameters>("/ecu_params", 1);

    readRuntimeParametersFromIni();
    publishRuntimeParameters();

    fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        RCLCPP_ERROR(get_logger(), "failed to open port %s\n", port.c_str());
        exit(1);
    }
    RCLCPP_INFO(get_logger(), "set up ECU connection on port %s\n", port.c_str());

    tcgetattr(fd, &term);
    cfsetispeed(&term, baud);
    term.c_cflag = 0x100fdab2;
    term.c_iflag = 0x04;
    term.c_lflag = 0;
    term.c_oflag = 0;
    tcflush[fd, TCIFLUSH];
    tcsetattr(fd, TCSANOW, &term);

    odometryData.x = 0.0;
    odometryData.y = 0.0;
    odometryData.o = 0.0;
    odometryData.dx = 0.0;
    odometryData.dy = 0.0;
    odometryData.w = 0.0;
    odometryData.dt = 0.0;
    odometryData.leftTicks = 0;
    odometryData.rightTicks = 0;

    // Perform hardware initialization
    setRuntimeParameters(runtimeParameters);
    enableCtrlRPM(true);
    reset();
    setLED(1);

    // Start communication thread
    thread = std::thread(&ECU::worker, this);

    subCmdVel = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1, std::bind(&ECU::setTwist, this, std::placeholders::_1));
    subEnableCtrlRPM = create_subscription<std_msgs::msg::Bool>(
        "/enable_rpmctrl", 1, [&](const std_msgs::msg::Bool msg) { enableCtrlRPM(msg.data); });
    subRequestRPM = create_subscription<std_msgs::msg::UInt16MultiArray>(
        "/rpm_request", 1, [&](const std_msgs::msg::UInt16MultiArray msg) {
            requestRPM(msg.data[0], msg.data[1], false);
        });
    subRequestTorque = create_subscription<std_msgs::msg::UInt16MultiArray>(
        "/torque_request", 1, [&](const std_msgs::msg::UInt16MultiArray msg) {
            requestRPM(msg.data[0], msg.data[1], true);
        });
    subRequestHeading = create_subscription<std_msgs::msg::UInt32>(
        "/heading_request", 1, [&](const std_msgs::msg::UInt32 msg) { requestHeading(msg.data); });
    subShit = create_subscription<std_msgs::msg::UInt32>(
        "/shit_ball", 1, [&](const std_msgs::msg::UInt32 msg) { shitBall(); });
    subRequestParams = create_subscription<std_msgs::msg::Bool>(
        "/ecu_params_request", 1, [&](const std_msgs::msg::Bool msg) { publishRuntimeParameters(); });
    subSetParams = create_subscription<lois_ecu::msg::RuntimeParameters>(
        "/ecu_params", 1, [&](const lois_ecu::msg::RuntimeParameters params) {
            ECU::RuntimeParameters_t dat;
            dat.terminalMode = params.terminal_mode.data;
            dat.rpmctrlEnable = params.rpmctrl_enable.data;
            dat.rpmLowpass = params.rpm_lowpass.data;
            dat.kaLeft = params.ka_left.data;
            dat.kpLeft = params.kp_left.data;
            dat.tnLeft = params.tn_left.data;
            dat.tdLeft = params.td_left.data;
            dat.kaRight = params.ka_right.data;
            dat.kpRight = params.kp_right.data;
            dat.tnRight = params.tn_right.data;
            dat.tdRight = params.td_right.data;
            dat.corrLongLeft = params.corr_long_left.data;
            dat.corrShortLeft = params.corr_short_left.data;
            dat.corrLongRight = params.corr_long_right.data;
            dat.corrShortRight = params.corr_short_right.data;
            dat.periodLatLon = params.period_latlon.data;
            dat.periodTime = params.period_time.data;
            dat.periodDate = params.period_date.data;
            dat.periodHeading = params.period_heading.data;
            dat.periodEncoders = params.period_encoders.data;
            dat.periodOdometry = params.period_odometry.data;
            setRuntimeParameters(dat, params.save.data);
        });
}

ECU::~ECU()
{
    // Stop communication thread
    running = false;
    thread.join();
    // Close serial connection
    close(fd);
}

void ECU::worker()
{
    thread.detach();
    while (running)
    {
        // Periodically receive and process messages
        receive();
        processMessages();
    }
}

void ECU::send(uint8_t instruction, uint16_t val1, uint16_t val2)
{
    char cmd[20];
    char cmd2[20];
    // Formatting
    int len = sprintf(cmd, ":%02x%04x%04x", instruction, val1, val2);
    len = sprintf(cmd2, ";%s%02x\n", cmd, calculateCRC(cmd));
    // Write to kernel module
    write(fd, cmd2, len);
    // Delay
    usleep(MESSAGE_TIMEOUT_MS * 1000);
}

void ECU::send(uint8_t instruction, uint32_t val)
{
    send(instruction, static_cast<uint16_t>(val >> 16), static_cast<uint16_t>(val & 0xFFFF));
}

void ECU::send(uint8_t instruction, float val)
{
    uint32_t v32 = 0;
    memcpy(&v32, &val, sizeof(val));
    send(instruction, v32);
}

void ECU::receive()
{
    char buf[2 + MESSAGE_INSTRUCTIONBYTES * 2 + MESSAGE_DATABYTES * 2 + 2 + 1];
    read(fd, buf, 1);
    if (':' == buf[0]) // Wait for start
    {
        read(fd, &buf[1],
             1 + MESSAGE_INSTRUCTIONBYTES * 2 + MESSAGE_DATABYTES * 2 + 2); // Read whole message
        buf[2 + MESSAGE_INSTRUCTIONBYTES * 2 + MESSAGE_DATABYTES * 2 + 2] = '\0';
        char inStr[MESSAGE_INSTRUCTIONBYTES * 2 + 1];
        char datStr[MESSAGE_DATABYTES * 2 + 1];
        char csStr[3];
        inStr[MESSAGE_INSTRUCTIONBYTES * 2] = '\0';
        datStr[MESSAGE_DATABYTES * 2] = '\0';
        csStr[2] = '\0';
        InterfaceData_t dat;
        // Decode instruction byte
        memcpy(inStr, &buf[1], MESSAGE_INSTRUCTIONBYTES * 2 * sizeof(char));
        dat.instruction = strtol(inStr, NULL, 16);
        // Decode payload 2 x 16-bit
        memcpy(datStr, &buf[1 + MESSAGE_INSTRUCTIONBYTES * 2], MESSAGE_DATABYTES * sizeof(char));
        dat.val1 = strtol(datStr, NULL, 16);
        memcpy(datStr, &buf[1 + MESSAGE_INSTRUCTIONBYTES * 2 + MESSAGE_DATABYTES],
               MESSAGE_DATABYTES * sizeof(char));
        dat.val2 = strtol(datStr, NULL, 16);
        // Decode CRC byte
        memcpy(csStr, &buf[1 + MESSAGE_INSTRUCTIONBYTES * 2 + MESSAGE_DATABYTES * 2],
               2 * sizeof(char));
        int cs = strtol(csStr, NULL, 16);
        // Do CRC check
        dat.csOK = (calculateCRC(buf) == cs);
        if (!dat.csOK)
        {
            RCLCPP_WARN(get_logger(), "CS faulty\n");
        }
        // Push to message queue
        messages.push(dat);
        buf[0] = ' ';
    }
}

void ECU::processMessages()
{
    static int i = 0;
    // Pop message from queue and distribute them
    while (!messages.empty())
    {
        InterfaceData_t dat = messages.front();
        messages.pop();
        switch (dat.instruction)
        {
        case Commands::RECORDRPM:
            addRPMMeasurement(&dat);
            break;
        case Commands::EMSTOP:
            emergencyStop = dat.val2;
            callback_EmergencyStop(isEmergencyStop());
            break;
        case Commands::RECORDHEADING:
            addHeadingMeasurement(&dat);
            break;
        case Commands::PERIODIC_HEADING:
            if (dat.csOK)
            {
                kvhHeading = (int)((short)dat.val2);
                callback_Heading(getKVHHeading());
            }
            break;
        case Commands::PERIODIC_LATLON:
            addGPSData(&dat);
            break;
        case Commands::PERIODIC_ENCODERS:
            addOdometryData(&dat);
            break;
        case Commands::PERIODIC_ODOM:
            addOdometryDataECU(&dat);
            break;
        default:
            break;
        }
    }
}

void ECU::addRPMMeasurement(InterfaceData_t *dat)
{
    // Push rpm measurements to queue
    uint16_t left = dat->val1;
    uint16_t right = dat->val2;
    if (0 != left || 0 != right) // Messages end with 0 0
    {
        std::lock_guard<std::mutex> lock(mu);
        rpmRecord.left.push(left);
        rpmRecord.right.push(right);
    }
    else
    {
        // Check for torque or RPM and do callback
        if (torque)
        {
            RCLCPP_INFO(get_logger(), "New Torque record available (left: %d, right: %d)",
                        rpmRecord.left.size(), rpmRecord.right.size());
            callback_TorqueRecord(getTorqueRecord());
        }
        else
        {
            RCLCPP_INFO(get_logger(), "New RPM record available (left: %d, right: %d)",
                        rpmRecord.left.size(), rpmRecord.right.size());
            callback_RPMRecord(getRPMRecord());
        }
    }
}

void ECU::addHeadingMeasurement(InterfaceData_t *dat)
{
    uint16_t val1 = dat->val1;
    uint16_t val2 = dat->val2;
    static short last = 0;
    short h1 = static_cast<short>(val1);
    short h2 = static_cast<short>(val2);
    std::lock_guard<std::mutex> lock(mu);
    // Messages end with 362
    if (362 != h1)
    {
        headingRecord.heading.push(h1);
    }
    if (362 != h2)
    {
        headingRecord.heading.push(h2);
    }

    if (362 == h1 && 362 == h2)
    {
        RCLCPP_INFO(get_logger(), "New heading record available (measugements: %d)",
                    headingRecord.heading.size());
        callback_HeadingRecord(getHeadingRecord());
    }
}

void ECU::addGPSData(InterfaceData_t *dat)
{
    static int cnt = 0;
    if (!dat->csOK)
    {
        cnt = 1000;
        return;
    }
    uint16_t val1 = dat->val1;
    uint16_t val2 = dat->val2;
    if (0xffff == val1 && 0xffff == val2) // GPS data transfer ends with 0xffff0xffff payload
    {
        cnt = 0;
    }
    else if (1 == cnt) // latituzde
    {
        gpsData.latitude = float_2x16(val1, val2);
    }
    else if (2 == cnt) // longitude
    {
        gpsData.longitude = float_2x16(val1, val2);
        callback_GPSData(getGPSData());
    }
    cnt++;
}

void ECU::addOdometryData(InterfaceData_t *dat)
{
    static int cnt = 0;
    static uint32_t lastTime = 0;
    static int lastLeft = 0;
    static int lastRight = 0;

    static uint32_t time;
    static int left, right;

    if (!dat->csOK)
    {
        cnt = 1000;
        return;
    }
    uint16_t val1 = dat->val1;
    uint16_t val2 = dat->val2;

    if (0xffff == val1 && 0xffff == val2)
    {
        cnt = 0;
    }
    else if (1 == cnt) // time_ms
    {
        time = uint32_2x16(val1, val2);
    }
    else if (2 == cnt) // left
    {
        left = static_cast<int>(uint32_2x16(val1, val2));
    }
    else if (3 == cnt) // right
    {
        std::lock_guard<std::mutex> lock(mu);
        right = (int)uint32_2x16(val1, val2);
        lastTime = odometryData.time;
        lastLeft = odometryData.leftTicks;
        lastRight = odometryData.rightTicks;
        odometryData.time = time;
        odometryData.dt = time - lastTime;
        odometryData.leftTicks = left;
        odometryData.rightTicks = right;
        int dLeft = odometryData.leftTicks - lastLeft;
        int dRight = odometryData.rightTicks - lastRight;
        float sLeft = (static_cast<float>(dLeft) * 2.0 * M_PI * wheelRadius /
                       static_cast<float>(encoderSteps));
        float sRight = (static_cast<float>(dRight) * 2.0 * M_PI * wheelRadius /
                        static_cast<float>(encoderSteps));
        float vLeft = sLeft / static_cast<float>(odometryData.dt);
        float vRight = sRight / static_cast<float>(odometryData.dt);

        float R = 0;
        if (dLeft == dRight) // axial rotation
        {
            R = 0.5 * wheelWidth;
        }
        else
        {
            R = 0.5 * wheelWidth * (vLeft + vRight) / (vRight - vLeft);
        }
        float w = (vRight - vLeft) / wheelWidth;
        float ICCx = odometryData.x - R * sin(odometryData.o);
        float ICCy = odometryData.y + R * cos(odometryData.o);
        float x = odometryData.x;
        float y = odometryData.y;
        float o = odometryData.o;
        float dt = odometryData.dt;
        odometryData.x = cos(w * dt) * (x - ICCx) - sin(w * dt) * (y - ICCy) + ICCx;
        odometryData.y = sin(w * dt) * (x - ICCx) + cos(w * dt) * (y - ICCy) + ICCy;
        odometryData.o = odometryData.o + w * dt;
        odometryData.dx = (odometryData.x - x) / dt;
        odometryData.dy = (odometryData.y - y) / dt;
        odometryData.w = (odometryData.o - o) / dt;
        // callback_Odometry(getOdometry());
    }
    cnt++;
}

void ECU::addOdometryDataECU(InterfaceData_t *dat)
{
    static int cnt = 0;
    std::lock_guard<std::mutex> lock(mu);
    if (!dat->csOK)
    {
        cnt = 1000;
        return;
    }
    uint16_t val1 = dat->val1;
    uint16_t val2 = dat->val2;
    if (0xffff == val1 && 0xffff == val2) // GPS data transfer ends with 0xffff0xffff payload
    {
        cnt = 0;
    }
    else if (1 == cnt) // x
    {
        odometryData.x = float_2x16(val1, val2);
    }
    else if (2 == cnt) // y
    {
        odometryData.y = float_2x16(val1, val2);
    }
    else if (3 == cnt) // o
    {
        odometryData.o = float_2x16(val1, val2);
    }
    else if (4 == cnt) // dx
    {
        odometryData.dx = float_2x16(val1, val2);
    }
    else if (5 == cnt) // dy
    {
        odometryData.dy = float_2x16(val1, val2);
    }
    else if (6 == cnt) // w
    {
        odometryData.w = float_2x16(val1, val2);
        callback_Odometry(getOdometry());
    }
    cnt++;
}

ECU::RPMData_t ECU::getRPMRecord()
{
    std::lock_guard<std::mutex> lock(mu);
    RPMData_t dat = rpmRecord;

    return dat;
}

ECU::RPMData_t ECU::getTorqueRecord() { return getRPMRecord(); }

void ECU::setDutycycle(short left, short right) { send(Commands::DUTYCYCLE, left, right); }

void ECU::enableCtrlRPM(bool enable)
{
    RCLCPP_INFO(get_logger(), "Enable RPM: %i", enable);
    isCtrlRPM = enable;
}

void ECU::setRPM(short left, short right)
{
    static short lastLeft = 0;
    static short lastRight = 0;
    if ((lastLeft < 0 && left > 0) || (lastLeft > 0 && left < 0) || (lastRight < 0 && right > 0) ||
        (lastRight > 0 && right < 0)) // Send rpm commands only on change
    {
        send(Commands::RPM, 0, 0);
    }
    send(Commands::RPM, left, right);
    lastLeft = left;
    lastRight = right;
}

void ECU::setLED(bool on) { send(Commands::LED, 0, on ? 1 : 0); }

void ECU::shitBall() { send(Commands::SHIT, 0, 0); }

void ECU::requestRPM(uint16_t left, uint16_t right, bool torque)
{
    std::lock_guard<std::mutex> lock(mu);
    this->torque = torque;
    // Free data queues
    while (!rpmRecord.left.empty())
    {
        rpmRecord.left.pop();
    }
    while (!rpmRecord.right.empty())
    {
        rpmRecord.right.pop();
    }

    if (torque)
    {
        RCLCPP_INFO(get_logger(), "Requesting Torque record: left: %d, right: %d", left, right);
        send(Commands::RECORDTORQUE, left, right);
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Requesting RPM record: left: %d, right: %d", left, right);
        send(Commands::RECORDRPM, left, right);
    }
}

void ECU::setTwist(const geometry_msgs::msg::Twist &msg)
{
    float vX = msg.linear.x;
    float w = msg.angular.z;
    if (isCtrlRPM)
    {
        float deltaV = (w * wheelWidth) / 2.0;
        float vL = vX + deltaV;
        float vR = vX - deltaV;

        float nL = (vL * 60.0) / (2 * M_PI * wheelRadius);
        float nR = (vR * 60.0) / (2 * M_PI * wheelRadius);

        if (vX < 0)
        {
            float tmp = nL;
            nL = nR;
            nR = tmp;
        }
        RCLCPP_INFO(get_logger(), "Controlled Twist, vX: %.3f, w: %.3f, %i, %i", vX, w,
                    static_cast<short>(nL), static_cast<short>(nR));
        setRPM(static_cast<short>(nL), static_cast<short>(nR));
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Dutycycle Twist, left: %d, right: %d",
                    static_cast<short>(vX + w), static_cast<short>(vX - w));
        setDutycycle(static_cast<short>(vX + w), static_cast<short>(vX - w));
    }
}

void ECU::setPILeft(float Ka, float Kp, float Tn, float Td)
{
    send(Commands::LEFT_KA, Ka);
    send(Commands::LEFT_KP, Kp);
    send(Commands::LEFT_TN, Tn);
    send(Commands::LEFT_TD, Td);
}

void ECU::setPIRight(float Ka, float Kp, float Tn, float Td)
{
    send(Commands::RIGHT_KA, Ka);
    send(Commands::RIGHT_KP, Kp);
    send(Commands::RIGHT_TN, Tn);
    send(Commands::RIGHT_TD, Td);
}

void ECU::setRPMLowpass(float coeff)
{
    send(Commands::RPM_LOWPASS, coeff);
}

void ECU::requestHeading(uint32_t count)
{
    RCLCPP_INFO(get_logger(), "Requesting heading record (measurements: %d)", count);
    std::lock_guard<std::mutex> lock(mu);
    while (!headingRecord.heading.empty())
    {
        headingRecord.heading.pop();
    }

    send(Commands::RECORDHEADING, count);
}

void ECU::reset() { send(Commands::RESET, (uint32_t)0); }

ECU::HeadingData_t ECU::getHeadingRecord()
{
    std::lock_guard<std::mutex> lock(mu);
    ECU::HeadingData_t dat = headingRecord;

    return dat;
}

void ECU::readRuntimeParametersFromIni()
{
    inipp::Ini<char> ini;
    std::ifstream is(runtimeParametersIni.c_str());
    ini.parse(is);

    inipp::get_value(ini.sections["UART"], "TERMINALMODE", runtimeParameters.terminalMode);

    inipp::get_value(ini.sections["RPMCTRL"], "RPMCTRLENABLE", runtimeParameters.rpmctrlEnable);
    inipp::get_value(ini.sections["RPMCTRL"], "LOWPASS", runtimeParameters.rpmLowpass);
    inipp::get_value(ini.sections["RPMCTRL"], "KALEFT", runtimeParameters.kaLeft);
    inipp::get_value(ini.sections["RPMCTRL"], "KPLEFT", runtimeParameters.kpLeft);
    inipp::get_value(ini.sections["RPMCTRL"], "TNLEFT", runtimeParameters.tnLeft);
    inipp::get_value(ini.sections["RPMCTRL"], "TDLEFT", runtimeParameters.tdLeft);
    inipp::get_value(ini.sections["RPMCTRL"], "KARIGHT", runtimeParameters.kaRight);
    inipp::get_value(ini.sections["RPMCTRL"], "KPRIGHT", runtimeParameters.kpRight);
    inipp::get_value(ini.sections["RPMCTRL"], "TNRIGHT", runtimeParameters.tnRight);
    inipp::get_value(ini.sections["RPMCTRL"], "TDRIGHT", runtimeParameters.tdRight);
    inipp::get_value(ini.sections["RPMCTRL"], "CORRLEFTSHORT", runtimeParameters.corrShortLeft);
    inipp::get_value(ini.sections["RPMCTRL"], "CORRLEFTLONG", runtimeParameters.corrLongLeft);
    inipp::get_value(ini.sections["RPMCTRL"], "CORRRIGHTSHORT", runtimeParameters.corrShortRight);
    inipp::get_value(ini.sections["RPMCTRL"], "CORRRIGHTLONG", runtimeParameters.corrLongRight);

    inipp::get_value(ini.sections["PERIODS"], "LATLON", runtimeParameters.periodLatLon);
    inipp::get_value(ini.sections["PERIODS"], "TIME", runtimeParameters.periodTime);
    inipp::get_value(ini.sections["PERIODS"], "DATE", runtimeParameters.periodDate);
    inipp::get_value(ini.sections["PERIODS"], "HEADING", runtimeParameters.periodHeading);
    inipp::get_value(ini.sections["PERIODS"], "ENCODERS", runtimeParameters.periodEncoders);
    inipp::get_value(ini.sections["PERIODS"], "ODOMETRY", runtimeParameters.periodOdometry);

    RCLCPP_INFO(get_logger(), "Runtime Parameters\n%s\n", serializeRuntimeParameters().c_str());
    writeRuntimeParametersToIni();
}

std::string ECU::serializeRuntimeParameters()
{
    std::string params = "";
    params += "[UART]\n";
    params += "TERMINALMODE = " + std::to_string(runtimeParameters.terminalMode) + "\n";

    params += "\n[RPMCTRL]\n";
    params += "RPMCTRLENABLE = " + std::to_string((int)runtimeParameters.rpmctrlEnable) + "\n";
    params += "LOWPASS = " + std::to_string(runtimeParameters.rpmLowpass) + "\n";
    params += "KALEFT = " + std::to_string(runtimeParameters.kaLeft) + "\n";
    params += "KPLEFT = " + std::to_string(runtimeParameters.kpLeft) + "\n";
    params += "TNLEFT = " + std::to_string(runtimeParameters.tnLeft) + "\n";
    params += "TDLEFT = " + std::to_string(runtimeParameters.tdLeft) + "\n";
    params += "KARIGHT = " + std::to_string(runtimeParameters.kaRight) + "\n";
    params += "KPRIGHT = " + std::to_string(runtimeParameters.kpRight) + "\n";
    params += "TNRIGHT = " + std::to_string(runtimeParameters.tnRight) + "\n";
    params += "TDRIGHT = " + std::to_string(runtimeParameters.tdRight) + "\n";
    params += "CORRLEFTSHORT = " + std::to_string(runtimeParameters.corrShortLeft) + "\n";
    params += "CORRLEFTLONG = " + std::to_string(runtimeParameters.corrLongLeft) + "\n";
    params += "CORRRIGHTSHORT = " + std::to_string(runtimeParameters.corrShortRight) + "\n";
    params += "CORRRIGHTLONG = " + std::to_string(runtimeParameters.corrLongRight) + "\n";

    params += "\n[PERIODS]\n";
    params += "LATLON = " + std::to_string(runtimeParameters.periodLatLon) + "\n";
    params += "TIME = " + std::to_string(runtimeParameters.periodTime) + "\n";
    params += "DATE = " + std::to_string(runtimeParameters.periodDate) + "\n";
    params += "HEADING = " + std::to_string(runtimeParameters.periodHeading) + "\n";
    params += "ENCODERS = " + std::to_string(runtimeParameters.periodEncoders) + "\n";
    params += "ODOMETRY = " + std::to_string(runtimeParameters.periodOdometry) + "\n";

    return params;
}

void ECU::writeRuntimeParametersToIni()
{
    std::ofstream file(runtimeParametersIni.c_str());
    file << serializeRuntimeParameters();
}

void ECU::setCorrLeft(float corrLong, float corrShort)
{
    send(LEFT_CORRFAC_SHORT, corrShort);
    send(LEFT_CORRFAC_LONG, corrLong);
}

void ECU::setCorrRight(float corrLong, float corrShort)
{
    send(RIGHT_CORRFAC_SHORT, corrShort);
    send(RIGHT_CORRFAC_LONG, corrLong);
}

void ECU::setTerminalMode(uint16_t mode) { send(INSTRUCTION_TERMINALMODE, 0, mode); }

ECU::RuntimeParameters_t ECU::getRuntimeParameters(bool readFromIni)
{
    if (readFromIni)
    {
        readRuntimeParametersFromIni();
    }

    return runtimeParameters;
}

void ECU::setRuntimeParameters(ECU::RuntimeParameters_t params, bool writeToIni)
{
    runtimeParameters = params;
    setTerminalMode((uint16_t)runtimeParameters.terminalMode);

    enableCtrlRPM((runtimeParameters.rpmctrlEnable > 0));
    setPILeft(runtimeParameters.kaLeft, runtimeParameters.kpLeft, runtimeParameters.tnLeft, runtimeParameters.tdLeft);
    setPIRight(runtimeParameters.kaRight, runtimeParameters.kpRight, runtimeParameters.tnRight, runtimeParameters.tdRight);
    setCorrLeft(runtimeParameters.corrLongLeft, runtimeParameters.corrShortLeft);
    setCorrRight(runtimeParameters.corrLongRight, runtimeParameters.corrShortRight);
    setRPMLowpass(runtimeParameters.rpmLowpass);

    send(PERIODIC_LATLON, 0, (uint16_t)runtimeParameters.periodLatLon);
    send(PERIODIC_TIME, 0, (uint16_t)runtimeParameters.periodTime);
    send(PERIODIC_DATE, 0, (uint16_t)runtimeParameters.periodDate);
    send(PERIODIC_HEADING, 0, (uint16_t)runtimeParameters.periodHeading);
    send(PERIODIC_ENCODERS, 0, (uint16_t)runtimeParameters.periodEncoders);
    send(PERIODIC_ODOM, 0, (uint16_t)runtimeParameters.periodOdometry);

    if (writeToIni)
    {
        writeRuntimeParametersToIni();
    }

    RCLCPP_INFO(this->get_logger(), "Runtime Parameters\n%s\n",
                serializeRuntimeParameters().c_str());
}

void ECU::publishRuntimeParameters()
{
  ECU::RuntimeParameters_t dat = getRuntimeParameters();
  lois_ecu::msg::RuntimeParameters params;
  params.terminal_mode.data = dat.terminalMode;
  params.rpmctrl_enable.data = dat.rpmctrlEnable;
  params.rpm_lowpass.data = dat.rpmLowpass;
  params.ka_left.data = dat.kaLeft;
  params.kp_left.data = dat.kpLeft;
  params.tn_left.data = dat.tnLeft;
  params.td_left.data = dat.tdLeft;
  params.ka_right.data = dat.kaRight;
  params.kp_right.data = dat.kpRight;
  params.tn_right.data = dat.tnRight;
  params.td_right.data = dat.tdRight;
  params.corr_long_left.data = dat.corrLongLeft;
  params.corr_short_left.data = dat.corrShortLeft;
  params.corr_long_right.data = dat.corrLongRight;
  params.corr_short_right.data = dat.corrShortRight;
  params.period_latlon.data = dat.periodLatLon;
  params.period_time.data = dat.periodTime;
  params.period_date.data = dat.periodDate;
  params.period_heading.data = dat.periodHeading;
  params.period_encoders.data = dat.periodEncoders;
  params.period_odometry.data = dat.periodOdometry;

  pubRuntimeParameters->publish(params);
}

int ECU::calculateCRC(char *message)
{
    int cs = 0;
    for (int i = 0; i < 1 + MESSAGE_INSTRUCTIONBYTES * 2 + MESSAGE_DATABYTES * 2; i++)
    {
        cs = cs ^ static_cast<int>(message[i]);
    }
    return cs;
}

uint32_t ECU::uint32_2x16(uint16_t val1, uint16_t val2)
{
    uint32_t val = static_cast<uint32_t>(val1);
    val = val << 16;
    val += static_cast<uint32_t>(val2);

    return val;
}

float ECU::float_2x16(uint16_t val1, uint16_t val2)
{
    uint32_t val = uint32_2x16(val1, val2);

    float f;
    memcpy(&f, &val, sizeof(uint32_t));

    return f;
}

void ECU::callback_EmergencyStop(bool isPressed)
{
    RCLCPP_ERROR(get_logger(), "EMERGENCYSTOP: %d", isPressed);
    diagnostic_msgs::msg::DiagnosticStatus emStop;
    if (isPressed)
    {
        emStop.level = 2;
    }
    else
    {
        emStop.level = 0;
    }
    pubEmergencyStop->publish(emStop);
}

void ECU::callback_RPMRecord(ECU::RPMData_t record)
{
    std_msgs::msg::UInt16MultiArray msg;

    while (!record.left.empty())
    {
        msg.data.push_back(record.left.front());
        record.left.pop();
    }
    while (!record.right.empty())
    {
        msg.data.push_back(record.right.front());
        record.right.pop();
    }

    pubRPMRecord->publish(msg);
}

void ECU::callback_TorqueRecord(ECU::RPMData_t record)
{
    std_msgs::msg::UInt16MultiArray msg;

    while (!record.left.empty())
    {
        msg.data.push_back(record.left.front());
        record.left.pop();
    }
    while (!record.right.empty())
    {
        msg.data.push_back(record.right.front());
        record.right.pop();
    }

    pubTorqueRecord->publish(msg);
}

void ECU::callback_HeadingRecord(ECU::HeadingData_t heading)
{
    std_msgs::msg::Int16MultiArray msg;

    while (!heading.heading.empty())
    {
        msg.data.push_back(heading.heading.front());
        heading.heading.pop();
    }

    pubHeadingRecord->publish(msg);
}

void ECU::callback_Heading(int heading)
{
    std_msgs::msg::Int32 msg;
    msg.data = heading;
    pubKVH->publish(msg);
}

void ECU::callback_GPSData(ECU::GPSData_t position)
{
    sensor_msgs::msg::NavSatFix msg;
    msg.latitude = position.latitude;
    msg.longitude = position.longitude;
    pubGPS->publish(msg);
}

void ECU::callback_Odometry(ECU::OdometryData_t position)
{
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = odomParent;

    msg.child_frame_id = odomChild;
    msg.pose.pose.position.x = position.x;
    msg.pose.pose.position.y = position.y;
    msg.pose.pose.position.z = 0.0;

    msg.twist.twist.linear.x = position.dx;
    msg.twist.twist.linear.y = position.dy;
    msg.twist.twist.angular.z = position.w;
    pubOdometry->publish(msg);
}
