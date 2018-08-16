/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Haikal Pribadi <haikal.pribadi@gmail.com>
 * Copyright (c) 2018, Zeyu Zhang <zeyuz@outlook.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the Haikal Pribadi nor the names of other
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "eddie.h"
#include <iostream>

typedef std::map<std::string, unsigned char[6] > CommandMap;

// maximum number of tries to contact serial port
#define MAX_N_TRIES   3

Eddie::Eddie() :
  GPIO_COUNT(10),
  ADC_PIN_COUNT(8),
  DIGITAL_PIN_COUNT(10),
  AUXILIARY_POWER_RELAY_COUNT(3),
  PARALLAX_MAX_BUFFER(256), 
  ENCODER_COUNT(2),
  ADC_VOLTAGE_MULTIPLIER(1 / 819),
  BATTERY_VOLTAGE_DIVIDER(3.21), 
  MOTOR_POWER_STOP(0),
  MOTOR_POWER_MAX_FORWARD(127),
  MOTOR_POWER_MAX_REVERSE(-127), 
  TRAVEL_SPEED_MAX_FORWARD(32767),
  TRAVEL_SPEED_MAX_REVERSE(-32767),
  TRAVEL_MAX_SPEED(65535), 
  RELAY_33V_PIN_NUMBER(17),
  RELAY_5V_PIN_NUMBER(17),
  RELAY_12V_PIN_NUMBER(18), 
  PACKET_TERMINATOR('\r'),
  PARAMETER_DELIMITER(' '),
  ERROR("ERROR"), 
  DEFAULT_WHEEL_RADIUS(0.0762),
  DEFAULT_TICKS_PER_REVOLUTION(36),
  GET_VERSION_STRING("VER"), 
  SET_GPIO_DIRECTION_OUT_STRING("OUT"),
  SET_GPIO_DIRECTION_IN_STRING("IN"),
  SET_GPIO_STATE_HIGH_STRING("HIGH"), 
  SET_GPIO_STATE_LOW_STRING("LOW"),
  GET_GPIO_STATE_STRING("READ"),
  GET_ADC_VALUE_STRING("ADC"), 
  GET_PING_VALUE_STRING("PING"),
  SET_DRIVE_POWER_STRING("GO"),
  SET_DRIVE_SPEED_STRING("GOSPD"), 
  SET_DRIVE_DISTANCE_STRING("TRVL"),
  SET_STOP_DISTANCE_STRING("STOP"),
  SET_ROTATE_STRING("TURN"), 
  GET_CURRENT_SPEED_STRING("SPD"),
  GET_CURRENT_HEADING_STRING("HEAD"),
  GET_ENCODER_TICKS_STRING("DIST"), 
  RESET_ENCODER_TICKS_STRING("RST"),
  SET_RAMPING_VALUE_STRING("ACC"),
  FLUSH_BUFFERS_STRING("\r\r\r")
{
  sem_init(&mutex, 0, 1);
  ping_pub_ = node_handle_.advertise<eddiebot_msgs::Ping > ("/eddie/ping_data", 1);
  adc_pub_ = node_handle_.advertise<eddiebot_msgs::ADC > ("/eddie/adc_data", 1);
  encoder_pub_ = node_handle_.advertise<eddiebot_msgs::Encoders > ("/eddie/encoders_data", 1);

  accelerate_srv_ = node_handle_.advertiseService("acceleration_rate", &Eddie::accelerate, this);
  drive_with_distance_srv_ = node_handle_.advertiseService("drive_with_distance", &Eddie::driveWithDistance, this);
  drive_with_power_srv_ = node_handle_.advertiseService("drive_with_power", &Eddie::driveWithPower, this);
  drive_with_speed_srv_ = node_handle_.advertiseService("drive_with_speed", &Eddie::driveWithSpeed, this);
  get_distance_srv_ = node_handle_.advertiseService("get_distance", &Eddie::getDistance, this);
  get_heading_srv_ = node_handle_.advertiseService("get_heading", &Eddie::getHeading, this);
  get_speed_srv_ = node_handle_.advertiseService("get_speed", &Eddie::GetSpeed, this);
  reset_encoder_srv_ = node_handle_.advertiseService("reset_encoder", &Eddie::resetEncoder, this);
  rotate_srv_ = node_handle_.advertiseService("rotate", &Eddie::rotate, this);
  stop_at_distance_srv_ = node_handle_.advertiseService("stop_at_distance", &Eddie::stopAtDistance, this);

  std::string port = "/dev/ttyUSB0";
  node_handle_.param<std::string>("serial_port", port, port);
  initialize(port);
  
  command(RESET_ENCODER_TICKS_STRING);
}

Eddie::~Eddie()
{
  command("STOP 0");
  close(tty_fd);
}

void Eddie::initialize(std::string port)
{
  ROS_INFO("Initializing Parallax board serial port connection");

  memset(&tio, 0, sizeof (tio));
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_cflag = CS8 | CREAD | CLOCAL; // 8n1, see termios.h for more information
  tio.c_lflag = 0;
  tio.c_cc[VMIN] = 1;
  tio.c_cc[VTIME] = 5;

  tty_fd = open(port.data(), O_RDWR | O_NONBLOCK);
  cfsetospeed(&tio, B115200); // 115200 baud
  cfsetispeed(&tio, B115200); // 115200 baud

  tcsetattr(tty_fd, TCSANOW, &tio);
  usleep(100000);
}

std::string Eddie::command(std::string str)
{
  sem_wait(&mutex);
  ssize_t written;
  std::stringstream result("");
  int count = 0;
  unsigned char c;
  int size = str.size() + 1;
  unsigned char command[size];

  for (int i = 0; i < size - 1; i++)
  {
    command[i] = str[i];
  }
  command[size - 1] = PACKET_TERMINATOR; // Having exces terminator is okay, it's good to guarantee

  written = write(tty_fd, command, size);
  while(read(tty_fd, &c, 1) <= 0){
    usleep(1000);
    count++;
    if(count>=80)
    {
      ROS_ERROR("ERROR: NO PARALLAX EDDIE ROBOT IS CONNECTED.");
      ROS_ERROR(str.c_str());
      break;
    }
  }
  if(count<80){
    result << c;
    count = 0;
    while(c != '\r'){
      if(read(tty_fd, &c, 1)>0)
        result << c;
    }
  }
  sem_post(&mutex);
  return result.str();
}

std::string Eddie::generateCommand(std::string str1)
{
  std::stringstream ss;
  ss << str1 << PACKET_TERMINATOR;
  return ss.str();
}

std::string Eddie::generateCommand(std::string str1, int num1)
{
  std::stringstream ss;
  ss << str1 << PARAMETER_DELIMITER << intToHexString(num1) << PACKET_TERMINATOR;
  return ss.str();
}

std::string Eddie::generateCommand(std::string str1, int num1, int num2)
{
  std::stringstream ss;
  ss << str1 << PARAMETER_DELIMITER << 
    intToHexString(num1) << PARAMETER_DELIMITER <<
    intToHexString(num2) << PACKET_TERMINATOR;
  return ss.str();
}

std::string Eddie::intToHexString(int num)
{
  std::stringstream ss;
  ss << std::hex << num;
  return ss.str();
}

eddiebot_msgs::Ping Eddie::getPingData()
{
  std::string result = command(GET_PING_VALUE_STRING);
  //std::string result = "133 3C9 564 0F9 29B 0F0 31A 566 1E0 A97\r";
  eddiebot_msgs::Ping ping_data;
  if (result.size() <= 1)
  {
    ping_data.status = "EMPTY";
    return ping_data;
  }
  else if (result.size() >= 6)
  {
    if (result.substr(0, 5) == "ERROR")
    {
      ping_data.status = result;
      return ping_data;
    }
  }
  ping_data.status = "SUCCESS";
  std::stringstream value;
  uint16_t data;
  for (ushort i = 0; i < result.size(); i += 4)
  {
    if (result[i] == '\r') break;
    value << std::hex << result.substr(i, 3);
    value >> data;
    ping_data.value.push_back(data);
    value.str(std::string());
    value.clear();
  }
  return ping_data;
}

eddiebot_msgs::ADC Eddie::getADCData()
{
  std::string result = command(GET_ADC_VALUE_STRING);
  //std::string result = "9C7 11E E4E 5AB 20F 97B 767 058\r";
  eddiebot_msgs::ADC adc_data;
  if (result.size() <= 1)
  {
    adc_data.status = "EMPTY";
    return adc_data;
  }
  else if (result.size() >= 6)
  {
    if (result.substr(0, 5) == "ERROR")
    {
      adc_data.status = result;
      return adc_data;
    }
  }
  adc_data.status = "SUCCESS";
  std::stringstream value;
  uint16_t data;
  for (ushort i = 0; i < result.size(); i += 4)
  {
    if (result[i] == '\r') break;
    value << std::hex << result.substr(i, 3);
    value >> data;
    adc_data.value.push_back(data);
    value.str(std::string());
    value.clear();
  }
  return adc_data;
}


bool Eddie::getEncodersData(eddiebot_msgs::Encoders &data)
{
  std::string cmd = GET_ENCODER_TICKS_STRING;
  std::string cmd_response = command(cmd);

  if (cmd_response.substr(0, 5) != "ERROR" && cmd_response.size() >= 18)
  {
    std::stringstream value;
    unsigned int tmp;
    value << std::hex << cmd_response.substr(0, 8);
    value >> tmp;
    data.left = static_cast<int>(tmp);

    value.str(std::string());
    value.clear();
    value << std::hex << cmd_response.substr(9, 8);
    value >> tmp;
    data.right = static_cast<int>(tmp);

    return true;
  }
  else
    return false;
}


void Eddie::publishPingData()
{
  ping_pub_.publish(getPingData());
}

void Eddie::publishADCData()
{
  adc_pub_.publish(getADCData());
}

void Eddie::publishEncodersData()
{
  int cnt = 0;
  eddiebot_msgs::Encoders encoders_data;

  if( !getEncodersData(encoders_data) && cnt < MAX_N_TRIES)
    cnt++;

  if(cnt == MAX_N_TRIES){
    ROS_ERROR("[Eddie::publishEncodersData] Cannot contact serial port");
    exit(-1);
  }
  else
    encoder_pub_.publish(encoders_data);
}

bool Eddie::accelerate(eddiebot_msgs::Accelerate::Request &req,
  eddiebot_msgs::Accelerate::Response &res)
{
  //this feature does not need to validate the parameters due the limited range of parameter data type
  std::string cmd;
  cmd = generateCommand(SET_RAMPING_VALUE_STRING, req.rate);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}

bool Eddie::driveWithDistance(eddiebot_msgs::DriveWithDistance::Request &req,
  eddiebot_msgs::DriveWithDistance::Response &res)
{
  //this feature does not need to validate the parameters due the limited range of parameter data type
  std::string cmd;
  cmd = generateCommand(SET_DRIVE_DISTANCE_STRING, req.distance, req.speed);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}

bool Eddie::driveWithPower(eddiebot_msgs::DriveWithPower::Request &req,
  eddiebot_msgs::DriveWithPower::Response &res)
{
  if (req.left > MOTOR_POWER_MAX_FORWARD || req.right > MOTOR_POWER_MAX_FORWARD ||
      req.left < MOTOR_POWER_MAX_REVERSE || req.right < MOTOR_POWER_MAX_REVERSE)
  {
    return false;
  }
  std::string cmd;
  cmd = generateCommand(SET_DRIVE_POWER_STRING, req.left, req.right);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else{
    ROS_ERROR("%s",cmd_response.data());
    return false;
  }
}

bool Eddie::driveWithSpeed(eddiebot_msgs::DriveWithSpeed::Request &req,
  eddiebot_msgs::DriveWithSpeed::Response &res)
{
  if (req.left > TRAVEL_SPEED_MAX_FORWARD || req.right > TRAVEL_SPEED_MAX_FORWARD ||
      req.left < TRAVEL_SPEED_MAX_REVERSE || req.right < TRAVEL_SPEED_MAX_REVERSE)
  {
    return false;
  }
  std::string cmd;
  cmd = generateCommand(SET_DRIVE_SPEED_STRING, req.left, req.right);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}

bool Eddie::getDistance(eddiebot_msgs::GetDistance::Request &req,
  eddiebot_msgs::GetDistance::Response &res)
{
  std::string cmd = GET_ENCODER_TICKS_STRING;
  std::string cmd_response = command(cmd);
  if (cmd_response.substr(0, 5) != "ERROR" && cmd_response.size() >= 18)
  {
    std::stringstream value;
    value << std::hex << cmd_response.substr(0, 8);
    value >> res.left;
    value.str(std::string());
    value.clear();
    value << std::hex << cmd_response.substr(9, 8);
    value >> res.right;
    return true;
  }
  else
    return false;
}

bool Eddie::getHeading(eddiebot_msgs::GetHeading::Request &req,
  eddiebot_msgs::GetHeading::Response &res)
{
  std::string cmd = GET_CURRENT_HEADING_STRING;
  std::string cmd_response = command(cmd);
  if (cmd_response.substr(0, 5) != "ERROR" && cmd_response.size() >= 4)
  {
    std::stringstream value;
    value << std::hex << cmd_response.substr(0, 3);
    value >> res.heading;
    return true;
  }
  else
    return false;
}

bool Eddie::GetSpeed(eddiebot_msgs::GetSpeed::Request &req,
  eddiebot_msgs::GetSpeed::Response &res)
{
  std::string cmd = GET_CURRENT_SPEED_STRING;
  std::string cmd_response = command(cmd);
  if (cmd_response.substr(0, 5) != "ERROR" && cmd_response.size() >= 10)
  {
    uint16_t tmp;
    std::stringstream value;
    value << std::hex << cmd_response.substr(0, 4);
    value >> tmp;
    res.left = static_cast<uint16_t>(tmp);

    value.str(std::string());
    value.clear();
    value << std::hex << cmd_response.substr(5, 4);
    value >> tmp;
    res.right = static_cast<uint16_t>(tmp);
    
    return true;
  }
  else
    return false;
}

bool Eddie::resetEncoder(eddiebot_msgs::ResetEncoder::Request &req,
  eddiebot_msgs::ResetEncoder::Response &res)
{
  std::string cmd = RESET_ENCODER_TICKS_STRING;
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}

bool Eddie::rotate(eddiebot_msgs::Rotate::Request &req,
  eddiebot_msgs::Rotate::Response &res)
{
  std::string cmd;
  cmd = generateCommand(SET_ROTATE_STRING, req.angle, req.speed);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}

bool Eddie::stopAtDistance(eddiebot_msgs::StopAtDistance::Request &req,
  eddiebot_msgs::StopAtDistance::Response &res)
{
  std::string cmd;
  cmd = generateCommand(SET_STOP_DISTANCE_STRING, req.distance);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}

int main(int argc, char** argv)
{
  ROS_INFO("Parallax Board booting up");
  ros::init(argc, argv, "parallax_board");
  Eddie eddie; //set port to connect to Parallax controller board
  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    // eddie.publishPingData();
    // eddie.publishADCData();
    eddie.publishEncodersData();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
