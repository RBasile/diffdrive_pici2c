#ifndef DIFFDRIVE_PIC_I2C_HPP
#define DIFFDRIVE_PIC_I2C_HPP

#include <sstream>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>

using namespace std::chrono_literals;

class I2CpicCom
{

public:

  I2CpicCom() = default;

  void connect(const std::string &device, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    i2c_fd_ = open("/dev/i2c-1", O_RDWR);
    if (!i2c_fd_) {
      std::__throw_runtime_error("Failed to open I2C interface!");
    }
    if (i2c_fd_ < 0 || ioctl(i2c_fd_, I2C_SLAVE, slave_address) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("MyI2CMotorInterface"), "Failed to open I2C");
      std::__throw_runtime_error("fail1");
    }

    std::cout << "device: " << device << std::endl;
  }

  void disconnect()
  {
    sendDataToMotor(0.0,0.0);
    if (i2c_fd_) {
      close(i2c_fd_);
    }
  }

  bool connected() const
  {
    if (i2c_fd_){return true;}
    else{return false;}
  }

  float convertPIC18Float(uint8_t *data) {
    // Reverse the endianness
    uint32_t raw = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];

    uint8_t sign = raw >> 23 & 1;
    uint8_t exponent = raw >> 24 & 0xFF;
    uint32_t mantissa = raw & 0x7FFFFF;

    // Reconstruct the float
    uint32_t reconstructedBits = (sign << 31) | (exponent << 23) | mantissa;
    float result;

    // Use memcpy to convert the reconstructed bits into a float
    memcpy(&result, &reconstructedBits, sizeof(result));

    return result; 
  }


  void floatToPIC18Float(uint8_t *data, float value) {
    uint32_t rawBits;

    // Use memcpy to get the raw bits of the float
    memcpy(&rawBits, &value, sizeof(rawBits));

    uint8_t sign = (rawBits >> 31) & 1;
    uint8_t exponent = (rawBits >> 23) & 0xFF;
    uint32_t mantissa = rawBits & 0x7FFFFF;

    // Reconstruct the uint32_t in the PIC18 format
    uint32_t pic18Format = (mantissa) | (sign << 23) | (exponent << 24);

    // Adjust the endianness
    data[0] = pic18Format & 0xFF;
    data[1] = (pic18Format >> 8) & 0xFF;
    data[2] = (pic18Format >> 16) & 0xFF;
    data[3] = (pic18Format >> 24) & 0xFF;
  }


/* the ic2 reg on the pic
structure statusStruc
   dim targetR as float    '00
   dim targetL as float    '04
   dim speedR  as float    '08
   dim speedL  as float    '12
   dim rotR    as float    '16
   dim rotL    as float    '20
   dim xPos as float       '24
   dim yPos as float
   dim rotation as float
   dim rotRprev as float
   dim rotLprev as float
end structure
*/

void read_encoder_values(float &rotR, float &rotL)
{
  uint8_t reg = 16;  // rotR
  if (write(i2c_fd_, &reg, 1) != 1) {
    return;
  }
  uint8_t buf[8];
  if (read(i2c_fd_, buf, 8) == 8)
  {
    rotR = convertPIC18Float(buf);
    rotL = convertPIC18Float(buf + 4);
    std::cout << "left: " << rotL << " right: " << rotR << std::endl;
  }
}
int reset_encoder_values()
{
  uint8_t buffer[2];
  buffer[0] = 56;  //status register
  buffer[1] = 1;  //reset bit in status
  if (write(i2c_fd_, buffer, sizeof(buffer)) != sizeof(buffer)) {
    std::cout << "Error Reset: " << std::endl;
    return 0;
  }
  std::cout << "Reset: " << std::endl;
  return 1;
}

int sendDataToMotor(float rotSpeedR = 0,float rotSpeedL = 0)
{
  //std::cout << "left: " << rotSpeedL << " right: " << rotSpeedR << std::endl;

  uint8_t buffer[9];  // 1 byte for register, 4 for right speed, 4 for left speed
  buffer[0] = 0x08;
  floatToPIC18Float(&buffer[1], rotSpeedR);
  floatToPIC18Float(&buffer[5], rotSpeedL);

  if (write(i2c_fd_, buffer, sizeof(buffer)) != sizeof(buffer)) {
    return 0;
  }
  return 1;
}




private:
    //LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
    int i2c_fd_;
    int slave_address = 100;
    uint8_t buf_[10];
};

#endif // DIFFDRIVE_PICI2C_ARDUINO_COMMS_HPP