#include <uwb.h>
#include <ros/ros.h>



namespace tiersuwb
{
    UWB::UWB(std::string _name, int _baudRate) :
      portName_(_name), baudRate_(_baudRate)
    {
      serial_ = open(_name.c_str(), O_RDWR);
      if(serial_ == -1)
      {
        ROS_ERROR_STREAM("Failed to open serial port!");
        exit(0);
      }

      struct termios options;
   	  if(tcgetattr(serial_, &options) != 0){
     	  ROS_ERROR_STREAM("Can't get serial port sets!");
     	  exit(0);
      }
      tcflush(serial_, TCIFLUSH);
      switch(_baudRate)
      {
        case 921600:
          cfsetispeed(&options, B921600);
          cfsetospeed(&options, B921600);
          break;
        case 576000:
          cfsetispeed(&options, B576000);
          cfsetospeed(&options, B576000);
          break;
        case 500000:
          cfsetispeed(&options, B500000);
          cfsetospeed(&options, B500000);
          break;
        case 460800:
          cfsetispeed(&options, B460800);
          cfsetospeed(&options, B460800);
          break;
        case 230400:
          cfsetispeed(&options, B230400);
          cfsetospeed(&options, B230400);
          break;
        case 115200:
          cfsetispeed(&options, B115200);
          cfsetospeed(&options, B115200);
          break;
        case 57600:
          cfsetispeed(&options, B57600);
          cfsetospeed(&options, B57600);
          break;
        case 38400:
          cfsetispeed(&options, B38400);
          cfsetospeed(&options, B38400);
          break;
        case 19200:
          cfsetispeed(&options, B19200);
          cfsetospeed(&options, B19200);
          break;
        case 9600:
          cfsetispeed(&options, B9600);
          cfsetospeed(&options, B9600);
          break;
        case 4800:
          cfsetispeed(&options, B4800);
          cfsetospeed(&options, B4800);
          break;
        default:
          ROS_ERROR_STREAM("Unsupported baud rate!");
          exit(0);
      }
      options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
      options.c_oflag &= ~OPOST;
      options.c_iflag &= ~(IXON | IXOFF | IXANY);
      options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
      options.c_oflag &= ~(ONLCR | OCRNL);
      if(tcsetattr(serial_, TCSANOW, &options) != 0){
    	  ROS_ERROR_STREAM("Can't set serial port options!");
    	  exit(0);
   	  }
    }

    bool UWB::readData(unsigned char* _buf, int _nRead)
    {
      int total = 0, ret = 0;

      while(total != _nRead)
      {
        ret = read(serial_, _buf + total, (_nRead - total));
        if(ret < 0)
        {
          return false;
        }
        total += ret;
      }

      return true;
    }

    float UWB::getDist()
    {
      while(true)
      {
        if(readData(dataBuf, 2))
        {
          if(dataBuf[0] == 0x59 && dataBuf[1] == 0x59)
            break;
        }
        else
        {
          return -1.0;
        }
      }

      if(readData(dataBuf, 7))
      {
        int sumCheck = (0x59 + 0x59) % 256;
        for(int i = 0; i < 6; i++)
        {
          sumCheck = (sumCheck + dataBuf[i]) % 256;
        }

        if(sumCheck == dataBuf[6])
        {
          return ((float)(dataBuf[1] << 8 | dataBuf[0]) / 100.0);
        }
        else
        {
          return 0.0;
        }
      }
      else
      {
        return -1.0;
      }
    }

    void UWB::closePort()
    {
      close(serial_);
    }
}