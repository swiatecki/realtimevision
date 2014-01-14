# include "setserial.h"

int set_serial(int fd, unsigned int speed)
{
  struct termios options;

  /* Get handle for serial port */
  if(tcgetattr(fd, &options) == -1)
  {
    fprintf(stderr, "set_serial: Unable to tcgetattr(fd, &options)\n");
    return(1);
  }
  
  switch(speed)
  {
    case 2400:
    {
      cfsetispeed(&options, B2400);  /*/Set input baudrate */
      cfsetospeed(&options, B2400);  /*/Set output baudrate */
      break;
    }
    case 4800:
    {
      cfsetispeed(&options, B4800);  /*/Set input baudrate */
      cfsetospeed(&options, B4800);  /*/Set output baudrate */
      break;
    }
    case 9600:
    {
      cfsetispeed(&options, B9600);  /*/Set input baudrate */
      cfsetospeed(&options, B9600);  /*/Set output baudrate */
      break;
    }
    case 19200:
    {
      cfsetispeed(&options, B19200);  /*/Set input baudrate */
      cfsetospeed(&options, B19200);  /*/Set output baudrate */
      break;
    }
    case 38400:
    {
      cfsetispeed(&options, B38400);  /*/Set input baudrate */
      cfsetospeed(&options, B38400);  /*/Set output baudrate */
      break;
    }
    case 115200:
    {
      cfsetispeed(&options, B115200);  /*/Set input baudrate */
      cfsetospeed(&options, B115200);  /*/Set output baudrate */
      break;
    }
    case 500000:
    {
      cfsetispeed(&options, B500000);  /*/Set input baudrate */
      cfsetospeed(&options, B500000);  /*/Set output baudrate */
      break;
    }
    default:
    {
      fprintf(stderr, "Trying to set speed of serial port to unsupported %d bit/sec\n", speed);
      fprintf(stderr, "- supports 2400,4800,9600,19200,38400,115200,500000 bit/sec\n");
      return(2);
    }
  }

  /* Enable the receiver and set local mode... */
  options.c_cflag |= (CLOCAL | CREAD);

  /*/ Set to no parity 8 bit,1 stop, 8N1
  //options.c_cflag &= ~PARENB;
  //options.c_cflag &= ~CSTOPB;
  //options.c_cflag &= ~CSIZE; */
  options.c_cflag |= CS8;
  /*
  //options.c_cflag = 0;
  //options.c_cflag |= (CLOCAL | CREAD | CS8);


  //Using raw input mode
  //options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); */
  options.c_lflag = 0;
  /*
  //ignore parity errors
  //options.c_iflag |= IGNPAR; */
  options.c_iflag =0;  /*/Must be zero, change it at own risk */
  /*
  //Set output to raw mode
  //options.c_oflag &= ~OPOST; */
  options.c_oflag = 0;

  /*/Set the new options for the port... */
  if(tcsetattr(fd, TCSANOW, &options) == -1)
  {
    fprintf(stderr, "ste_serial: Can not set serial port parameters\n");
    return(3);
  }

  return(0);  /* success */
}
