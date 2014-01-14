# include <stdio.h>
# include <stdlib.h>
# include <unistd.h>
# include <pthread.h>
# include <sys/types.h>
# include <sys/stat.h>
# include <fcntl.h>
# include <signal.h>
# include <SDL/SDL.h>

# include "setserial.h"

# define XBOW_PORT "/dev/ttyS0"
# define BUFFER_LEN 100

unsigned short xbow-crc(char *pkg, int pkglen);
{
  int i;
  unsigned short res = 0x1D0F;  // Start value
  
  for(i = 2; i < pkglen - 2; i++)
  {
    res ^= pkg[i] << 8;
    
    for(j = 0; j < 8; j++)
    {
      if(res & 0x8000)
        res = (res << 1) ^ 0x1021;
      else
        res = (res << 1);
    }
  }
  
  printf("Calculated: 0x%04x\nGiven:      0x%04x\n", res, *(pkg + pkglen - 2))
  return(res);
}

int running = 1;

void stopme(int signo)
{
  running = 0;
}

int main(int argc, char **argv)
{
  unsigned char buffer[BUFFER_LEN], msg[BUFFER_LEN];
  int portfd;
  int i, n, msg_idx;
  pthread_t recv_thread;
  
  /* SDL variables */
  //SDL_Surface *scr;
  
  /* Catch interrupt signal ctrl+c */
  signal(SIGINT, stopme);
  
  if((portfd = open(XBOW_PORT, O_RDWR | O_NONBLOCK)) == -1)
  {
    fprintf(stderr, "Unable to open xbow port: %s\n", XBOW_PORT);
    exit(1);
  }
  
  if(set_serial(portfd, 38400) != 0)
  {
    fprintf(stderr, "Error in set_serial\n");
    exit(2);
  }
  /*
  if(SDL_Init(SDL_INIT_AUDIO | SDL_INIT_VIDEO) < 0) 
  {
    fprintf(stderr, "Unable to init SDL: %s\n", SDL_GetError());
    exit(3);
  }

  scr = SDL_SetVideoMode(400, 100, 16, SDL_SWSURFACE);
  */
  while(running)
  {
    n = read(portfd, buffer, BUFFER_LEN);
    
    if(n > 0)
    {
      for(i = 0; i < n; i++)
      {
        printf("0x%02X ", buffer[i]);
      }
      printf("\n");
      
      if((buffer[0] == 0x55) && (buffer[1] == 0x55))  // New package
      {
        
      }
    }
    usleep(2000);
  }
  
  close(portfd);
  SDL_Quit();
  exit(0);
}
