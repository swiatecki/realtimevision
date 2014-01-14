/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>

//#include <ugen4/uimage2.h>
//#include <urob4/ulogfile.h>
//#include <urob4/uvarpool.h>

#include "fwguppy.h"
//#include "labyrinthgame.h"
//#include "gamesim.h"
//#include "showimage.h"
//#define USE_GUPPY
#include <dc1394/control.h>
#include <dc1394/types.h>
#define FILENAME_MAX_SIZE (60)
  //

  /**
  Set camera node info */
  bool setCamDeviceNode(int camNum);
  /**
  Get number of cameras as reported by DC1394 */
  int getDc1394CamCnt();
  /**
  Capture an image after iso-streaming is started */
  bool getSingleImage(UImage * destination);
  /**
  Start iso transmission
  \param isoBW is allocated bandwith for camera connection
  and must be one of 100, 200, 400, 800, 1600, 3200. maximum depends on HW.
  \param packetSize is number of image bytes send in each timeslot (of 125us).
  // The IEEE 1394 specification imposes the following limits on the payload size of isochronous packets:
  // Speed   Cycle  Max. packet size  Max. bandwidth per ISO channel
  // [Mb/s]  [µs]         [B]       [MB/s]  [MiB/s]
  // S100    125         1024       8.192   7.8125
  // S200    125         2048      16.384  15.6250   --- used for guppy to allow 2cameras on one s400 connection
  // S400    125         4096      32.768  31.2500   --- max for normal FW cable
  // S800    125         8192      65.536  62.5000
  // S1600   125        16384     131.072 125.0000
  // S3200   125        32768     262.144 250.0000
  \returns true if successful. */
  bool startIsoTransmission(int isoBW, int packetSize);
  /**
  Stop iso transmission */
  bool stopIsoTransmission();
  /**
  Setup dma image capture */
  bool setDMAcapture();
  /**
  Test if souch device exist and can be opened.
  Returns true if device can be opened (a device exists). */
  bool deviceExist();
  /**
  Open devise to get default camera parameters.
  \param idx is number of camera if more cameras available - first is 0
  Returns true if successful. */
  bool openDevice(int idx);
  /**
  close if the device is open */
  void closeDevice();
  /**
   * close and release all resources */
  void closeDeviceRelease();
  /**
  Get new image - the newest available to the provided image buffer.
  if 'image' == NULL, then get and discard an image.
  \returns true if an image could be captured. */
  bool getImageSnapshot(UImage * image);
  /**
  Get current frame rate setting */
  int getFrameRate();
  /**
  Stop the frame read thread.
  If 'wait' == true, then the thread are joined prior to return. */
  void stop(bool andWait);
  /**
  Star the read thread.
  Waits until the thread has had time to start and initialize.
  \returns true if the thread is started. */
  bool start();
  /**
  Thread that read frames off the camera device driver queue to allow camPush functions to be
  executed.
  If no users of the frames, they are just discarded.
  If single snapshots are needed, these are serviced by this thread too.
  Should not be called directly, are called implicitly,
  when camera is opened.*/
  void run();
  /**
  Toggle the control 2 output (pin 2 on Guppy) on and off, to trigger a new set of images, that
  is if pin 2 is connected to pin 4 on this and the other camera(s) to be triggered.
  Does nothing if not connected and if external trigger is not available.
  \returns true if calls were successfull. */
  bool makeTriggerPulse();
  /**
  Get the bayer pattern from the camera.
  Result is one of the PIX_PLANEX_BGGR integers. */
  int getBayerPattern();
  /**
  Get one of the feature sets stored in 'features', holding
  requested data. */
  /**
  Print camera infor to console */
  void print_mode_info(dc1394camera_t *camera , uint32_t mode);
  /**
  Print format name to console */
  void print_format(uint32_t format);
  /**
  Release BW left over form crash or similar */
  void release_iso_and_bw();
  /**
  Set shutter value to camera. if value is -1 then camera is set to auto.
  Value should be within supported range.
  \returns true if set. */
  bool setShutterRaw(int value);
  /**
  Get shutter values from camera and set fShutter structure as well as global variable 'shutter'
  \returns true if data is fresh from camera. */
  bool getShutterRaw();
  /**
  Set gain value and assumes that noone else mangles with camera or global variables
  A negative value (-1) sets automatic gain.
  \Returns true if set successful - i.e. the function is supported by camera.*/
  bool setGainRaw(int agc);
  /**
  Gets raw information from camera and sets global variables.
  Assumes noone else fiddles with camera nor the gain global var
  \returns true if call to camera is successfull */
  bool getGainRaw();
  /**
  Set exposure target to this value - must be within allowed range - see global vars for device.
  \returns true if set. */
  bool setExposureTargetRaw(int value);
  /**
  Get exposure target value (used in auto shutter mode only) from camera
  \returns true on fresh data. */
  bool getExposureTargetRaw();
  /**
  Get white ballance settings from camera
  \returns true if fresh from camera */
  bool getWhiteBalanceRaw();
  /**
  Set white ballance to camera device using these values.
  \param mode is 3 for manual and 4 for auto (auto is one-shot only)
  \param red is manual red gain value - must be within range (see global vars for device)
  \param blue is manual blue gain value - must be within range (see global vars for device)
  \returns true if set. */
  bool setWhiteBalanceRaw(int mode, int red, int blue);
  /**
  Set external trigger on or off.
  \param value is value to set off=free running trigger based on available bandwidth.
  on is trigger using external pin (may be connected to trigger function
  \returns true if set */
  bool setExternalTriggerRaw(bool value);
  /**
  Get fresh trugger data from camera. Sets fTrigger structure and global variable for device.
  \returns true if fresh. */
  bool getExternalTriggerRaw();
  /**
  * Set roi for camera
  * \param top is new top line
  * \param hgt is new height
  * \returns true if changed successfully */
  bool setROI(int top, int hgt);
  /**
   * Save PNG image into data subdirectory - with image number and time in filename
   * \param img is image to save
   * \returns true if saved */
  bool saveImage(UImage * img);


/**
Node for this camera */
dc1394_t * devPlatform = NULL;
/**
Handle for accessing the camera */
dc1394camera_t * camHandle = NULL;
/**
format mode (should be format 7 (DC1394_VIDEO_MODE_FORMAT7_0)) */
dc1394video_mode_t selected_mode;
/// position unit - increment unit for ROI position (col, row)
uint32_t position_unit[2];
/// size unit - increment size for ROI size (width, height)
uint32_t size_unit[2];
/// max size of image in this format 7 mode (width, height)
uint32_t size_max[2];

/** supported video modes */
dc1394video_modes_t modes;
/**
ROI settings */
// unsigned int roiWidth;
// unsigned int roiHeight;
// unsigned int roiTop;
// unsigned int roiLeft;
/** Terminate the frame read thread */
bool stopFrameRead;
/** Flag that is set by the read thread on entry
and reset by the thread on exit */
bool threadRunning;
/**
Thread handle for frame read thread. */
pthread_t thRead;
//   /**
//   Image pointer for single image capture */
//   UImage * captureImage;
/** Last time an image was received - for frame-rate calculation */
UTime tLastFrame;
/**  Time last full frame were received */
UTime tLastFull;
/**
Capture lock to be unlocked (posted) when
a snapshot image is needed.
Should be posted by the thread needing the image, and will be
accepted (locked by a tryPost() or tryLock()) by the frame read thread.*/
ULock captureDo;
/**
Capture finished post, posted by the read thread and the thread
needing the image may wait on this lock. */
ULock captureDone;
/// feature info
/// gain feature
dc1394feature_info_t fGain;
/// shutter
dc1394feature_info_t fShutter;
/// white ballance
dc1394feature_info_t fWhite;
/// exposure - target value for auto gain/shutter
dc1394feature_info_t fExposure;
/// external trigger settings
dc1394feature_info_t fTrigger;
/// external trigger delay
dc1394feature_info_t fTriggerDelay;
/**
Flag indicating that settings are changed and should be implemented now */
//bool settingsChanged;
// for framerate calculation
int lastImageNumber;
/// implemented isoBW;
int isoBW = 400;
/// is camera open
bool cameraOpen;
/// device number (on firewire bus)
int deviceNumber = 0;
/// camera name - from camera
const int MAZ_CAM_NAME_CNT = 500;
char camName[MAZ_CAM_NAME_CNT] = "";
// size of fw packet (bytes each 125us)
int packetSize;
//
int imageNumber = 0;
int frameHeight = 0;
int frameWidth = 0;
int frameTop = 0;
// actual measured framerate
float frameRate = 0.0;
int gainValue = 64;
int shutterValue = 187;

UImage * imgBuff = NULL;
UImage * imgFull = NULL;
//UImage * imgBall = NULL;
// bool initialized = false;
// bool saveSample = false;

ULock roi_lock;
int roi_hgt = 64, roi_top = 0; //hgt = 15,top =0 emborg
int roi_hgtMin = 64;
int roi_hgtMax = 480;
int roi_widthMax = 752;
/// ball position [0] is x (column) [1] is y (row) in small ROI image (imgBall)
float ballPosition[2];
bool ballOK = false;
// offset to green [even, odd] - one of them should be zero
int greenOffset[2];
// correlation som at ball
int ballQuality;
//difference of blue and red
int ballQualityBlueRed;
/// ball position time
UTime ballTime;
// control semaphore to start a new control cycle
USemaphore semBallPosition;
// initialize full image by scrolling down slize
bool initialImage = true;


/// test of basic camera loop
void looptest()
{
  bool isOK;
  UTime t;
  // init
  //camType = CAM_DEV_IEEE1394;
  stopFrameRead = true;
  threadRunning = false;
  fGain.id = (dc1394feature_t) 0;
  fShutter.id = (dc1394feature_t) 0;
  fWhite.id = (dc1394feature_t) 0;
  fExposure.id = (dc1394feature_t) 0;
  fTrigger.id = (dc1394feature_t) 0;
  fTriggerDelay.id = (dc1394feature_t) 0;
  imageNumber = 0;
  lastImageNumber = 0;
  gainValue=32;
  shutterValue=150;
  //
  camHandle = NULL;
  devPlatform = NULL;
  // find device and set camHandle and devPlatform (for FW device)
  setCamDeviceNode(deviceNumber);
  isoBW = 400;
  packetSize = 1600;
  isOK = openDevice(deviceNumber);
  if (isOK)
  {
    UImage * img;
    bool imageOK;
    double t2,t1;
    if (imgBuff == NULL)
    { // make push buffer (if needed)
      imgBuff = new UImage();
    }
    // initialize push buffer
    imgBuff->setSize(480, roi_widthMax, 1, 8, "RGGB");
    imgBuff->valid = false;
    img = imgBuff;
    t.now();
    t2 = 0.0;
    for (int j=0; j < 9; j++)
    {
      if (j > 0)
        setROI(j*50, 60);
      for (int i = 0; i < 8; i++)
      {
        imageOK = getSingleImage(img);
        t1 = img->imgTime - t;
        printf("Got image %3d %s, top %d, size %dx%d, imgTime %10.4f, dt=%f\n",imageNumber, bool2str(imageOK), frameTop, img->width(), img->height(), t1, t1-t2);
        t2 = t1;
      }
    }
    printf("Closing cam\n");
    closeDevice();
    if (camHandle != NULL)
    { // release device structures
      dc1394_capture_stop(camHandle);
      dc1394_camera_free(camHandle);
      printf("~UCamDevGuppy closed camera handle\n");
      camHandle = NULL;
    }
    if (devPlatform != NULL)
    {
      dc1394_free(devPlatform);
      devPlatform = NULL;
      //printf("~UCamDevGuppy closed device handle\n");
    }
  }
}

/// //////////////////////////////////////////////////////////////////////////

void findSubPixel(UImage * img, int r, int c, float * row, float * col)
{
  int v1g, v2g;
  uint8_t * p1 = (uint8_t *)img->getCharRef(r, c);
  uint8_t * p0 = (uint8_t *)img->getCharRef(r-1, c);
  uint8_t * p2 = (uint8_t *)img->getCharRef(r+1, c);
  if (r % 2 == 0)
  { // center pixel is in a blue row
    v1g = greenOffset[0];
    v2g = greenOffset[1];
  }
  else
  { // center pixel is in a red row
    v2g = greenOffset[0];
    v1g = greenOffset[1];
  }
  int v1 = *p1 + v1g;
  int v2 = v1 - p0[-1] - v2g; // difference to upper-left green pixel
  int v3 = v1 - p0[1] - v2g;  // difference to upper-right green pixel
  int v4 = v1 - p2[-1] - v2g; // difference to lower-left green pixel
  int v5 = v1 - p2[1] - v2g;  // difference to lower-right green pixel
  // diagonal upper-left to lower right
  // multiply by 1.4 sqrt 2 as line is diagonal, or 2 to compensate for never 0-max relation
  float v25 = (v2 + v5) * 2.0;
  float delta1 = (v2 - v5)/v25;   // diagonalen ++ til -- positive if to lower right
  delta1 = limitf(delta1, -1.0, 1.0);
  // diagonal upper-right to lower left
  float v34 = (v3 + v4) * 2.0;
  float delta2=(v3 - v4)/v34;   // diagonalen +- til -+  positive if lower left
  delta2 = limitf(delta2, -1.0, 1.0);
//   v1=billed(x,y);     //den centtralle pixel
// v2=v1-billed(x-1,y-1);  //forskel i lysstyrke i forhold til centeret
// v3=v1-billed(x+1,y-1);  //forskel i lysstyrke i forhold til centeret
// v4=v1-billed(x-1,y+1);  //forskel i lysstyrke i forhold til centeret
// v5=v1-billed(x+1,y+1);  //forskel i lysstyrke i forhold til centeret

// int delta1=((v2/(v2+v5))-v5/(v2+v5))/2;   // diagonalen ++ til --
// int delta2=((v3/(v3+v4))-v4/(v3+v4))/2;   // diagonalen +- til -+

// deltaX=delta1+delta2;
// deltaY=delta1-delta2;
  // column
  *col = c + delta1 - delta2;
  // row
  *row = r + delta1 + delta2;
}

///  ///////////////////////////////////////////////////////////////////////


bool findBall2(UImage * img, int minRow, int maxRow, int minCol, int maxCol, float * row, float * col)
{
  img->lock();
  const int w = img->width();
  const int h = img->height();
//  int maxx, maxy, maxv = 0;
  // compensation for weak green pixel in second (odd) rows
  // loaded from ini-file
  int aBG = greenOffset[0];
  int aGR = greenOffset[1];
  const int minZ = 50;
  const int greenOverRed = 50;
  //int Rxi=0, Ryi=0;
  int Rzi=0;
  //int Gxi=0, Gyi=0;
  int Gzi=0;
  //int Bxi=0, Byi=0;
  int Bzi=0,Ballx=0,Bally=0,Ballz=minZ;
//      uint64_t numPixels = w*h;
  uchar* num =  img->getUCharRef(0,0);
  // ensure bayer pattern is BGGR
  minRow = minRow & 0xFFFe;
  minCol = minCol & 0xFFFe;
  if (minRow < 2)
    minRow = 2;
  if (maxRow > h - 4)
    maxRow = h - 4;
  if (minCol < 2)
    minCol = 2;
  if (maxCol > w - 4)
    maxCol = w - 4;

  for(int FooY= minRow; FooY < maxRow ; FooY+=2)
  {  // look in blue row
    for(int FooX= minCol; (FooX  < maxCol); FooX+=2)
    {  //< w and FooX
//       unsigned char b1 = num[FooX+FooY*w];
//       unsigned char g1 = num[FooX+FooY*w + 1];
//       unsigned char r1 = num[FooX+(FooY -1)*w + 1];
//       unsigned char r2 = num[FooX+(FooY +1)*w + 1];
      if (Bzi < num[FooX+FooY*w])
      {
        Bzi=num[FooX+FooY*w];
//         Bxi=FooX;
//         Byi=FooY;
      }
      if (Gzi < num[FooX+FooY*w+1])
      {
        Gzi = num[FooX+FooY*w+1];
//         Gxi=FooX+1;
//         Gyi=FooY;
      }
      if (Ballz < num[FooX+FooY*w+1] + aBG)
      {           // green                  red one down
        if ((num[FooX+FooY*w+1] + aBG -greenOverRed) > num[FooX+(FooY+1)*w+1])
        {        //  green                  red one up
          if ((num[FooX+FooY*w+1] + aBG -greenOverRed) > num[FooX+(FooY-1)*w+1])
          {
            Ballx=FooX+1;
            Bally=FooY;
            Ballz=num[FooX+FooY*w+1] + aBG;
          }
        }
      }
    }
    // look in odd rows (red row)
    for(int FooX = minCol; (FooX < maxCol); FooX += 2)
    {
//       unsigned char g1 = num[FooX+(FooY+1)*w];
//       unsigned char r1 = num[FooX+(FooY +1)*w - 1];
//       unsigned char r2 = num[FooX+(FooY +1)*w + 1];
//       unsigned char b1 = num[FooX+(FooY +0)*w - 0];
//       unsigned char b2 = num[FooX+(FooY +2)*w - 0];
      if (Ballz < num[FooX+(FooY+1)*w])
      {
        Gzi=num[FooX+(FooY+1)*w];
//         Gxi=FooX;
//         Gyi=FooY+1;
      }
      if (Rzi < num[FooX+(FooY+1)*w+1])
      {
        Rzi = num[FooX+(FooY+1)*w+1];
//         Rxi=FooX+1;
//         Ryi=FooY+1;
      }
      if (Ballz < num[FooX+(FooY+1)*w] + aGR)
      {            // green                                      red
        if ((( num[FooX+(FooY+1)*w]) + aGR -greenOverRed) > num[FooX+(FooY+1)*w-1])
        {         //more grøn                                       than rød
          if ((( num[FooX+(FooY+1)*w]) + aGR -greenOverRed) > num[FooX+(FooY+1)*w+1])
          { // test also more blue than red
            if((num[FooX+(FooY+0)*w-0] + num[FooX+(FooY+2)*w-0]) >
                (num[FooX+(FooY+1)*w-1] + num[FooX+(FooY+1)*w+1]))
            { // if blue is > red ?? // chr
              Ballx=FooX;
              Bally=FooY+1;
              Ballz=num[FooX+(FooY+1)*w] + aGR;
            }
          }
        }
      }
    }
  }
  if (false)
  { // no subpixel
    *col = Ballx;
    *row = Bally;
  }
  else
  { // use subpixel
    if (Ballz > minZ)
      findSubPixel(img, Bally, Ballx, row, col);
  }
  img->unlock();
  ballQuality = Ballz;
  ballQualityBlueRed = 0;
  ballTime = img->imgTime;
  return Ballz > minZ;
}


bool findBall(UImage * img, int minRow, int maxRow, int minCol, int maxCol, float * row, float * col)
{
  img->lock();
  const int w = img->width();
  const int h = img->height();
  int v, red, blue;
  unsigned char * p1;
  int maxx, maxy, maxv = 0;
  // compensation for weak green pixel in second (odd) rows
  // loaded from ini-file
  int aBG = greenOffset[0];
  int aGR = greenOffset[1];
  int bmr = 0;
  // ensure bayer pattern is BGGR
  minRow = minRow & 0xFFFe;
  minCol = minCol & 0xFFFe;
  if (minRow < 2)
    minRow = 2;
  if (maxRow > h - 4)
    maxRow = h - 4;
  if (minCol < 2)
    minCol = 2;
  if (maxCol > w - 4)
    maxCol = w - 4;
  // look in all rows except a bit near the top and bottom
  for (int r = minRow; r < maxRow; r += 2)
  {
    p1 = (unsigned char *) img->getCharRef(r, minCol);
    for (int c = minCol; c < maxCol; c += 2)
    {
      // debug
//       if (p1[1*w] == 105)
//         printf("got 105\n");
      // debug end
      v = 0;
      v += -(p1[-2 * w - 1] + aBG)    /*blue*/          -(p1[-2 * w + 1] + aBG)       /*blue*/        -(p1[-2 * w + 3] + aBG);
      v +=      /*red*/          +(p1[-1 * w - 0] + aGR)     /*red*/          +(p1[-1 * w + 2] + aGR);
      v += -(p1[ 0 * w - 1] + aBG)                      +(p1[ 0 * w + 1] + aBG) * 4   /*blue*/        -(p1[ 0 * w + 3] + aBG);
      v +=                       +(p1[ 1 * w - 0] + aGR)     /*red*/          +(p1[ 1 * w + 2] + aGR);
      v += -(p1[ 2 * w - 1] + aBG)                      -(p1[ 2 * w + 1] + aBG)       /*blue*/        -(p1[ 2 * w + 3] + aBG);
      red = p1[-1*w + 1] + p1[+1*w + 1];
      blue = p1[0] + p1[2];
      if (v > maxv and blue > red + 110)
      { // best green pixel is in an even row
        maxv = v;
        maxx = c + 1;
        maxy = r;
        bmr = blue-red;
      }
      // odd row green pixel in center
      v = 0;
      v += -(p1[-1 * w - 2] + aGR)     /*red*/          -(p1[-1 * w + 0] + aGR)      /*red*/          -(p1[-1 * w + 2] + aGR);
      v +=     /*blue*/          +(p1[ 0 * w - 1] + aBG)    /*blue*/          +(p1[ 0 * w + 1] + aBG);
      v += -(p1[ 1 * w - 2] + aGR)    /*red*/           +(p1[ 1 * w + 0] + aGR) * 4  /*red*/          -(p1[ 1 * w + 2] + aGR);
      v +=     /*blue*/          +(p1[ 2 * w - 1] + aBG)   /*blue*/           +(p1[ 2 * w + 1] + aBG);
      v += -(p1[ 3 * w - 2] + aGR)    /*red*/           -(p1[ 3 * w + 0] + aGR)      /*red*/          -(p1[ 3 * w + 2] + aGR);
      red = p1[+1 * w - 1] + p1[+1 * w + 1];
      blue = p1[0]         + p1[+2 * w];
      if (v > maxv and blue > red + 110)
      { // best green is in an odd row - one pixel down to the right
        maxv = v;
        maxx = c;
        maxy = r + 1;
        bmr = blue-red;
      }
      p1 += 2;
    }
  }
  if (false)
  { // no subpixel
    *col = maxx;
    *row = maxy;
  }
  else
  { // use subpixel
    if (maxv > 0)
      findSubPixel(img, maxy, maxx, row, col);
  }
  img->unlock();
  ballQuality = maxv;
  ballQualityBlueRed = bmr;
  ballTime = img->imgTime;
  return maxv > 200;
}

/// //////////////////////////////////////////////////////////////////////////

void run()
{
  bool isOK;
  UTime t;
  int roi_top_old = -1;
  int roi_hgt_old = -1;
  int imn = 0;
  int shutter_old = shutterValue;
  int gain_old = gainValue;
  int newTop = -1;
  UTime tf;
  UTime tRoi;
  //bool newShutGain = true;
  int initialImageCount = 0;
  ULogFile flog;
  //
  flog.setLogName("ball");
  if (debugLog)
    flog.openLog();
  // init
  //camType = CAM_DEV_IEEE1394;
  threadRunning = true;
  fGain.id = (dc1394feature_t) 0;
  fShutter.id = (dc1394feature_t) 0;
  fWhite.id = (dc1394feature_t) 0;
  fExposure.id = (dc1394feature_t) 0;
  fTrigger.id = (dc1394feature_t) 0;
  fTriggerDelay.id = (dc1394feature_t) 0;
  lastImageNumber = 0;
  //
  camHandle = NULL;
  devPlatform = NULL;
  // find device and set camHandle and devPlatform (for FW device 0)
  setCamDeviceNode(0);
  isoBW = 400;
  packetSize = 1600;
  if (globalDebug >= 2)
    isOK = true;
  else
    isOK = openDevice(0);
  if (isOK)
  {
    UImage * img;
    bool imageOK;
    //double  t2, t1;
    if (imgBuff == NULL)
    { // make push buffer (if needed)
      imgBuff = new UImage();
    }
    // initialize push buffer
    imgBuff->setSize(480, 752, 1, 8, "BGGR");
    imgBuff->valid = false;
    imgBuff->imgTime.now();
    img = imgBuff;
    Wait(0.2);
    setShutterRaw(450); // set too long
    shutter_old = 50;
    setGainRaw(32); // set too low
    gain_old = 10;
    t.now();
    tf.now();
    //t2 = 0.0;
    stopFrameRead = false;
    while (not stopFrameRead)
    {
      // test push stack for commands in need of images
//      printf("RUN: requesting new image ... \n");
      if (globalDebug >= 2)
      {
        Wait(0.004);
        img->lock();
        img->imgTime.now();
        img->camDevice = 10;
        img->cam = NULL;
        img->imageNumber = ++imageNumber;
        img->valid = true;
        frameHeight = roi_hgt; //img->height();
        frameWidth = img->width();
        frameTop = roi_top;
        img->unlock();
        imageOK = true;
      }
      else
        // get real image from camera
        imageOK = getSingleImage(img);
//      printf("RUN: got new image %s - number %d\n", bool2str(imageOK), imageNumber);
      if ((roi_hgt_old != roi_hgt) or (roi_top_old != roi_top))
      {
        if (setROI(roi_top, roi_hgt))
        {
          roi_hgt_old = roi_hgt;
          roi_top_old = roi_top;
          tRoi.now();
        }
        imn = imageNumber;
        newTop = roi_top;
        // reset to ball-finder value
      }
      if (shutterValue != shutter_old)
      {
        if (globalDebug < 2)
          setShutterRaw(shutterValue);
        shutter_old = shutterValue;
        printf("run: video shutter set to %d\n", shutterValue);
        //newShutGain = true;
      }
      if (gainValue != gain_old)
      {
        if (globalDebug < 2)
          setGainRaw(gainValue);
        gain_old = gainValue;
        printf("run: video gain set to %d\n", gainValue);
        //newShutGain = true;
      }
      //
      if (imageOK and (imageNumber != imn+1))
      { // skip first image after change in image position
        // production mode
//        t1 = img->imgTime - t;
//         if (imageNumber - imn < 5 or newShutGain)
//         {
//           t1 = img->imgTime - t;
//           printf("  want to process image %d at %d, size %dx%d - time %10.4f sec T=%f\n", imageNumber, frameTop, img->width(), img->height(), t1, t1-t2);
//         }
//        t2 = t1;
        /// do some image processing
        if (imageNumber > 50)
        {
          float row, col;
          if (globalDebug >= 2)
          {  // find in full image
            if (simValid and globalDebug == 2)
            { // use simulated ball position
              row = simBallPosY;
              col = simBallPosX;
              ballQuality = 1000;
              ballQualityBlueRed = 50;
              ballTime = simBallTime;
            }
            else
            { // find ball in image
              ballOK = findBall(imgFull, frameTop, frameTop + frameHeight, 0, imgFull->width(), &row, &col);
            }
            row -= frameTop;
          }
          else
          { // find in camera image only
            if (true) // not ballOK)
              // look in full image
              ballOK = findBall(img, 0, img->height(), 0, img->width(), &row, &col);
            else
              // look near old ball position only
              ballOK = findBall(img, ballPosition[1] - 10, ballPosition[1] + 10,
                                     ballPosition[0] - 10, ballPosition[0] + 10, &row, &col);
          }
          if (ballOK)
          {
            ballPosition[0] = col;
            ballPosition[1] = row;
          }
          // post to control thread that new data is available
          semBallPosition.clearPosts();
          semBallPosition.post();
          // log ball position
          if (flog.isOpen())
          {
            fprintf(flog.getF(), "%ld.%06ld %d %.2f %.2f %d %d  %d %d\n",
                    ballTime.getSec(), ballTime.GetMicrosec(), ballOK,
                    ballPosition[1], ballPosition[0], ballQuality, 
                    ballQualityBlueRed, frameTop, frameHeight);
          }
          if (ballOK)
          { // change roi for camera - when needed
            if (not initialImage and tRoi.getTimePassed() > 0.3)
            { // follow ball
              if (row > frameHeight * 0.75 and (frameTop + frameHeight) < roi_hgtMax)
              {  // change ROI up
                setROIrequest(frameTop + frameHeight / 2, frameHeight);
                printf("moving down\n");
              }
              else if (row < frameHeight * 0.25 and frameTop > 0)
              { // change ROI up
                setROIrequest(frameTop - frameHeight / 2, frameHeight);
                printf("moving up\n");
              }
            }
          }
        }
        /// ////////////////////////
        // update full image if new top - or maybe all the time
        img->updated();
        if ((frameTop == newTop) or (tf.getTimePassed() > 0.5))
        { // copy new image part to overview image
          //copyToFullImage(img);
          char * dst, *src;
          int n;
          imgFull->lock();
          // copy image time, name and such
          imgFull->copyMeta(img, false);
          dst = (char*) imgFull->getLine(frameTop + 1);
          src = (char*) img->getLine(0 + 1);
          n = img->getDataSize() - img->width();
          if (globalDebug < 2)
            memcpy(dst, src, n);
          imgFull->valid = true;
          imgFull->updated();
          imgFull->unlock();
          if (initialImage and frameTop == roi_top)
            initialImageCount++;
          // changed ROI has now updated GUI image
          newTop = -1;
//          printf("run: added new part of full image (top=%d image number %d)\n", frameTop, imageNumber);
          if (initialImage and initialImageCount > 2)
          {
            if (frameHeight + frameTop >= roi_hgtMax)
            { // set ROI back to start position
              setROIrequest(framePoints[0][0], frameHeight);
              initialImage = false;
            }
            else
              setROIrequest(frameTop + frameHeight - 10, frameHeight);
            initialImageCount = 0;
          }
          tf.now();
        }
//        newShutGain = false;
        //
        double dt = img->imgTime - tLastFrame;
        if (dt > 5.0)  // 5
        {
          frameRate = (imageNumber - lastImageNumber) / dt;
          tLastFrame.now();
          lastImageNumber = imageNumber;
        }

      }
      // wait a bit
      // Wait(0.001);
    }
    printf("Closing cam\n");
    flog.closeLog();
    if (globalDebug < 2)
    {
      closeDevice();
      if (camHandle != NULL)
      { // release device structures
        dc1394_capture_stop(camHandle);
        dc1394_camera_free(camHandle);
        printf("~UCamDevGuppy closed camera handle\n");
        camHandle = NULL;
      }
      if (devPlatform != NULL)
      {
        dc1394_free(devPlatform);
        devPlatform = NULL;
        //printf("~UCamDevGuppy closed device handle\n");
      }
      saveImage(imgFull);
    }
  }
}

/////////////////////////////////////////////

bool startCamera()
{
  const char * fullimage = "BGGRimgFull.bmp";
  if (imgFull == NULL)
    imgFull = new UImage();
  imgFull->setSize(480, 752, 1, 8, "gray");
  if (globalDebug >= 2)
  { // load from file rather than using live image
    bool isOK = imgFull->loadBMP(fullimage);
    if (not isOK)
      printf("Failed to load %s as full image\n", fullimage);
    imgFull->imgTime.now();
    imgFull->cam = NULL;
    imgFull->camDevice = 10;
    imgFull->valid = true;
    imgFull->updated();
    imgFull->savePNG("debug-full-image.png");
  }
  return start();
}

///////////////////////////////////////////

void stopCamera()
{
  printf("Waiting for camera to close ...\n");
  stop(true);
  printf("            camera and images released.\n");
  if (imgFull != NULL)
  {
    delete imgFull;
    imgFull = NULL;
  }
}

///////////////////////////////////////////

int getIeee1394PortCnt()
{
  const int MAX_IEEE1394_PORTS = 10;
  struct raw1394_portinfo ports[MAX_IEEE1394_PORTS];
  int portCnt = 0;
  raw1394handle_t handle;
  //
  handle = raw1394_new_handle();
  if (handle==NULL)
  {
    fprintf( stderr, "Unable to aquire a raw1394 handle - no camera or no rights?\n");
/*        "Please check \n"
            "  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded \n"
            "  - if you have read/write access to /dev/raw1394\n\n");*/
  }
  /* get the number of ports (cards) and write info (up to 10) into 'ports' */
  if (handle != NULL)
  {
    portCnt = raw1394_get_port_info(handle, ports, MAX_IEEE1394_PORTS);
    raw1394_destroy_handle(handle);
  }
  /* look across all ports for cameras */
  return portCnt;
}

///////////////////////////////////////////

int getDc1394CamCnt()
{
  int camCnt = 0;
  dc1394camera_list_t * list = NULL;
  dc1394error_t err = DC1394_SUCCESS;
  //
  if (devPlatform == NULL)
  {
    devPlatform = dc1394_new ();
    if (devPlatform == NULL)
    {
      printf("getDc1394CamCnt failed to get access to /dev/raw1394\n");
    }
  }
  //
  if (devPlatform != NULL)
  {
    err = dc1394_camera_enumerate(devPlatform, &list);
    if (err != DC1394_SUCCESS)
      printf("getDc1394CamCnt failed to enumerate cameras\n");
      // DC1394_ERR_RTN(err,"Failed to enumerate cameras");
  }
  if (devPlatform != NULL and err == DC1394_SUCCESS)
  {
/*    printf("getDc1394CamCnt found %d cameras\n", list->num);
    for (int i = 0; i < (int)list->num; i++)
      printf("  - %d ID: %llx\n", i, list->ids[i].guid);*/
    if (list->num == 0)
    {
        dc1394_log_error("No cameras found");
        return 1;
    }
    camCnt = list->num;
  }
  if (list != NULL)
    dc1394_camera_free_list(list);
  return camCnt;
}


////////////////////////////////////////////

bool setCamDeviceNode(int camNum)
{ // this is to test that the camera exist and is working
  // and is not called after camera detect is finished (except for >> camsget all)
  bool result = false;
  //dc1394_t * d;
  dc1394camera_list_t * list;
  dc1394error_t err = DC1394_SUCCESS;
//   dc1394video_modes_t modes;
//   dc1394featureset_t allFeatures;
  int c = getDc1394CamCnt();
    //printf(" - port %d has %d camera node(s)\n", i, c);
    // test if camera device number is on this interface card
  result = c > camNum;
  if (not result)
    printf("No camera number %d (found only %d cameras\n", camNum, c);
  if (result)
  {
    if (devPlatform == NULL)
    {
      devPlatform = dc1394_new();
      if (devPlatform == NULL)
        printf("setCamDeviceNode failed to access /dev/fw0\n");
      cameraOpen = false;
    }
    //
    if (devPlatform != NULL)
      err = dc1394_camera_enumerate(devPlatform, &list);
    result &= (err == DC1394_SUCCESS);
    if (err != DC1394_SUCCESS)
      printf("setCamDeviceNode failed to enumerate cameras\n");
  }
    // DC1394_ERR_RTN(err,"Failed to enumerate cameras");
  if (result and list->num > 0)
  { // get handle to camera bus (port)
    camHandle = dc1394_camera_new(devPlatform, list->ids[camNum].guid);
    if (camHandle == NULL)
      printf("setCamDeviceNode failed to initialize camera %d with GUID=%lx\n", camNum, (long int)list->ids[camNum].guid);
    if (camHandle != NULL)
    { // handle is OK,
      // debug
// //       dc1394_camera_print_info(camHandle, stdout);
// //       dc1394_feature_get_all(camHandle, &allFeatures);
// //       dc1394_feature_print_all(&allFeatures, stdout);
      // debug end
      // printf("Device is a %s %s\n", camHandle->vendor, camHandle->model);
      snprintf(camName, MAZ_CAM_NAME_CNT, "%s %lx", camHandle->model, (long int)camHandle->guid);
      // release any existing iso channels
      release_iso_and_bw();
      // get camera get mode to use
      err = dc1394_video_get_supported_modes(camHandle, &modes);
      if (err == DC1394_SUCCESS)
      {
        // debug
//         for (int i=0; i < (int)modes.num; i++)
//         {
//           selected_mode = modes.modes[i];
//           print_mode_info(camHandle, selected_mode);
//         }
        // debug end
        selected_mode = modes.modes[modes.num - 1];
        print_mode_info(camHandle, selected_mode);
        result = true;
      }
      //
      err = dc1394_external_trigger_set_mode(camHandle, DC1394_TRIGGER_MODE_0);
      if (err != DC1394_SUCCESS)
        printf("Could not set trigger mode\n");
//      err = dc1394_feature_set_power(camHandle, DC1394_FEATURE_TRIGGER, DC1394_ON);
      err = dc1394_feature_set_power(camHandle, DC1394_FEATURE_TRIGGER, DC1394_OFF);
      if (err != DC1394_SUCCESS)
        printf("Could not set trigger on-off\n");
      err = dc1394_feature_set_value(camHandle, DC1394_FEATURE_EXPOSURE, 100);
      if (err != DC1394_SUCCESS)
        printf("Could not set exposure\n");
      err = dc1394_feature_set_value(camHandle, DC1394_FEATURE_BRIGHTNESS, 200);
      if (err != DC1394_SUCCESS)
        printf("Could not set brightness\n");
      result &= (err == DC1394_SUCCESS);
      err = dc1394_format7_get_unit_position(camHandle, selected_mode, &position_unit[0], &position_unit[1]);
      if (err != DC1394_SUCCESS)
        printf("Could not get position unit\n");
      err = dc1394_format7_get_max_image_size(camHandle, selected_mode, &size_max[0], &size_max[1]);
      if (err != DC1394_SUCCESS)
        printf("Could not get max size\n");
      else
      {
        roi_widthMax = size_max[0];
        roi_hgtMax = size_max[1];
      }
      err = dc1394_format7_get_unit_size(camHandle, selected_mode, &size_unit[0], &size_unit[1]);
      if (err != DC1394_SUCCESS)
        printf("Could not get size unit\n");
      printf("setCamDeviceNode: dc1394 say top position must be in increments of %d and height must be in increments of %d\n", position_unit[1], size_unit[1]);
      printf("setCamDeviceNode: But height must be minimum 60\n");

      //dc1394_feature_get_all(camHandle, &allFeatures);
      //dc1394_feature_print_all(&allFeatures, stdout);
    }
  }
  dc1394_camera_free_list(list);
  //dc1394_free(d);
  return result;
}

////////////////////////////////////////////////////////////////

void release_iso_and_bw()
{
  uint32_t val;
  dc1394error_t err = DC1394_SUCCESS;

  err = dc1394_video_get_bandwidth_usage(camHandle, &val);
  if (err == DC1394_SUCCESS)
  {
    err = dc1394_iso_release_bandwidth(camHandle, val);
    //printf("release_iso_and_bw: released (%s) %d bytes/sec? of bandwidth for device %d\n",
    //       bool2str(err == DC1394_SUCCESS), val, devNum);
  }

  err = dc1394_video_get_iso_channel(camHandle, &val);
  if (err == DC1394_SUCCESS)
  {
    printf(" - A failure to  channel is OK\n");
    err = dc1394_iso_release_channel(camHandle, val);
    //printf("release_iso_and_bw: released (%s) iso channel (value %d) for device %d\n",
    //       bool2str(err == DC1394_SUCCESS), val, devNum);
  }
}

////////////////////////////////////////////

void print_format(uint32_t format)
{
#define print_case(A) case A: printf(#A ""); break;

    switch( format ) {
        print_case(DC1394_VIDEO_MODE_160x120_YUV444);
        print_case(DC1394_VIDEO_MODE_320x240_YUV422);
        print_case(DC1394_VIDEO_MODE_640x480_YUV411);
        print_case(DC1394_VIDEO_MODE_640x480_YUV422);
        print_case(DC1394_VIDEO_MODE_640x480_RGB8);
        print_case(DC1394_VIDEO_MODE_640x480_MONO8);
        print_case(DC1394_VIDEO_MODE_640x480_MONO16);
        print_case(DC1394_VIDEO_MODE_800x600_YUV422);
        print_case(DC1394_VIDEO_MODE_800x600_RGB8);
        print_case(DC1394_VIDEO_MODE_800x600_MONO8);
        print_case(DC1394_VIDEO_MODE_1024x768_YUV422);
        print_case(DC1394_VIDEO_MODE_1024x768_RGB8);
        print_case(DC1394_VIDEO_MODE_1024x768_MONO8);
        print_case(DC1394_VIDEO_MODE_800x600_MONO16);
        print_case(DC1394_VIDEO_MODE_1024x768_MONO16);
        print_case(DC1394_VIDEO_MODE_1280x960_YUV422);
        print_case(DC1394_VIDEO_MODE_1280x960_RGB8);
        print_case(DC1394_VIDEO_MODE_1280x960_MONO8);
        print_case(DC1394_VIDEO_MODE_1600x1200_YUV422);
        print_case(DC1394_VIDEO_MODE_1600x1200_RGB8);
        print_case(DC1394_VIDEO_MODE_1600x1200_MONO8);
        print_case(DC1394_VIDEO_MODE_1280x960_MONO16);
        print_case(DC1394_VIDEO_MODE_1600x1200_MONO16);
        print_case(DC1394_VIDEO_MODE_FORMAT7_0);
        print_case(DC1394_VIDEO_MODE_FORMAT7_1);
        print_case(DC1394_VIDEO_MODE_FORMAT7_2);
        print_case(DC1394_VIDEO_MODE_FORMAT7_3);
        print_case(DC1394_VIDEO_MODE_FORMAT7_4);
        print_case(DC1394_VIDEO_MODE_FORMAT7_5);
        print_case(DC1394_VIDEO_MODE_FORMAT7_6);
        print_case(DC1394_VIDEO_MODE_FORMAT7_7);

    default:
        dc1394_log_error("Unknown format\n");
        break;
    }

}

////////////////////////////////////////////////////

void print_mode_info( dc1394camera_t *camera , uint32_t mode )
{
  int j;
  dc1394video_mode_t vmode = (dc1394video_mode_t)mode;

/*  printf("Mode: ");
  print_format(mode);
  printf("\n");*/
  if (vmode < DC1394_VIDEO_MODE_FORMAT7_MIN)
  { // format 7 has no framerate?
    dc1394framerates_t framerates;
    dc1394error_t err;
    err=dc1394_video_get_supported_framerates(camera, vmode, &framerates);
    if (err != 0)
      printf("print_mode_info failed to get framerate\n");
    else
    {
      printf("Frame Rates:\n");
      for( j = 0; j < (int)framerates.num; j++ )
      {
        dc1394framerate_t rate = framerates.framerates[j];
        float f_rate;
        dc1394_framerate_as_float(rate,&f_rate);
        printf("  [%d] rate = %f\n",j,f_rate );
      }
    }
  }
}



////////////////////////////////////////////////////////

bool getSingleImage(UImage * destination)
{
  bool result;
  int bytes, n;
  dc1394error_t err;
  UImage * img = destination;
  unsigned int depth;
  dc1394video_frame_t *frame=NULL;
  uint64_t ts, tu;
  UTime t;
  //uint64_t bCnt;
  //

//   err = dc1394_format7_get_total_bytes(camHandle, selected_mode, &bCnt);
//   if (err != DC1394_SUCCESS)
//     printf("failed to get image size in bytes\n");
  //
  err = dc1394_capture_dequeue(camHandle, DC1394_CAPTURE_POLICY_WAIT, &frame);
  //err = dc1394_capture_dequeue(camHandle, DC1394_CAPTURE_POLICY_POLL, &frame);
  result = err == DC1394_SUCCESS and frame != NULL;
  if (err != DC1394_SUCCESS)
    printf("getSingleImage: Could not dequeue (or poll for) an image frame\n");

  if (result)
  { // frame time
    ts = frame->timestamp / 1000000;
    tu = frame->timestamp - ts * 1000000;
    t.setTime(ts, tu);
    //
    if (img != NULL and img->tryLock())
    {
//       frameWidth = w; // frame->size[0];
//       frameHeight = h; // frame->size[1];
      depth = frame->data_depth; // mono8 => 8 bit depth
      // frame->color_coding == DC1394_COLOR_CODING_MONO8
      // frame->color_filter == DC1394_COLOR_CODING_RGGB
      // frame->little_endian == DC1394_FALSE
      //dc1394_get_image_size_from_video_mode(camHandle, selected_mode, &width, &height);
      frameWidth = frame->size[0];
      frameHeight = frame->size[1];
      frameTop = frame->position[1];
      img->setSize(frameHeight, frameWidth, 1, depth, "BGGR");
      n = frameHeight * frameWidth * depth / 8;
      //n = frame->image_bytes;
      //packetSize = frame->packet_size;
      //
//       if (imageNumber % 500 == 0)
//         printf("getSingleImage: number %6d at %lu.%06lu size (hxW) %dx%d, pos (top,left) %dx,%dy, depth=%d (pkg-size=%d)\n",
//              imageNumber, t.getSec(), t.getMicrosec(),
//              frameHeight, frameWidth, frame->position[1], frame->position[0],
//              depth, frame->packet_size);
      //
      bytes = img->getBufferSize();
      if (n <= bytes)
      {
        memcpy(img->getData(), frame->image, n);
      }
      img->camDevice = 10;
      img->cam = NULL;
      img->imgTime = t;
      img->imageNumber = ++imageNumber;
      img->valid = true;
      img->unlock();
    }
    else
    { // no image to get - wait a bit and retry
      Wait(0.0011);
    }
    if (false)
    {
      uint w, h;
      err = dc1394_format7_get_image_size(camHandle, selected_mode, &w, &h);
      if (err != DC1394_SUCCESS)
        printf("getSingleImage: failed to get image size in HxW\n");
      else
      {
        printf("getSingleImage: camera say ROI is %dx, %dy, %dw, %dh\n", 0, roi_top, w, h);
      }
    }
    // debug
//     if (roi_top > 300)
//     {
//       printf("debug break\n");
//     }
    // debug end
    // put buffer back in camera queue
    err = dc1394_capture_enqueue(camHandle, frame);
    if (err != DC1394_SUCCESS)
      printf("getSingleImage: could not return image buffer\n");
  }
  return result;
}

/////////////////////////////////////////////////////////////


bool startIsoTransmission(int isoBw, int packetSize)
{
  dc1394error_t err;
  /*-----------------------------------------------------------------------
    *  setup capture
    *-----------------------------------------------------------------------*/
  //printf("Starting video-mode, capture and video transmission (dev %d)\n", devNum);
  switch (isoBW)
  {
    case 100: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_100); break;
    case 200: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_200); break;
    case 400: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_400); break;
    case 800: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_800); break;
    case 1600: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_1600); break;
    case 3200: err = dc1394_video_set_iso_speed(camHandle, DC1394_ISO_SPEED_3200); break;
  }
  if (err != DC1394_SUCCESS)
    printf("startIsoTransmission: Could not set ISO BW (to%d)\n", isoBW);
  // max size is determined by iso_speed:
// The IEEE 1394 specification imposes the following limits on the payload size of isochronous packets:
// Speed, Cycle, Max. packet size, Max. bandwidth per ISO channel
// [Mb/s]  [µs]    [B]     [MB/s]  [MiB/s]
// S100    125     1024    8.192   7.8125
// S200    125     2048    16.384  15.6250   --- used s200 for guppy to allow 2cameras on one s400 connection
// S400    125     4096    32.768  31.2500
// S800    125     8192    65.536  62.5000
// S1600   125     16384   131.072 125.0000
// S3200   125     32768   262.144 250.0000
  err = dc1394_format7_set_packet_size(camHandle, selected_mode, packetSize);
  if (err != DC1394_SUCCESS)
    printf("startIsoTransmission: Could not set packet size to %d\n", packetSize);
  //
  err = dc1394_video_set_mode(camHandle, selected_mode);
  if (err != DC1394_SUCCESS)
    printf("startIsoTransmission: Could not set video mode FORMAT_7_0\n");

  if (err == DC1394_SUCCESS)
  {
    err = dc1394_capture_setup(camHandle, 14, DC1394_CAPTURE_FLAGS_DEFAULT);
    if (err != DC1394_SUCCESS and err != DC1394_CAPTURE_IS_RUNNING)
    {
      printf("startIsoTransmission: Could not setup camera\n");
      // DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not setup camera-\nmake sure that the video mode and framerate are\nsupported by your camera\n");
    }
  }
  if (err == DC1394_SUCCESS)
  {
    err = dc1394_video_set_multi_shot(camHandle, 14, DC1394_OFF);
    if (err != DC1394_SUCCESS)
    {
      printf("startIsoTransmission: Could not stop multi_shot mode??\n");
      // DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not setup camera-\nmake sure that the video mode and framerate are\nsupported by your camera\n");
    }
  }

  if (err == DC1394_SUCCESS)
  {
    /*-----------------------------------------------------------------------
      *  have the camera start sending us data
      *-----------------------------------------------------------------------*/
    err = dc1394_video_set_transmission(camHandle, DC1394_ON);
    if (err != DC1394_SUCCESS)
      printf("startIsoTransmission: Could not start camera iso transmission\n");
    cameraOpen = (err == DC1394_SUCCESS);
  }
  else if (err == DC1394_CAPTURE_IS_RUNNING)
    cameraOpen = true;
  return cameraOpen;
}

//////////////////////////////////////////////////////////////

bool stopIsoTransmission()
{
  dc1394error_t err;
  bool result = true;
  //
  if (cameraOpen)
  {
    cameraOpen = false;
    // wait for any last image to be captured
    Wait(0.2);
    /*Step 6: Stop sending data*/
    dc1394_capture_stop(camHandle);
    err = dc1394_video_set_transmission(camHandle,DC1394_OFF);
    result = err == DC1394_SUCCESS;
    if (not result)
      printf("stopIsoTransmission: could not stop the camera\n");
  }
  else
    printf("stopIsoTransmission(): camera is closed already\n");

  return result;
}

///////////////////////////////////////////////////////////

bool setGainRaw(int agc)
{
//   int v;
//   dc1394_feature_info * feature;
  dc1394error_t err;
  //
  if (agc == -1)
  {
    if (fGain.id == 0)
      getGainRaw();
    if (fGain.current_mode != DC1394_FEATURE_MODE_AUTO)
    {
      err = dc1394_feature_set_mode(camHandle, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_AUTO);
      if (err != DC1394_SUCCESS)
        printf("setGain: failed to set gain to auto\n");
      err = dc1394_feature_get_mode(camHandle, DC1394_FEATURE_GAIN, &fGain.current_mode);
    }
    else
      err = DC1394_SUCCESS;
  }
  else
    err = dc1394_feature_set_value(camHandle, DC1394_FEATURE_GAIN, agc);
  if (err != DC1394_SUCCESS)
    printf("setGain: to %d failed\n", agc);
   return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////////


bool getGainRaw()
{
  dc1394error_t err;
  //
  fGain.id = DC1394_FEATURE_GAIN;
  err = dc1394_feature_get(camHandle, &fGain);
  if (err != DC1394_SUCCESS)
    printf("getGain: Failed to get gain feature info\n");
  //
//   fGain.value;
//   fGain.min;
//   fGain.max;
//   fGain.current_mode == DC1394_FEATURE_MODE_AUTO;
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////////

bool setShutterRaw(int value)
{
  dc1394error_t err;
  if (value == -1)
  {
    err = DC1394_FAILURE;
    if (fShutter.id == 0)
      getShutterRaw();
    if (fShutter.current_mode != DC1394_FEATURE_MODE_AUTO)
    {
      err = dc1394_feature_set_mode(camHandle, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_AUTO);
      err = dc1394_feature_get_mode(camHandle, DC1394_FEATURE_SHUTTER, &fShutter.current_mode);
    }
  }
  else
  {
    if (fShutter.current_mode == DC1394_FEATURE_MODE_AUTO)
    {
      err = dc1394_feature_set_mode(camHandle, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
      if (err != DC1394_SUCCESS)
        printf("setShutter: failed to change to manuel mode\n");
    }
    err = dc1394_feature_set_value(camHandle, DC1394_FEATURE_SHUTTER, value);
    err = dc1394_feature_get_value(camHandle, DC1394_FEATURE_SHUTTER, &fShutter.value);
  }
  if (err != DC1394_SUCCESS)
    printf("setGain: to %d failed\n", value);
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////////

bool getShutterRaw()
{
  dc1394error_t err;
  //
  fShutter.id = DC1394_FEATURE_SHUTTER;
  err = dc1394_feature_get(camHandle, &fShutter);
  if (err != DC1394_SUCCESS)
    printf("getGain: Failed to get gain feature info\n");
  //
// fShutter.value;
// fShutter.min;
// fShutter.max;
// fShutter.current_mode == DC1394_FEATURE_MODE_AUTO;
  return err == DC1394_SUCCESS;
}


////////////////////////////////////////////////////////


bool getWhiteBalanceRaw()
{ // get red and blue gain, and mode (4=auto, 3=manual)
  dc1394error_t err;
  //
  fWhite.id = DC1394_FEATURE_WHITE_BALANCE;
  err = dc1394_feature_get(camHandle, &fWhite);
  if (err != DC1394_SUCCESS)
    printf("getGain: Failed to get gain feature info\n");
  //
// fWhite.min;
// fWhite.max;
// fWhite.RV_value;
// fWhite.BU_value;
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////////////////

bool setWhiteBalanceRaw(int mode, int red, int blue)
{ // set mode (4=auto, 3=manual), red and blue gain
  dc1394error_t err;
  printf("Setting whitebal to mode=%d, red=%d blue=%d\n", mode, red, blue);
  if (mode != 3)
  {
    err = DC1394_FAILURE;
    if (fWhite.id == 0)
      getWhiteBalanceRaw();
    err = dc1394_feature_set_mode(camHandle, DC1394_FEATURE_WHITE_BALANCE, DC1394_FEATURE_MODE_ONE_PUSH_AUTO);
    if (err != DC1394_SUCCESS)
      printf("setWhite: failed to set white-bal one-push auto\n");
    //err = dc1394_feature_get_mode(camHandle, DC1394_FEATURE_WHITE_BALANCE, &fWhite.current_mode);
  }
  else
  { // manuel mode
    err = dc1394_feature_whitebalance_set_value(camHandle, blue, red);
    err = dc1394_feature_whitebalance_get_value(camHandle, &fWhite.BU_value, &fWhite.RV_value);
  }
  if (err != DC1394_SUCCESS)
    printf("setWhite balance: to mode=%d (3=manual, 4=auto), red=%d, blue=%d failed\n", mode, red, blue);
  return err == DC1394_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////

bool openDevice(int idx)
{
  dc1394error_t err;
  bool result;
  unsigned int width, height;
  //
  result = camHandle != NULL;
  if (frameRate <= 0)
    frameRate = 100;
  if (result)
  {
    getShutterRaw();
    getGainRaw();
    printf("shutter range %d-%d, gain range %d-%d\n", fShutter.min, fShutter.max, fGain.min, fGain.max);
    result = startIsoTransmission(isoBW, packetSize);
    if (result and frameHeight <= 0)
    { // no size is selected, so use as is.
      err = dc1394_get_image_size_from_video_mode(camHandle, selected_mode, &width, &height);
      result = err == DC1394_SUCCESS;
      if (width > 0 and result)
      { // set current image size
        frameHeight = (int)height;
        frameWidth = (int)width;
      }
    }
  }
  return result;
}

////////////////////////////////////////////////////

void closeDevice()
{
  if (cameraOpen)
  { // stop iso_transmission
    stopIsoTransmission();
    closeDeviceRelease();
  }
}

void closeDeviceRelease()
{
  if (camHandle != NULL)
  {
    dc1394_capture_stop(camHandle);
    dc1394_camera_free(camHandle);
    printf("~UCamDevGuppy closed camera handle\n");
    camHandle = NULL;
  }
  if (devPlatform != NULL)
  {
    dc1394_free(devPlatform);
    devPlatform = NULL;
    //printf("~UCamDevGuppy closed device handle\n");
  }
}

////////////////////////////////////////////////////

void setROIrequest(int top, int height)
{
  roi_lock.lock();
  //
  if (height < roi_hgtMin)
    roi_hgt = roi_hgtMin;
  else if (height > roi_hgtMax)
    roi_hgt = roi_hgtMax;
  //
  if (top < 0)
    roi_top = 0;
  else if (top >= roi_hgtMax - height)
    roi_top = roi_hgtMax - height;
  else
    roi_top = top;
  //
  roi_lock.unlock();
}

///////////////////////////////////////////////

bool setROI(int top, int hgt)
{
  // change ROI
  dc1394error_t err = DC1394_FAILURE;
  uint32_t w, h;
  bool restart = false;
//  uint32_t l, t;
  if (camHandle != NULL)
  {
    roi_lock.lock();
    err = dc1394_format7_get_image_size(camHandle, selected_mode, &w, &h);
    if (err != DC1394_SUCCESS)
      printf("setROI: failed to get image size in HxW\n");
    if ((int)h != hgt)
    {
      printf("setROI: setting RIO to %dtop, %dhgt\n", top, hgt);
      err = dc1394_format7_set_roi(camHandle, selected_mode,  DC1394_COLOR_CODING_RAW8,
                packetSize,
                0 /*left*/, top /*top*/, roi_widthMax /* width*/, hgt /*height*/ );
      if (err != DC1394_SUCCESS)
        printf("setROI: failed to set ROI to %dtop, %dhgt\n", top, hgt);
      restart = true;
    }
    else
    {
      printf("setROI: setting position to %dtop only\n", top);
      err = dc1394_format7_set_image_position(camHandle, selected_mode, 0, top);
      if (err != DC1394_SUCCESS)
        printf("setROI: failed to set image top to %d\n", top);
      // should be possible without, but
      restart = true;
    }

// debug
    if (restart)
    { // transmission restart is needed
      dc1394_capture_stop(camHandle);
      err = dc1394_video_set_transmission(camHandle,DC1394_OFF);
      if (err != DC1394_SUCCESS)
        printf("setROI: Could not stop camera iso transmission\n");
      err = dc1394_video_set_mode(camHandle, selected_mode);
      if (err != DC1394_SUCCESS)
        printf("startIsoTransmission: Could not set video mode FORMAT_7_0\n");

      if (err == DC1394_SUCCESS)
      {
        err = dc1394_capture_setup(camHandle, 14, DC1394_CAPTURE_FLAGS_DEFAULT);
        if (err != DC1394_SUCCESS and err != DC1394_CAPTURE_IS_RUNNING)
        {
          printf("startIsoTransmission: Could not setup camera\n");
          // DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not setup camera-\nmake sure that the video mode and framerate are\nsupported by your camera\n");
        }
      }
      // trying to restart iso with new ROI
      err = dc1394_video_set_transmission(camHandle, DC1394_ON);
      if (err != DC1394_SUCCESS)
        printf("setROI: Could not start camera iso transmission\n");
    }
// debug end

//     if (false)
//     {
//       err = dc1394_format7_get_image_size(camHandle, selected_mode, &w, &h);
//       if (err != DC1394_SUCCESS)
//         printf("setROI: failed to get image size in HxW\n");
//       err = dc1394_format7_get_image_position(camHandle, selected_mode, &l, &t);
//       if (err != DC1394_SUCCESS)
//         printf("setROI: failed to get image position in Top,Left\n");
//       //
//       roi_top = top;
//       roi_hgt = h;
//       printf("setROI: camera say ROI is %dtop, %dleft, %dwidth, %dheight\n", t, l, w, h);
//     }
//     frameHeight = h;
//     frameWidth = w;
//     frameTop = t;
    roi_lock.unlock();
  }
  return err == DC1394_SUCCESS;
}

///////////////////////////////////////////////////

void * runIeee1394CamThread(void * camobj)
{ // Start thread here and call thread function
  //looptest();
  stopFrameRead = false;
  run();
  pthread_exit((void*)NULL);
  return NULL;
}

////////////////////////////////////////////////////

pthread_attr_t  thAttr;

bool start()
{
  pthread_attr_t  thAttr;
  int i = 0;
  //
  if (not threadRunning)
  { // start thread to read and timestamp images
    pthread_attr_init(&thAttr);
    //
    stopFrameRead = false;
    // create socket server thread
    if (pthread_create(&thRead, &thAttr, &runIeee1394CamThread, NULL) != 0)
      // report error
      perror(camName);
      // wait for thread to initialize
    while ((not threadRunning) and (++i < 100))
      Wait(0.05);
    // test for started thread
    if (not threadRunning)
    { // failed to start
      stopFrameRead = true;
      printf("Failed to start read thread\n");
    }
//    pthread_attr_destroy(&thAttr);
  }
  return threadRunning;
}

//////////////////////////////////////////////////////////////

void stop(bool andWait)
{
  if (threadRunning)
  {
    stopFrameRead = true;
    if (andWait and threadRunning)
      pthread_join(thRead, NULL);
    // debug
    printf("stop: read thread stopped\n");
  }
}

///////////////////////////////////////////////////////////////

bool saveImage(UImage * img)
{
  const int MSL = 500;
  char s[MSL];
  char s2[MSL];
  bool isOK = false;
  if (img != NULL)
  {
    img->imgTime.getForFilename(s2);
    snprintf(s, MSL, "data/imgguppy%06d_%s.png", imageNumber, s2);// s2);//imgfwguppy%06d_%s.png"
  //    snprintf(s, MSL, "/misc/shome/ex25/Dropbox/Guppy/fisk/imgguppy%06d.png", imageNumber);// s2);//imgfwguppy%06d_%s.png"
    isOK = img->savePNG(s);
  }
  return isOK;
}

////////////////////////////////////////////////////////////

/**
 * Do processing of new usefull image */
// void processImage(UImage * img)
// {
//   // printf("got image at %lu.%06lu of size %dx%d\n", img->imgTime.getSec(), img->imgTime.getMicrosec(), img->height(), img->width());
//   //
//   double dt = tLastFrame.getTimePassed();
//   int w = img->width();
//   int h = img->height();
//   //
//     ////////////////////////7
//   if (false)
//   {
//     if (0 < h and 0 < w)
//     {
//       FILE* imagefile4;
//       const int MSL = 500;
//       char s[MSL];
//       uint64_t numPixels = w*h;
//       uchar* num =  img->getUCharRef(0,0);
//       snprintf(s, MSL, "/misc/shome/ex25/Dropbox/Guppy/fisk/imgguppy%06d.ppm", imageNumber);
//       imagefile4=fopen(s, "wb");
//       if( imagefile4 == NULL)
//       {
//         perror( "Can't create fisk '" "/misc/shome/ex25/Dropbox/Guppy/fisk/ImageRGB1003.ppm" "'");
//       }
//       fprintf(imagefile4,"P5\n%u %u\n255\n", w, h);
//       fwrite((const char *)num, 1,numPixels, imagefile4);
//       fclose(imagefile4);
//     }
//   }
//   /////////////////////
//   //
//   printf("Image number %d\n", imageNumber);
//
//   if (dt > 5.0)  // 5
//   {
//     frameRate = (imageNumber - lastImageNumber) / dt;
//     tLastFrame.now();
//     lastImageNumber = imageNumber;
//     printf("Framerate = %.2f Hz\n", frameRate);
//     saveSample = true;
//   }
//   //int num =  img->height();//getData();//img->height();//img->width(); //img->imageNumber;
//   if(0 < w and 0 < h)
//   {
//     int Rxi=0, Ryi=0, Rzi=0,Gxi=0, Gyi=0, Gzi=0,Bxi=0, Byi=0, Bzi=0,Ballx=0,Bally=0,Ballz=100;
// //      uint64_t numPixels = w*h;
//     uchar* num =  img->getUCharRef(0,0);
//
//     for(int FooY= 2; FooY < h ; FooY+=2)
//     {  //h
//       for(int FooX= 2; (FooX  < w); FooX+=2)
//       {  //< w and FooX
//         if (Bzi < num[FooX+FooY*w])
//         {
//           Bzi=num[FooX+FooY*w];
//           Bxi=FooX;
//           Byi=FooY;
//         }
//         if (Gzi < num[FooX+FooY*w+1])
//         {
//           Gzi = num[FooX+FooY*w+1];
//           Gxi=FooX+1;
//           Gyi=FooY;
//         }
//         if (Ballz < num[FooX+FooY*w+1])
//         {
//           if ((num[FooX+FooY*w+1]-70) > num[FooX+(FooY+1)*w+1])
//           {      //mere grøn end rød
//             if ((num[FooX+FooY*w+1]-70) > num[FooX+(FooY-1)*w+1])
//             {
//               Ballx=FooX+1;
//               Bally=FooY;
//               Ballz=num[FooX+FooY*w+1];
//             }
//           }
//         }
//       }
//       // find average
//       for(int FooX = 0; (FooX < w); FooX += 2)
//       {
//         if (Ballz < num[FooX+(FooY+1)*w])
//         {
//           Gzi=num[FooX+(FooY+1)*w];
//           Gxi=FooX;
//           Gyi=FooY+1;
//         }
//         if (Ballz < num[FooX+(FooY+1)*w])
//         {
//           if ((( num[FooX+(FooY+1)*w])-70) > num[FooX+(FooY+1)*w-1])
//           {     //mere grøn end rød
//             if ((( num[FooX+(FooY+1)*w])-70) > num[FooX+(FooY+1)*w+1])
//             {
//               if((num[FooX+(FooY+0)*w-0] + num[FooX+(FooY+2)*w-0]) >
//                  (num[FooX+(FooY+1)*w-1] + num[FooX+(FooY+1)*w+1]))
//               { // if blue is > red ?? // chr
//                 Ballx=FooX;
//                 Bally=FooY+1;
//                 Ballz=num[FooX+(FooY+1)*w];
//               }
//             }
//           }
//           if (Rzi < num[FooX+(FooY+1)*w+1])
//           {
//             Rzi = num[FooX+(FooY+1)*w+1];
//             Rxi=FooX+1;
//             Ryi=FooY+1;
//           }
//         }
//       }
//     }
//     printf("R: %d %d %d G:    %d %d %d \nB: %d %d %d Ball: %d %d %d g/s %d %d\n", Rxi,Ryi,Rzi, Gxi,Gyi,Gzi, Bxi, Byi, Bzi,Ballx,Bally,Ballz,gainValue,shutterValue );
//
//     if ((Ballz >240 or Gzi >240 ) and (gainValue >16))
//     {
//       gainValue--;
//       if ((Ballz >254 or Gzi >254) and (gainValue >21))
//         gainValue-=5;
//       if (gainValue < 32)
//       {
//         gainValue *=2;
//         shutterValue /= 2;
//         setShutterRaw(shutterValue);
//       }
//       setGainRaw(gainValue);
//     }
//   }
// }

///////////////////////////////////////////////


bool setExternalTriggerRaw(bool value)
{
  dc1394error_t err = DC1394_FAILURE;
  uint32_t   flag;
  uint64_t IO_OUTP_CTRL2 = 0x1000324;
  //
  fTrigger.id = DC1394_FEATURE_TRIGGER;
  fTriggerDelay.id = DC1394_FEATURE_TRIGGER_DELAY;
  if (camHandle != NULL)
  {
    //err = dc1394_feature_get(camHandle, &fTriggerDelay);
    //if (err != DC1394_SUCCESS)
    //  printf("setExternalTrigger: Failed to get trigger delay feature info\n");
    //dc1394_feature_print(&fTriggerDelay, stdout);
    err = dc1394_feature_get(camHandle, &fTrigger);
    if (err != DC1394_SUCCESS)
      printf("setExternalTrigger: Failed to get trigger feature info\n");
    //dc1394_feature_print(&fTrigger, stdout);
  }
  if (err == DC1394_SUCCESS and fTrigger.available)
  {
    if (value != (fTrigger.is_on == DC1394_ON))
    { // need to set trigger
      if (value)
        dc1394_external_trigger_set_power(camHandle, DC1394_ON);
      else
        dc1394_external_trigger_set_power(camHandle, DC1394_OFF);
      // get new state and set global variables
      getExternalTriggerRaw();
    }
    if (value)
    { // set to external trigger - set also output 2 to allow it to control trigger from software
      // through pin 2 in the 8-pin plug (should be connected to pin 4 on this and other
      // connected cameras.
      //
      // get current state of IO_CONTROL 2
      err = dc1394_get_register(camHandle, IO_OUTP_CTRL2, &flag);
      // debug
      // printf("setExternalTrigger: is  ioCtrl (%llx) value is %x\n", IO_OUTP_CTRL2, flag);
      // debug end
      // set also output 2 to be follow pin state
      flag = 0x80000000; // mode is on      bit [0]
      //flag += 0x1 << 24; // invert output - bit [7]
      flag += 0x01 << 16; // follow pin state - bit [11-15]
      //flag += 0x1; // pin state = 1  bit [31]
      err = dc1394_set_register(camHandle, IO_OUTP_CTRL2, flag);
      // debug
      // printf("setExternalTrigger: to  ioCtrl (%llx) value is %x\n\n", IO_OUTP_CTRL2, flag);
      // debug end
      //err = dc1394_get_register(camHandle, IO_OUTP_CTRL2, &flag);
      // debug
      //printf("setExternalTrigger: now ioCtrl (%llx) value is %x\n", IO_OUTP_CTRL2, flag);
      // debug end
    }
  }
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////

bool getExternalTriggerRaw()
{
  dc1394error_t err = DC1394_FAILURE;
  //
  if (camHandle != NULL)
  {
    fTrigger.id = DC1394_FEATURE_TRIGGER;
    err = dc1394_feature_get(camHandle, &fTrigger);
    if (err != DC1394_SUCCESS)
      printf("getExternalTrigger: Failed to get trigger feature info\n");
    //dc1394_feature_print(&fTrigger, stdout);
  }
  //  fTrigger.is_on == DC1394_ON;
  return err == DC1394_SUCCESS;
}

/////////////////////////////////////////////

bool makeTriggerPulse()
{
  uint32_t   flag;
  uint64_t IO_OUTP_CTRL2 = 0x1000324;
  dc1394error_t err = DC1394_FAILURE;
  //
  if (camHandle != NULL and fTrigger.available == DC1394_TRUE)
  { // make shure noone else has an io-control function in action
    err = dc1394_get_register(camHandle, IO_OUTP_CTRL2, &flag);
    //printf("makeTriggerPulse: was  ioCtrl (0x%llx) value is 0x%x\n", IO_OUTP_CTRL2, flag);
    // set pin state (LSB bit, i.e. bit [31]) to 1
    flag |= 0x1;
    //printf("makeTriggerPulse: to   ioCtrl (0x%llx) value is 0x%x\n", IO_OUTP_CTRL2, flag);
    err = dc1394_set_register(camHandle, IO_OUTP_CTRL2, flag);
    //printf("makeTriggerPulse: got  ioCtrl (%llx) value is %x\n", IO_OUTP_CTRL2, flag);
    // and set it back to zero
    flag &= 0xfffffffe;
    err = dc1394_set_register(camHandle, IO_OUTP_CTRL2, flag);
  }
  return err == DC1394_SUCCESS;
}


///////////////////////////////////////////////

bool getExposureTargetRaw()
{
  dc1394error_t err = DC1394_FAILURE;
  //
  fExposure.id = DC1394_FEATURE_EXPOSURE;
  if (camHandle != NULL)
  {
    err = dc1394_feature_get(camHandle, &fExposure);
    if (err != DC1394_SUCCESS)
      printf("getExposure: Failed to get exposure feature info\n");
  }
//     fExposure.value;
//     fExposure.min;
//     fExposure.max;
  return err == DC1394_SUCCESS;
}

///////////////////////////////////////////////

bool setExposureTargetRaw(int value)
{
  bool supported = false;
  dc1394error_t err = DC1394_FAILURE;
  //
  supported = getExposureTargetRaw();
  if (supported)
    err = dc1394_feature_set_value(camHandle, DC1394_FEATURE_EXPOSURE, (uint32_t)mini(fExposure.max, maxi(fExposure.min, value)));
  return err == DC1394_SUCCESS;
}

////////////////////////////////////////////////

int findKugleAgain(double * x, double * y)
{
  return false;
}

//bool blueDot(UImage * img, int value)
int findKugleFirst(double * x, double * y)
{
  UImage * img = imgBuff;
  ///
  /// test if new image?
  /// flyt til image read thread, og så lad x,y være variable der kan hentes
  /// når en semafor er sat
  ///
  if(0 < img->getWidth() and 0 < img->getHeight())
  {
    int Rxi=0, Ryi=0, Rzi=0,Gxi=0, Gyi=0, Gzi=0,Bxi=0, Byi=0, Bzi=0,Ballx=0,Bally=0,Ballz=100;
//	    uint64_t numPixels = img->getWidth()*img->getHeight();
    uchar* num =  img->getUCharRef(0,0);
    for(uint FooY= 2; FooY <58 ; FooY+=2)
    {  //img->getHeight()
      for(uint FooX= 2; (FooX  < 650); FooX+=2)
      {  //< img->getWidth() and FooX
        if (Bzi < num[FooX+FooY*img->getWidth()])
        {
          Bzi=num[FooX+FooY*img->getWidth()];
          Bxi=FooX;
          Byi=FooY;
        }
        if (Gzi < num[FooX+FooY*img->getWidth()+1])
        {
          Gzi = num[FooX+FooY*img->getWidth()+1];
          Gxi=FooX+1;
          Gyi=FooY;
        }
        if (Ballz < num[FooX+FooY*img->getWidth()+1])
        {
          if ((num[FooX+FooY*img->getWidth()+1]-70) > num[FooX+(FooY+1)*img->getWidth()+1])
          {	    //mere grøn end rød
            if ((num[FooX+FooY*img->getWidth()+1]-70) > num[FooX+(FooY-1)*img->getWidth()+1])
            {
          //      if (num[FooX+(FooY-0)*img->getWidth()+0]>num[FooX+(FooY+1)*img->getWidth()+1]){
          //	if (num[FooX+(FooY-0)*img->getWidth()+2]>num[FooX+(FooY-1)*img->getWidth()+1]){
            Ballx=FooX+1;
            Bally=FooY;
            Ballz=num[FooX+FooY*img->getWidth()+1];;
          //	}
          //      }
            }
          }
        }
      }
      for(uint FooX= 0; (FooX  < 650); FooX+=2)
      {
        if (Ballz < num[FooX+(FooY+1)*img->getWidth()])
        {
          Gzi=num[FooX+(FooY+1)*img->getWidth()];
          Gxi=FooX;
          Gyi=FooY+1;
        }
        if (Ballz < num[FooX+(FooY+1)*img->getWidth()])
        {
          if ((( num[FooX+(FooY+1)*img->getWidth()])-70) > num[FooX+(FooY+1)*img->getWidth()-1])
          {	    //mere grøn end rød
            if ((( num[FooX+(FooY+1)*img->getWidth()])-70) > num[FooX+(FooY+1)*img->getWidth()+1])
            {
              if((num[FooX+(FooY+0)*img->getWidth()-0]+num[FooX+(FooY+2)*img->getWidth()-0]) >
                (num[FooX+(FooY+1)*img->getWidth()-1]+num[FooX+(FooY+1)*img->getWidth()+1]))
              {
                //  if (num[FooX+(FooY-0)*img->getWidth()+0]>num[FooX+(FooY+1)*img->getWidth()-1]){
            //	if (num[FooX+(FooY+2)*img->getWidth()+0]>num[FooX+(FooY+1)*img->getWidth()+1]){
                  Ballx=FooX;
                  Bally=FooY+1;
                  Ballz=num[FooX+(FooY+1)*img->getWidth()];;
            //	}
              //    }
              }
            }
          }
        }
        if (Rzi < num[FooX+(FooY+1)*img->getWidth()+1])
        {
          Rzi = num[FooX+(FooY+1)*img->getWidth()+1];
          Rxi=FooX+1;
          Ryi=FooY+1;
        }
      }
      /*for(uint i= 0; i < img->getWidth()+img->getHeight(); i++){
        printf("%d %d %d\n", num[i], num[i+1], num[i+2]) ;
      }*/
    }
    printf("R: %d %d %d G:    %d %d %d \nB: %d %d %d Ball: %d %d %d g/s %d %d\n", Rxi,Ryi,Rzi, Gxi,Gyi,Gzi, Bxi, Byi, Bzi,Ballx,Bally,Ballz,gainValue,shutterValue );

    if ((Ballz >240 or Gzi >240 ) and (gainValue >16))
    {
      gainValue--;
      if ((Ballz >254 or Gzi >254) and (gainValue >21))
        gainValue-=5;
      if (gainValue < 32)
      {
        gainValue *=2;
        shutterValue /= 2;
        setShutterRaw(shutterValue);
      }
      setGainRaw(gainValue);
    }
  }

  return true;
}

////////////////////////////////////////////////



//////////////////////////////////////////////

