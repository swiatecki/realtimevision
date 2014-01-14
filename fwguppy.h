/***************************************************************************
 *   Copyright (C) 2012 by DTU                                             *
 *   jca@elektro.dtu.dk                                                    *
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

/*! \file fwguppy.h
    \brief firewire camera device integration using dc1394 library version 22 (or newer)
    \author Christian Andersen <jca@elektro.dtu.dk>

*/
#ifndef FWGUPPY_H
#define FWGUPPY_H

#include <semaphore.h>
##include <ugen4/uimage.h>
#include <libraw1394/raw1394.h>
#include <dc1394/dc1394.h>

#define COLS 752
#define ROWS 480

/**
Camera and image functions
*/
  /**
   * Image of full labyrinth game (painted when ROI is available) - same format as imgBall */
  extern UImage * imgFull;
  /** raw buffer image - just the ROI part */
  extern UImage * imgBuff;
  /** camera top of image line */
  extern int frameTop;
  /** camera image height - with is always full image width */
  extern int frameHeight;
  /** current gain value 16..64 */
  extern int gainValue;
  /** current shutter value 1..4096 (should be lower than 180 for sample time about 4ms) */
  extern int shutterValue;
  /// ball position [0] is x (column) [1] is y (row) in small ROI image (imgBall)
  extern float ballPosition[2];
  /// is the ball bosition OK (ball not lost)
  extern bool ballOK;
  /// relative quality of the ball - from main search
  extern int ballQuality;
  /// Validity time for ball position
  extern UTime ballTime;
  /// difference of blue and red
  extern int ballQualityBlueRed;
  /// image number
  extern int imageNumber;
  /// compensation for green offset - (depends on actual guppy camera)
  /// forst value is added to green in even rows (BG part), second is added green value to odd rows (GR part)
  extern int greenOffset[2];
  /// ballance point for game plate to be about newtral 
  /// a number between 0 and 255 (should be about 128, 
  /// but that woulde require HW modification - not done as of 9/8/2013 /Chr
  //extern int xyBallance[2];
  /// control semaphore to start a new control cycle
  extern USemaphore semBallPosition;
  /// actual measured framerate (camera frames per second)
  extern float frameRate;
  /// initialize full image by scrolling down slize
  extern bool initialImage;
  /**
   * start camera processing thread
   * \returns true if thread is running and camera is open */
  bool startCamera();
  /**
   * stop camera processing and close camera device */
  void stopCamera();
  /**
  * Set Desired ROI - the width is always the full image width.
  * The roi_lock is used to protect roi setting
  * \param top is the first line of the desired image
  * \param height is the number of lines desired */
  void setROIrequest(int top, int height);

  /**
   * Take initial image
   using automatic illumination and full frame size */
  bool takeInitialImage();

  /**
   * find kuglen første gang
   */
  int findKugleFirst(double * x, double * y);
  /**
   * find kuglen første gang
   */
  int findKugleAgain(double * x, double * y);

  void looptest();
  //////////////////////////////////////////////////////////////////////////////


#endif
