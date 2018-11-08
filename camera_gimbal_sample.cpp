/*! @file camera_gimbal_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Camera and Gimbal Control API usage in a Linux environment.
 *  Shows example usage of camera commands and gimbal position/speed control
 *  APIs
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "camera_gimbal_sample.hpp"
#include <math.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

float prevX=0,prevY=0;
float xSpeed=0,ySpeed=0;
/*float yawMapping(float gaze_normX)
{
    float gimbleYawAngle;
    float slope;
    slope = (500-0)/(1-0.5);
    gimbleYawAngle = 0 + slope * ( gaze_normX - 0.5 );
    //cout <<"Gaze Value:"<<gazeValue<<"Gimble Angle:"<<gimbleAngle<<"\n";
    return gimbleYawAngle;
}

float pitchMapping(float gaze_normY)
{
    float gimblePitchAngle;
    float slope;
    slope = (300-0)/(1-0.5);
    gimblePitchAngle = 0 + slope * ( gaze_normY - 0.5 );
    //cout <<"Gaze Value:"<<gazeValue<<"Gimble Angle:"<<gimbleAngle<<"\n";
    return gimblePitchAngle;
}*/

float yawMapping(float gaze_normX)
{
   /* float gimbleYawAngle;
    float slope;
    slope = (500-0)/(1-0.5);
    gimbleYawAngle = 0 + slope * ( gaze_normX - 0.5 );
    //cout <<"Gaze Value:"<<gazeValue<<"Gimble Angle:"<<gimbleAngle<<"\n";
    return gimbleYawAngle;*/
    
    if(gaze_normX < 0.3)
    {
	if(prevX == 0){
        	prevX = -500;
		xSpeed = -1800;	
		return -500;
	}
	if(prevX == 500){
		prevX = -500;
		xSpeed = -1800;
		return -1000;
	}
	if(prevX == -500){
		prevX = -500;
		xSpeed = 0;
		return 0;
	}
	
    }
    else if(gaze_normX > 0.7)
    {
	        
	if(prevX == 0){
        	prevX = 500;
		xSpeed = 1800;	
		return 500;
	}
	if(prevX == 500){
		prevX = 500;
		xSpeed = 0;
		return 0;
	}
	if(prevX == -500){
		prevX = 500;
		xSpeed = 1800;
		return 1000;
	}
	
    }
    else
    {
	
	if(prevX == 0){
        	prevX = 0;
		xSpeed = 0;	
		return 0;
	}
	if(prevX == 500){
		prevX = 0;
		xSpeed = -1800;
		return -500;
	}
	if(prevX == -500){
		prevX = 0;
		xSpeed = 1800;
		return 500;
	}
	
    }
    
}

float pitchMapping(float gaze_normY)
{
    /*float gimblePitchAngle;
    float slope;
    slope = (300-0)/(1-0.5);
    gimblePitchAngle = 0 + slope * ( gaze_normY - 0.5 );
    //cout <<"Gaze Value:"<<gazeValue<<"Gimble Angle:"<<gimbleAngle<<"\n";
    return gimblePitchAngle;*/
    if(gaze_normY < 0.4)
    {
		
	if(prevY == 0){
        	prevY = -300;
		ySpeed = -1800;	
		return -300;
	}
	if(prevY == 300){
		prevY = -300;
		ySpeed = -1800;
		return -600;
	}
	if(prevY == -300){
		prevY = -300;
		ySpeed = 0;
		return 0;
	}
	
    }
    else if(gaze_normY > 0.6)
    {
	        
	if(prevY == 0){
        	prevY = 300;
		ySpeed = 1800;	
		return 300;
	}
	if(prevY == 300){
		prevY = 300;
		ySpeed = 0;
		return 0;
	}
	if(prevY == -300){
		prevY = 300;
		ySpeed = 1800;
		return 600;
	}
	
    }
    else
    {
	ySpeed = 0;	
	if(prevY == 0){
        	prevY = 0;
		ySpeed = 0;	
		return 0;
	}
	if(prevY == 300){
		prevY = 0;
		ySpeed = -1800;
		return -300;
	}
	if(prevY == -300){
		prevY = 0;
		ySpeed = 1800;
		return 300;
	}
	
    }
}

bool
gimbalCameraControl(Vehicle* vehicle)
{
  if(!vehicle->gimbal)
  {
    DERROR("Gimbal object does not exist.\n");
    return false;
  }

  int responseTimeout = 0;

  GimbalContainer              gimbal;
  RotationAngle                initialAngle;
  RotationAngle                currentAngle;
  DJI::OSDK::Gimbal::SpeedData gimbalSpeed;
  DJI::OSDK::Gimbal::SpeedData gimbalSpeed2;
  int                          pkgIndex;

  /*
   * Subscribe to gimbal data not supported in MAtrice 100
   */

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, __func__);
      return false;
    }

    // Telemetry: Subscribe to gimbal status and gimbal angle at freq 10 Hz
    pkgIndex                  = 0;
    int       freq            = 10;
    TopicName topicList10Hz[] = { TOPIC_GIMBAL_ANGLES, TOPIC_GIMBAL_STATUS };
    int       numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    bool      enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
      return pkgStatus;
    }
    subscribeStatus =
      vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
      ACK::getErrorCodeMessage(subscribeStatus, __func__);
      // Cleanup before return
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
      return false;
    }
  }

  sleep(1);

  std::cout
    << "Please note that the gimbal yaw angle you see in the telemetry is "
       "w.r.t absolute North"
       ", and the accuracy depends on your magnetometer calibration.\n\n";

  // Get Gimbal initial values
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    initialAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
    initialAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
    initialAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
  }
  else
  {
    initialAngle.roll  = vehicle->broadcast->getGimbal().roll;
    initialAngle.pitch = vehicle->broadcast->getGimbal().pitch;
    initialAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
  }

  std::cout << "Initial Gimbal rotation angle: [" << initialAngle.roll << ", "
            << initialAngle.pitch << ", " << initialAngle.yaw << "]\n\n";

  /*// Re-set Gimbal to initial values
  gimbal = GimbalContainer(0, 0, 0, 20, 1, initialAngle);
  doSetGimbalAngle(vehicle, &gimbal);

  std::cout << "Setting new Gimbal rotation angle to [0,20,180] using "
               "incremental control:\n";*/

  // Get current gimbal data to calc precision error in post processing
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
    currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
    currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
  }
  else
  {
    currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
    currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
    currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
  }
   
 // Set Gimble Speed
  /*gimbalSpeed.roll  = 0;
  gimbalSpeed.pitch = 1800;
  gimbalSpeed.yaw   = -1800;
  gimbalSpeed.gimbal_control_authority = 1;
  gimbalSpeed.disable_fov_zoom = 0;
  gimbalSpeed.ignore_user_stick = 1;
  gimbalSpeed.extend_control_range = 1;
  gimbalSpeed.ignore_aircraft_motion = 1;
  gimbalSpeed.yaw_return_neutral = 1;
  gimbalSpeed.reserved0 = 0;
  gimbalSpeed.reserved1 = 0;

  vehicle->gimbal->setSpeed(&gimbalSpeed);*/
 // Gimble Speed is set

  FILE *fp;
    char path[1035];
    char *newPath;
    float gx,gy,compansateGX,compansateGY; 
    float totalXTranslation=0, totalYTranslation=0;
    bool isFirstIteration = true;

    fp = popen("python3 /home/ubuntu16/readSecnario.py", "r");  // Open the command for reading.
 	printf(" testing-----\n" );
    if (fp == NULL) {
        printf("Failed to run command\n" );
        exit(1);
    }

  while(fgets(path, sizeof(path), fp) != NULL)
  {
	  gx = atof(strtok(path,","));
	  gy = atof(strtok(NULL,","));

	  // Set Gimble Speed
	  /*gimbalSpeed.roll  = 0;
	  gimbalSpeed.pitch = xSpeed;
	  gimbalSpeed.yaw   = ySpeed;
	  gimbalSpeed.gimbal_control_authority = 1;
	  gimbalSpeed.disable_fov_zoom = 0;
	  gimbalSpeed.ignore_user_stick = 0;
	  gimbalSpeed.extend_control_range = 0;
	  gimbalSpeed.ignore_aircraft_motion = 0;
	  gimbalSpeed.yaw_return_neutral = 0;
	  gimbalSpeed.reserved0 = 0;
	  gimbalSpeed.reserved1 = 0;

	  vehicle->gimbal->setSpeed(&gimbalSpeed);*/
	 // Gimble Speed is set
	  
	 /* if(isFirstIteration)
	  {
		compansateGX = abs(gx);
		compansateGY = abs(gy);
		if(gx < 0 )
	  		gx = gx + compansateGX;
	  	else
			gx = gx - compansateGX;
	
	  	if(gy < 0)
			gy = gy + compansateGY;
	  	else
			gy = gy - compansateGY;
		isFirstIteration = false;
          }
	  totalTranslationGX = gx - totalTranslationGX;
	  totalTranslationGX = gx - totalTranslationGX;*/
	 // std::cout <<"["<<gx<<","<<compansateGX<<"\t"<<gy<<","<<compansateGY<<"]\n";
	  // Get current gimbal data to calc precision error in post processing
  	if (!vehicle->isM100() && !vehicle->isLegacyM600())
	  {
	    currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
	    currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
	    currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
	  }
	  else
	  {
	    currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
	    currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
	    currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
	  }

	  gimbal = GimbalContainer(0, gy, gx, 20, 0, initialAngle, currentAngle);
	  doSetGimbalAngle(vehicle, &gimbal);

	  if (!vehicle->isM100() && !vehicle->isLegacyM600())
	  {
	    currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
	    currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
	    currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
	  }
	  else
	  {
	    currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
	    currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
	    currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
	  }
	  std::cout << "Setting new Gimbal rotation angle to [0,"<<gx<<","<<gy<<"] at ["<<xSpeed<<","<<ySpeed<<"]\n";
	  displayResult(&currentAngle);

	  /*gimbal = GimbalContainer(0, -gx, -gy, 20, 0, initialAngle, currentAngle);
	  doSetGimbalAngle(vehicle, &gimbal);

	  if (!vehicle->isM100() && !vehicle->isLegacyM600())
	  {
	    currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
	    currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
	    currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
	  }
	  else
	  {
	    currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
	    currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
	    currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
	  }
          std::cout << "Setting new Gimbal rotation angle to [0,0] using absolute "
		       "control:\n";
	  displayResult(&currentAngle);*/

	  
  }
  /*gimbalSpeed2.roll  = 0;
  gimbalSpeed2.pitch = -1800;
  gimbalSpeed2.yaw   = 1800;
  gimbalSpeed2.gimbal_control_authority = 1;
  gimbalSpeed2.disable_fov_zoom = 0;
  gimbalSpeed2.ignore_user_stick = 1;
  gimbalSpeed2.extend_control_range = 1;
  gimbalSpeed2.ignore_aircraft_motion = 1;
  gimbalSpeed2.yaw_return_neutral = 1;
  /*gimbalSpeed.reserved0 = 0;
  gimbalSpeed.reserved1 = 0;
  vehicle->gimbal->setSpeed(&gimbalSpeed2);*/

  
  /*gimbal = GimbalContainer(0, -300, 800, 20, 1, initialAngle);
  doSetGimbalAngle(vehicle, &gimbal);

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
    currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
    currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
  }
  else
  {
    currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
    currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
    currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
  }

  displayResult(&currentAngle);*/


  // Reset the position
  /*std::cout << "Resetting position...\n";
  gimbal = GimbalContainer(0, 0, 0, 20, 1, initialAngle);
  doSetGimbalAngle(vehicle, &gimbal);

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    currentAngle.roll  = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().y;
    currentAngle.pitch = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().x;
    currentAngle.yaw   = vehicle->subscribe->getValue<TOPIC_GIMBAL_ANGLES>().z;
  }
  else
  {
    currentAngle.roll  = vehicle->broadcast->getGimbal().roll;
    currentAngle.pitch = vehicle->broadcast->getGimbal().pitch;
    currentAngle.yaw   = vehicle->broadcast->getGimbal().yaw;
  }

  displayResult(&currentAngle);*/


  // Cleanup and exit gimbal sample
  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    ACK::ErrorCode ack =
      vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    if (ACK::getError(ack))
    {
      std::cout
        << "Error unsubscribing; please restart the drone/FC to get back "
           "to a clean state.\n";
    }
  }

  return true;
}

void
doSetGimbalAngle(Vehicle* vehicle, GimbalContainer* gimbal)
{
  DJI::OSDK::Gimbal::AngleData gimbalAngle;
  gimbalAngle.roll     = gimbal->roll;
  gimbalAngle.pitch    = gimbal->pitch;
  gimbalAngle.yaw      = gimbal->yaw;
  gimbalAngle.duration = gimbal->duration;
  gimbalAngle.mode |= 0;
  gimbalAngle.mode |= gimbal->isAbsolute;
  gimbalAngle.mode |= gimbal->yaw_cmd_ignore << 1;
  gimbalAngle.mode |= gimbal->roll_cmd_ignore << 2;
  gimbalAngle.mode |= gimbal->pitch_cmd_ignore << 3;

  vehicle->gimbal->setAngle(&gimbalAngle);
  // Give time for gimbal to sync
  sleep(4);
}

void
displayResult(RotationAngle* currentAngle)
{
  std::cout << "New Gimbal rotation angle is [";
  std::cout << currentAngle->roll << " ";
  std::cout << currentAngle->pitch << " ";
  std::cout << currentAngle->yaw;
  std::cout << "]\n\n";
}
