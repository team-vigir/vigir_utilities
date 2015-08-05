/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/* Author: David Conner (TORC Robotics) */
#include "vigir_utilities/timing.h"
#include <time.h>

Timing::Timing(const std::string& name, bool useUserTimes, bool autostart) : _useUserTimes(useUserTimes)
{
  _name = name;
  _stats = new Statistics<double>(name.c_str());
  _ended = true;
  if (autostart) start();
}

Timing::~Timing()
{
   //printf(" ~Timing : %s\n", _name.c_str());
   if (!_ended) printInfo(true); // print latest loop on exit
   printStats(true);
   delete _stats;
}

bool Timing::_timingEnabled = true;

void Timing::setGlobalTiming(bool enabled)
{
   _timingEnabled = enabled;
}

void Timing::start()
{
   if(_useUserTimes) {
       struct timespec ts;
       clock_gettime(CLOCK_REALTIME, &ts);
       _startTime = ros::Time(ts.tv_sec,ts.tv_nsec);
   } else {
      ros::WallTime wt = ros::WallTime::now();
      _startTime = ros::Time(wt.sec,wt.nsec);
   }
   //ROS_INFO("   Start time %s = %ld (%p)",  _name.c_str(), _startTime.toNSec(), this);
   _ended = false;
}

void Timing::end()
{
   if(!_ended) {
      if(_useUserTimes) {
          struct timespec ts;
          clock_gettime(CLOCK_REALTIME, &ts);
          _endTime = ros::Time(ts.tv_sec,ts.tv_nsec);
      } else {
          ros::WallTime wt = ros::WallTime::now();
          _endTime = ros::Time(wt.sec,wt.nsec);
       }
   }
   //ROS_INFO("   End   time %s = %ld (%p)", _name.c_str(), _endTime.toNSec(),this);
   _ended = true;
}

ros::Duration Timing::peekDiff()
{
    if(!_ended) {
        ros::Time endTime(0,0);
        if(_useUserTimes) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            endTime = ros::Time(ts.tv_sec,ts.tv_nsec);
        } else {
            ros::WallTime wt = ros::WallTime::now();
            endTime = ros::Time(wt.sec,wt.nsec);
        }
        return (endTime - _startTime);
    } else {
        return (_endTime - _startTime);
    }
}

ros::Duration Timing::diff()
{
    bool add = !_ended;

    end(); // set end and flag as required

    ros::Duration elapsed = _endTime - _startTime;

    if (add)
    { // only add measurement if we had an active timer running
       int num = _stats->addMeasurement(1000.0*elapsed.toSec()); // convert seconds to milliseconds

#ifdef TIMING_PRINT_INTERMEDIATE
       if (0 == num%TIMING_PRINT_INTERMEDIATE)
       {
           printSummary(true); // dump output periodically since Ctrl-C just kills processes without orderly shutdown
           _stats->reset();
       }
#endif
    }

   return elapsed;
}

void Timing::printInfo(bool alwaysPrint)
{
   if(!alwaysPrint) {
      if(!_timingEnabled)
         return;
   }

#ifdef _WIN32
#ifndef snprintf
#define snprintf _snprintf
#endif
#endif

   char buf[4096];
   snprintf(buf, 4095, "%s took: %.6f ms.\n", _name.c_str(), diff().toSec() * 1000.0);
   //printf("%s", buf);
   buf[strlen(buf) - 1] = '\0';  // remove \n
   ROS_INFO("%s", buf);
}

void Timing::printSummary(bool alwaysPrint)
{
   if(!alwaysPrint) {
      if(!_timingEnabled)
         return;
   }

#ifdef _WIN32
#ifndef snprintf
#define snprintf _snprintf
#endif
#endif

   char buf[4096];
   double mean, min_val, max_val, std_dev;
   uint32_t num_meas;
   _stats->getStatistics(mean,std_dev,min_val,max_val,num_meas); // single access lock
   snprintf(buf, 4095, "%45s: Mean %.6f ms, Std: %.6f  min=%.6f  max=%.6f  (Num: %d)\n",
                    _name.c_str(),  mean,      std_dev,    min_val, max_val, num_meas);

   buf[strlen(buf) - 1] = '\0';  // remove \n
   ROS_INFO("\n     %s\n--------------------------------------------", buf);
}

void Timing::printDebug(bool alwaysPrint)
{
   if(!alwaysPrint) {
      if(!_timingEnabled)
         return;
   }

#ifdef _WIN32
#ifndef snprintf
#define snprintf _snprintf
#endif
#endif

   char buf[4096];
   snprintf(buf, 4095, "%s took: %.6fms.\n", _name.c_str(), diff().toSec() * 1000.0);
   //printf("%s", buf);
   buf[strlen(buf) - 1] = '\0';  // remove \n
   ROS_DEBUG("%s", buf);
}

void Timing::printStats(bool alwaysPrint)
{
   if(!alwaysPrint) {
      if(!_timingEnabled)
         return;
   }
   _stats->print();
}

void Timing::printStatsAndReset(bool alwaysPrint)
{
   if(!alwaysPrint) {
      if(!_timingEnabled)
         return;
   }
   _stats->printAndReset();
}



