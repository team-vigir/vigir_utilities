#ifndef TIMING_H
#define TIMING_H

#include <ros/ros.h>
#include <string>
#include "vigir_utilities/statistics.h"

//#define TIMING_PRINT_INTERMEDIATE 15000

/// A Simple class for timing purposes.
/**
 * Common Usage:
 * Timing t("Nasty Function");
 * nastyFunction();
 * t.printInfo();
 *char*
 * Will printDebug: "Nasty Function took 123ms."
 *
 * The timing can be (re-)started and ended by calling the respective functions.
 * diff() and printInfo() will end the timing implicitly.
 */
class Timing
{
   public:
      /// Create a timing object, if useUserTimes is true, times() will be used instead of getCurrentTime()
      Timing(const std::string& name = "Timing", bool useUserTimes = true, bool autostart = true);
      ~Timing();

      /// Resets start time to current time.
      void start();
      /// Stops the timing until start is called again.
      void end();

      /// Secs since construction (or start()) - does not end() the timing.
      ros::Duration peekDiff();

      /// Secs since construction (or start())
      /**
       * This function also calls end().
       */
      ros::Duration diff();

      /// Does print time since construction.
      /**
       * This function also calls end().
       * \param [in] alwaysPrint if true, overrides global setting and prints anyways
       */
      void printInfo(bool alwaysPrint = false);
      /// Same as printInfo, but uses ROS_DEBUG.
      void printDebug(bool alwaysPrint = false);
      void printStats(bool alwaysPrint = false);
      void printSummary(bool alwaysPrint = false); // stats formated for easy viewing
      void printStatsAndReset(bool alwaysPrint = false);

      /// Set, if timing info will be printed out for all Timing instances.
      static void setGlobalTiming(bool enabled);

      inline void getStatistics(double& mean, double& std_dev, double& min_val, double& max_val, uint32_t& num_measurements, const bool& reset_flag = false)
      {
          _stats->getStatistics(mean, std_dev,min_val,max_val,num_measurements,reset_flag);
      }

   private:
      bool _useUserTimes;

      ros::Time     _startTime;
      ros::Time     _endTime;
      std::string   _name;
      bool          _ended;

      Statistics<double> * _stats;

      static bool _timingEnabled;
};

class TimeThis
{
public:
    TimeThis(Timing& timing)
        : timing_(timing)
    {
        timing_.start();
    }
    ~TimeThis()
    {
        timing_.diff(); // end timer and calc elapsed
    }

private:
    Timing& timing_;

};

// Define a macro to invoke the timing operation
//   if we define DO_TIMING as empty, then skips this
#ifndef DO_TIMING
#define  DO_TIMING(X)  TimeThis do_timing( (X) );
#endif

#endif

