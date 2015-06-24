#ifndef STATISTICS_H
#define STATISTICS_H

#include <math.h>
#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

/// This class calculates mean and standard deviation for incremental values.
template <class T>
class Statistics
{
   public:
      Statistics(const std::string& name = "");
      //~Statistics(){ printf("~Statistics: %s\n",_name.c_str());}

      void reset();
      int addMeasurement(T val);

      T getMean();
      T getStandardDeviation();
      int getNumMeasurements();

      T getMin() { return _min; }
      T getMax() { return _max; }

      void getStatistics(T& mean, T& std_dev, T& min_val, T& max_val, uint32_t& num_measurements, const bool& reset_flag = false);

      void print(const double factor=1.0);
      void printAndReset(const double factor=1.0);

   protected:
      T _sumMeasurements;
      T _sumSquaredMeasurements;
      T _min;
      T _max;

      uint32_t _numMeasurements;

      std::string       _name;
      boost::mutex      _stats_mutex;
};

template <class T>
Statistics<T>::Statistics(const std::string& name) : _name(name)
{
   //printf(" Statistics name %s (%p)\n",_name.c_str(), this);
   reset();
}

template <class T>
void Statistics<T>::reset()
{
    boost::lock_guard<boost::mutex> guard(_stats_mutex);
    //ROS_INFO("       Reset (%d) measurements for Statistics %s (%p) ", _numMeasurements, _name.c_str(),this);
   _sumMeasurements = static_cast<T>(0);
   _sumSquaredMeasurements = static_cast<T>(0);
   _numMeasurements = 0;
   _min =  HUGE_VAL;     // this probably only works for doubles
   _max = -HUGE_VAL;
}

template <class T>
int Statistics<T>::addMeasurement(T val)
{
    boost::lock_guard<boost::mutex> guard(_stats_mutex);
   _sumMeasurements += val;
   _sumSquaredMeasurements += (val * val);
   ++_numMeasurements;
   if(val > _max)
      _max = val;
   if(val < _min)
      _min = val;
   //ROS_INFO("       Add measurement(%d) = %f to %s (%p) - sum=%f _min=%f _max=%f", _numMeasurements, val, _name.c_str(),this, _sumMeasurements, _min, _max);
   return (int)_numMeasurements;
}

template <class T>
T Statistics<T>::getMean()
{
   if(_numMeasurements == 0)
      return static_cast<T>(0);

   boost::lock_guard<boost::mutex> guard(_stats_mutex);
   return _sumMeasurements/_numMeasurements;
}

template <class T>
T Statistics<T>::getStandardDeviation()
{
   if(_numMeasurements < 2)
      return static_cast<T>(0);

   boost::lock_guard<boost::mutex> guard(_stats_mutex);
   return sqrt( (_numMeasurements * _sumSquaredMeasurements - _sumMeasurements * _sumMeasurements) / (_numMeasurements * (_numMeasurements - 1)) );
}

template <class T>
int Statistics<T>::getNumMeasurements()
{
   boost::lock_guard<boost::mutex> guard(_stats_mutex);
   return (int)_numMeasurements;
}

template <class T>
void Statistics<T>::getStatistics(T& mean_val, T& std_dev, T& min_val, T& max_val, uint32_t& num_measurements, const bool& reset_flag)
{
    boost::lock_guard<boost::mutex> guard(_stats_mutex);
    num_measurements = _numMeasurements;
    if(_numMeasurements < 1)
    {
       mean_val = static_cast<T>(0);
       min_val  = static_cast<T>(0);
       max_val  = static_cast<T>(0);
       std_dev  = static_cast<T>(0);
       return;
    }
    mean_val = _sumMeasurements/_numMeasurements;
    min_val  = _min;
    max_val  = _max;
    std_dev  = static_cast<T>(0);
    if (_numMeasurements > 1)
    {
        std_dev = sqrt( (_numMeasurements * _sumSquaredMeasurements - _sumMeasurements * _sumMeasurements) / (_numMeasurements * (_numMeasurements - 1)) );
    }

    if (reset_flag)
    { // reset the statistics for next update
        _sumMeasurements        = static_cast<T>(0);
        _sumSquaredMeasurements = static_cast<T>(0);
        _numMeasurements = 0;
        _min =  HUGE_VAL;     // this probably only works for doubles
        _max = -HUGE_VAL;
    }
    return;
}

template <class T>
void Statistics<T>::print(const double factor)
{
   boost::lock_guard<boost::mutex> guard(_stats_mutex);
   T mean = static_cast<T>(0);
   T std_dev = static_cast<T>(0);
   if ( _numMeasurements)
   {  // calculate here to avoid thread deadlock
       mean    = _sumMeasurements/_numMeasurements;
       if (_numMeasurements > 1) std_dev = sqrt( (_numMeasurements * _sumSquaredMeasurements - _sumMeasurements * _sumMeasurements) / (_numMeasurements * (_numMeasurements - 1)) );
   }
   printf(" %s: Mean %.6f Std: %.6f  min=%.6f max=%.6f  (Num: %d)\n",
          _name.c_str(), mean*factor, std_dev*factor, _min, _max, _numMeasurements);
}

template <class T>
void Statistics<T>::printAndReset(const double factor)
{
   boost::lock_guard<boost::mutex> guard(_stats_mutex);
   T mean = static_cast<T>(0);
   T std_dev = static_cast<T>(0);
   if ( _numMeasurements)
   {  // calculate here to avoid thread deadlock
       mean    = _sumMeasurements/_numMeasurements;
       if (_numMeasurements > 1) std_dev = sqrt( (_numMeasurements * _sumSquaredMeasurements - _sumMeasurements * _sumMeasurements) / (_numMeasurements * (_numMeasurements - 1)) );
   }
   printf(" %38.38s : Mean %.6f Std: %.6f  min=%.6f max=%.6f  (Num: %d)\n",
          _name.c_str(), mean*factor, std_dev*factor, _min, _max, _numMeasurements);

   // Reset for the next update
   _sumMeasurements = static_cast<T>(0);
   _sumSquaredMeasurements = static_cast<T>(0);
   _numMeasurements = 0;
   _min =  HUGE_VAL;     // this probably only works for doubles
   _max = -HUGE_VAL;
}

#endif

