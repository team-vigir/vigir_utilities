#ifndef FILE_UTILS_H
#define FILE_UTILS_H


#include <string>
#include <ros/ros.h>

class FileUtils
{
   public:
      // Folder name without trailing "/"
      // suffix like so: ".png"
      static bool getTimeBasedUniqueFilename(const std::string& folder_name,
                                             const std::string& base_name,
                                             const std::string& suffix,
                                             std::string& result,
                                             ros::WallTime time = ros::WallTime::now())
      {
        if (folder_name == ""){
          return false;
        }

        result = folder_name + "/" + base_name + "_" + boost::lexical_cast<std::string>(time.toSec()) + suffix;

        return true;
      }

};

#endif

