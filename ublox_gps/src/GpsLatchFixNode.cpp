#include "ros/ros.h"
#include "ros/subscriber.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "ublox_msgs/NavSTATUS.h"
#include "sensor_msgs/NavSatFix.h"
#define RTK_FIX 3

class LatchNode // Problem, stampedStatus does not have a header field and cannot by synced - hence this node!
{
public:
  typedef std::pair<double,sensor_msgs::NavSatFix> stampedFix;
  typedef std::pair<double,ublox_msgs::NavSTATUS> stampedStatus;

  void GpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    fixStream_.push_back(stampedFix(ros::Time::now().toSec(),*msg));
    //std::cout << "GPS: " << msg->header.stamp << std::endl;;
    SyncCheckFixPublish();
  }
  void NavStatusCallback(const ublox_msgs::NavSTATUSConstPtr& msg)
  {
    statusStream_.push_back(stampedStatus(ros::Time::now().toSec(),*msg));
    //std::cout << "status: " << (int)msg->gpsFix << std::endl;
    SyncCheckFixPublish();
  }
  void ReadFrequency()
  {
    if(fixStream_.size() > 5){
      double dt = 0;
      int N;
      for(N = 0; N < fixStream_.size() -1 ; N++){
        dt += fabs(fixStream_[N+1].first - fixStream_[N].first);
      }
      dt_limit_ = dt/(N*2.0);
      frequency_found_ = true;
      //std::cout << "found frequency: " << 2*dt_limit_<< std::endl;
    }
  }
  void SyncCheckFixPublish()
  {
    if(!frequency_found_)
    {
      ReadFrequency();
    }
    else if(!fixStream_.empty() && !statusStream_.empty())
    {
      // Check for consistent RTK-FIX
      if( ((int)statusStream_.back().second.gpsFix) == RTK_FIX){
        //std::cout <<"GOT FIX" << nrConsequtive_  << std::endl;
        ROS_INFO_THROTTLE(2, "\033[1;32m GpsFixLatchNode - Valid RTK-FIX=3 \033[0m");
        nrConsequtive_++;
      }
      else{
        nrConsequtive_ = 0;
        ROS_INFO_THROTTLE(2, "\033[1;33m GpsFixLatchNode - InValid RTK-FIX \033[0m");
      }
      if(nrConsequtive_ > minNrConsecutive && fabs( fixStream_.back().first - statusStream_.back().first) < dt_limit_)
      {
        pubFixLatched_.publish(fixStream_.back().second);
        fixStream_.clear();
        statusStream_.clear();
      }
    }
  }

  LatchNode(ros::NodeHandle& nh) : nh_(nh)
  {
    subFix_ = nh_.subscribe("/ublox/fix", 1000, &LatchNode::GpsFixCallback, this);
    subStatus_ = nh_.subscribe("/ublox/navstatus", 1000, &LatchNode::NavStatusCallback, this);
    pubFixLatched_ = nh_.advertise<sensor_msgs::NavSatFix>("/fix/latched", 1000);
  }
  ros::Subscriber subFix_;
  ros::Subscriber subStatus_;
  ros::Publisher pubFixLatched_;
  ros::NodeHandle nh_;
  std::vector<stampedFix> fixStream_;
  std::vector<stampedStatus> statusStream_;
  bool frequency_found_ = false;
  double dt_limit_ = 0;
  int64_t nrConsequtive_ = 0;
  const int64_t minNrConsecutive = 10; // 1 sec


};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "GpsFixLatchNode");
  ros::NodeHandle nh;
  LatchNode latch(nh);
  ros::spin();
  return 0;
}
