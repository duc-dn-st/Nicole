// Generated by gencpp from file sdv_msgs/TrajectoryFlags.msg
// DO NOT EDIT!


#ifndef SDV_MSGS_MESSAGE_TRAJECTORYFLAGS_H
#define SDV_MSGS_MESSAGE_TRAJECTORYFLAGS_H

#include <ros/service_traits.h>


#include <sdv_msgs/TrajectoryFlagsRequest.h>
#include <sdv_msgs/TrajectoryFlagsResponse.h>


namespace sdv_msgs
{

struct TrajectoryFlags
{

typedef TrajectoryFlagsRequest Request;
typedef TrajectoryFlagsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct TrajectoryFlags
} // namespace sdv_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::sdv_msgs::TrajectoryFlags > {
  static const char* value()
  {
    return "bd4fde1f11e0a0f6e47a699d2934c6b7";
  }

  static const char* value(const ::sdv_msgs::TrajectoryFlags&) { return value(); }
};

template<>
struct DataType< ::sdv_msgs::TrajectoryFlags > {
  static const char* value()
  {
    return "sdv_msgs/TrajectoryFlags";
  }

  static const char* value(const ::sdv_msgs::TrajectoryFlags&) { return value(); }
};


// service_traits::MD5Sum< ::sdv_msgs::TrajectoryFlagsRequest> should match
// service_traits::MD5Sum< ::sdv_msgs::TrajectoryFlags >
template<>
struct MD5Sum< ::sdv_msgs::TrajectoryFlagsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::sdv_msgs::TrajectoryFlags >::value();
  }
  static const char* value(const ::sdv_msgs::TrajectoryFlagsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::sdv_msgs::TrajectoryFlagsRequest> should match
// service_traits::DataType< ::sdv_msgs::TrajectoryFlags >
template<>
struct DataType< ::sdv_msgs::TrajectoryFlagsRequest>
{
  static const char* value()
  {
    return DataType< ::sdv_msgs::TrajectoryFlags >::value();
  }
  static const char* value(const ::sdv_msgs::TrajectoryFlagsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::sdv_msgs::TrajectoryFlagsResponse> should match
// service_traits::MD5Sum< ::sdv_msgs::TrajectoryFlags >
template<>
struct MD5Sum< ::sdv_msgs::TrajectoryFlagsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::sdv_msgs::TrajectoryFlags >::value();
  }
  static const char* value(const ::sdv_msgs::TrajectoryFlagsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::sdv_msgs::TrajectoryFlagsResponse> should match
// service_traits::DataType< ::sdv_msgs::TrajectoryFlags >
template<>
struct DataType< ::sdv_msgs::TrajectoryFlagsResponse>
{
  static const char* value()
  {
    return DataType< ::sdv_msgs::TrajectoryFlags >::value();
  }
  static const char* value(const ::sdv_msgs::TrajectoryFlagsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SDV_MSGS_MESSAGE_TRAJECTORYFLAGS_H
