// Generated by gencpp from file sdv_msgs/TrajectoryFlagsRequest.msg
// DO NOT EDIT!


#ifndef SDV_MSGS_MESSAGE_TRAJECTORYFLAGSREQUEST_H
#define SDV_MSGS_MESSAGE_TRAJECTORYFLAGSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace sdv_msgs
{
template <class ContainerAllocator>
struct TrajectoryFlagsRequest_
{
  typedef TrajectoryFlagsRequest_<ContainerAllocator> Type;

  TrajectoryFlagsRequest_()
    {
    }
  TrajectoryFlagsRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct TrajectoryFlagsRequest_

typedef ::sdv_msgs::TrajectoryFlagsRequest_<std::allocator<void> > TrajectoryFlagsRequest;

typedef boost::shared_ptr< ::sdv_msgs::TrajectoryFlagsRequest > TrajectoryFlagsRequestPtr;
typedef boost::shared_ptr< ::sdv_msgs::TrajectoryFlagsRequest const> TrajectoryFlagsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace sdv_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sdv_msgs/TrajectoryFlagsRequest";
  }

  static const char* value(const ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrajectoryFlagsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::sdv_msgs::TrajectoryFlagsRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // SDV_MSGS_MESSAGE_TRAJECTORYFLAGSREQUEST_H
