// Generated by gencpp from file sdv_msgs/Trajectory.msg
// DO NOT EDIT!


#ifndef SDV_MSGS_MESSAGE_TRAJECTORY_H
#define SDV_MSGS_MESSAGE_TRAJECTORY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <sdv_msgs/TrajectoryPoint.h>

namespace sdv_msgs
{
template <class ContainerAllocator>
struct Trajectory_
{
  typedef Trajectory_<ContainerAllocator> Type;

  Trajectory_()
    : header()
    , points()  {
    }
  Trajectory_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , points(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> >> _points_type;
  _points_type points;





  typedef boost::shared_ptr< ::sdv_msgs::Trajectory_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sdv_msgs::Trajectory_<ContainerAllocator> const> ConstPtr;

}; // struct Trajectory_

typedef ::sdv_msgs::Trajectory_<std::allocator<void> > Trajectory;

typedef boost::shared_ptr< ::sdv_msgs::Trajectory > TrajectoryPtr;
typedef boost::shared_ptr< ::sdv_msgs::Trajectory const> TrajectoryConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sdv_msgs::Trajectory_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sdv_msgs::Trajectory_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::sdv_msgs::Trajectory_<ContainerAllocator1> & lhs, const ::sdv_msgs::Trajectory_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.points == rhs.points;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::sdv_msgs::Trajectory_<ContainerAllocator1> & lhs, const ::sdv_msgs::Trajectory_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace sdv_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::sdv_msgs::Trajectory_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sdv_msgs::Trajectory_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sdv_msgs::Trajectory_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sdv_msgs::Trajectory_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sdv_msgs::Trajectory_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sdv_msgs::Trajectory_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sdv_msgs::Trajectory_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4346f6f26976d7cfbbe01411176e4e52";
  }

  static const char* value(const ::sdv_msgs::Trajectory_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4346f6f26976d7cfULL;
  static const uint64_t static_value2 = 0xbbe01411176e4e52ULL;
};

template<class ContainerAllocator>
struct DataType< ::sdv_msgs::Trajectory_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sdv_msgs/Trajectory";
  }

  static const char* value(const ::sdv_msgs::Trajectory_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sdv_msgs::Trajectory_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Trajectory.msg\n"
"# similar msg to the msg in Autoware.auto\n"
"\n"
"std_msgs/Header header\n"
"TrajectoryPoint[] points\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: sdv_msgs/TrajectoryPoint\n"
"# TrajectoryPoint.msg\n"
"\n"
"#iteration of the trajectory points (time can be received by: trajectory_point * sampling_time)\n"
"uint32 trajectory_point\n"
"float32 x\n"
"float32 y\n"
"float32 heading\n"
"float32 x_dot\n"
"float32 y_dot\n"
"float32 velocity_mps\n"
"float32 acceleration_mps2\n"
"float32 heading_rate_radps\n"
"float32 heading_acc_radps2\n"
;
  }

  static const char* value(const ::sdv_msgs::Trajectory_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sdv_msgs::Trajectory_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.points);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Trajectory_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sdv_msgs::Trajectory_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sdv_msgs::Trajectory_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> >::stream(s, indent + "    ", v.points[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SDV_MSGS_MESSAGE_TRAJECTORY_H