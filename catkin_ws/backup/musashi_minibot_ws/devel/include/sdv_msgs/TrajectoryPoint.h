// Generated by gencpp from file sdv_msgs/TrajectoryPoint.msg
// DO NOT EDIT!


#ifndef SDV_MSGS_MESSAGE_TRAJECTORYPOINT_H
#define SDV_MSGS_MESSAGE_TRAJECTORYPOINT_H


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
struct TrajectoryPoint_
{
  typedef TrajectoryPoint_<ContainerAllocator> Type;

  TrajectoryPoint_()
    : trajectory_point(0)
    , x(0.0)
    , y(0.0)
    , heading(0.0)
    , x_dot(0.0)
    , y_dot(0.0)
    , velocity_mps(0.0)
    , acceleration_mps2(0.0)
    , heading_rate_radps(0.0)
    , heading_acc_radps2(0.0)  {
    }
  TrajectoryPoint_(const ContainerAllocator& _alloc)
    : trajectory_point(0)
    , x(0.0)
    , y(0.0)
    , heading(0.0)
    , x_dot(0.0)
    , y_dot(0.0)
    , velocity_mps(0.0)
    , acceleration_mps2(0.0)
    , heading_rate_radps(0.0)
    , heading_acc_radps2(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _trajectory_point_type;
  _trajectory_point_type trajectory_point;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _heading_type;
  _heading_type heading;

   typedef float _x_dot_type;
  _x_dot_type x_dot;

   typedef float _y_dot_type;
  _y_dot_type y_dot;

   typedef float _velocity_mps_type;
  _velocity_mps_type velocity_mps;

   typedef float _acceleration_mps2_type;
  _acceleration_mps2_type acceleration_mps2;

   typedef float _heading_rate_radps_type;
  _heading_rate_radps_type heading_rate_radps;

   typedef float _heading_acc_radps2_type;
  _heading_acc_radps2_type heading_acc_radps2;





  typedef boost::shared_ptr< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> const> ConstPtr;

}; // struct TrajectoryPoint_

typedef ::sdv_msgs::TrajectoryPoint_<std::allocator<void> > TrajectoryPoint;

typedef boost::shared_ptr< ::sdv_msgs::TrajectoryPoint > TrajectoryPointPtr;
typedef boost::shared_ptr< ::sdv_msgs::TrajectoryPoint const> TrajectoryPointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::sdv_msgs::TrajectoryPoint_<ContainerAllocator1> & lhs, const ::sdv_msgs::TrajectoryPoint_<ContainerAllocator2> & rhs)
{
  return lhs.trajectory_point == rhs.trajectory_point &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.heading == rhs.heading &&
    lhs.x_dot == rhs.x_dot &&
    lhs.y_dot == rhs.y_dot &&
    lhs.velocity_mps == rhs.velocity_mps &&
    lhs.acceleration_mps2 == rhs.acceleration_mps2 &&
    lhs.heading_rate_radps == rhs.heading_rate_radps &&
    lhs.heading_acc_radps2 == rhs.heading_acc_radps2;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::sdv_msgs::TrajectoryPoint_<ContainerAllocator1> & lhs, const ::sdv_msgs::TrajectoryPoint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace sdv_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2f3ee95494658728b8847e8661619e42";
  }

  static const char* value(const ::sdv_msgs::TrajectoryPoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2f3ee95494658728ULL;
  static const uint64_t static_value2 = 0xb8847e8661619e42ULL;
};

template<class ContainerAllocator>
struct DataType< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sdv_msgs/TrajectoryPoint";
  }

  static const char* value(const ::sdv_msgs::TrajectoryPoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# TrajectoryPoint.msg\n"
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

  static const char* value(const ::sdv_msgs::TrajectoryPoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.trajectory_point);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.heading);
      stream.next(m.x_dot);
      stream.next(m.y_dot);
      stream.next(m.velocity_mps);
      stream.next(m.acceleration_mps2);
      stream.next(m.heading_rate_radps);
      stream.next(m.heading_acc_radps2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrajectoryPoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sdv_msgs::TrajectoryPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sdv_msgs::TrajectoryPoint_<ContainerAllocator>& v)
  {
    s << indent << "trajectory_point: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.trajectory_point);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "heading: ";
    Printer<float>::stream(s, indent + "  ", v.heading);
    s << indent << "x_dot: ";
    Printer<float>::stream(s, indent + "  ", v.x_dot);
    s << indent << "y_dot: ";
    Printer<float>::stream(s, indent + "  ", v.y_dot);
    s << indent << "velocity_mps: ";
    Printer<float>::stream(s, indent + "  ", v.velocity_mps);
    s << indent << "acceleration_mps2: ";
    Printer<float>::stream(s, indent + "  ", v.acceleration_mps2);
    s << indent << "heading_rate_radps: ";
    Printer<float>::stream(s, indent + "  ", v.heading_rate_radps);
    s << indent << "heading_acc_radps2: ";
    Printer<float>::stream(s, indent + "  ", v.heading_acc_radps2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SDV_MSGS_MESSAGE_TRAJECTORYPOINT_H
