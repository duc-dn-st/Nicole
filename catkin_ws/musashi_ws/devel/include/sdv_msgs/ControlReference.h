// Generated by gencpp from file sdv_msgs/ControlReference.msg
// DO NOT EDIT!


#ifndef SDV_MSGS_MESSAGE_CONTROLREFERENCE_H
#define SDV_MSGS_MESSAGE_CONTROLREFERENCE_H


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
struct ControlReference_
{
  typedef ControlReference_<ContainerAllocator> Type;

  ControlReference_()
    : velocity_ref_front_left(0.0)
    , velocity_ref_front_right(0.0)
    , velocity_ref_back_left(0.0)
    , velocity_ref_back_right(0.0)
    , steering_ref_front_left(0.0)
    , steering_ref_front_right(0.0)
    , steering_ref_back_left(0.0)
    , steering_ref_back_right(0.0)
    , stop_motors(false)  {
    }
  ControlReference_(const ContainerAllocator& _alloc)
    : velocity_ref_front_left(0.0)
    , velocity_ref_front_right(0.0)
    , velocity_ref_back_left(0.0)
    , velocity_ref_back_right(0.0)
    , steering_ref_front_left(0.0)
    , steering_ref_front_right(0.0)
    , steering_ref_back_left(0.0)
    , steering_ref_back_right(0.0)
    , stop_motors(false)  {
  (void)_alloc;
    }



   typedef float _velocity_ref_front_left_type;
  _velocity_ref_front_left_type velocity_ref_front_left;

   typedef float _velocity_ref_front_right_type;
  _velocity_ref_front_right_type velocity_ref_front_right;

   typedef float _velocity_ref_back_left_type;
  _velocity_ref_back_left_type velocity_ref_back_left;

   typedef float _velocity_ref_back_right_type;
  _velocity_ref_back_right_type velocity_ref_back_right;

   typedef float _steering_ref_front_left_type;
  _steering_ref_front_left_type steering_ref_front_left;

   typedef float _steering_ref_front_right_type;
  _steering_ref_front_right_type steering_ref_front_right;

   typedef float _steering_ref_back_left_type;
  _steering_ref_back_left_type steering_ref_back_left;

   typedef float _steering_ref_back_right_type;
  _steering_ref_back_right_type steering_ref_back_right;

   typedef uint8_t _stop_motors_type;
  _stop_motors_type stop_motors;





  typedef boost::shared_ptr< ::sdv_msgs::ControlReference_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sdv_msgs::ControlReference_<ContainerAllocator> const> ConstPtr;

}; // struct ControlReference_

typedef ::sdv_msgs::ControlReference_<std::allocator<void> > ControlReference;

typedef boost::shared_ptr< ::sdv_msgs::ControlReference > ControlReferencePtr;
typedef boost::shared_ptr< ::sdv_msgs::ControlReference const> ControlReferenceConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sdv_msgs::ControlReference_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sdv_msgs::ControlReference_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::sdv_msgs::ControlReference_<ContainerAllocator1> & lhs, const ::sdv_msgs::ControlReference_<ContainerAllocator2> & rhs)
{
  return lhs.velocity_ref_front_left == rhs.velocity_ref_front_left &&
    lhs.velocity_ref_front_right == rhs.velocity_ref_front_right &&
    lhs.velocity_ref_back_left == rhs.velocity_ref_back_left &&
    lhs.velocity_ref_back_right == rhs.velocity_ref_back_right &&
    lhs.steering_ref_front_left == rhs.steering_ref_front_left &&
    lhs.steering_ref_front_right == rhs.steering_ref_front_right &&
    lhs.steering_ref_back_left == rhs.steering_ref_back_left &&
    lhs.steering_ref_back_right == rhs.steering_ref_back_right &&
    lhs.stop_motors == rhs.stop_motors;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::sdv_msgs::ControlReference_<ContainerAllocator1> & lhs, const ::sdv_msgs::ControlReference_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace sdv_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::sdv_msgs::ControlReference_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sdv_msgs::ControlReference_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sdv_msgs::ControlReference_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sdv_msgs::ControlReference_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sdv_msgs::ControlReference_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sdv_msgs::ControlReference_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sdv_msgs::ControlReference_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a0c35083c971411263b902e2dfd735f5";
  }

  static const char* value(const ::sdv_msgs::ControlReference_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa0c35083c9714112ULL;
  static const uint64_t static_value2 = 0x63b902e2dfd735f5ULL;
};

template<class ContainerAllocator>
struct DataType< ::sdv_msgs::ControlReference_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sdv_msgs/ControlReference";
  }

  static const char* value(const ::sdv_msgs::ControlReference_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sdv_msgs::ControlReference_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ControlReference.msg\n"
"# message which sends the reference for the low level controller\n"
"\n"
"# velocity in m/s\n"
"float32 velocity_ref_front_left\n"
"float32 velocity_ref_front_right\n"
"float32 velocity_ref_back_left\n"
"float32 velocity_ref_back_right\n"
"# steering angle in rad from -pi to pi\n"
"float32 steering_ref_front_left\n"
"float32 steering_ref_front_right\n"
"float32 steering_ref_back_left\n"
"float32 steering_ref_back_right\n"
"# boolean if emergency stop necessary\n"
"bool stop_motors\n"
;
  }

  static const char* value(const ::sdv_msgs::ControlReference_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sdv_msgs::ControlReference_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.velocity_ref_front_left);
      stream.next(m.velocity_ref_front_right);
      stream.next(m.velocity_ref_back_left);
      stream.next(m.velocity_ref_back_right);
      stream.next(m.steering_ref_front_left);
      stream.next(m.steering_ref_front_right);
      stream.next(m.steering_ref_back_left);
      stream.next(m.steering_ref_back_right);
      stream.next(m.stop_motors);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ControlReference_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sdv_msgs::ControlReference_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sdv_msgs::ControlReference_<ContainerAllocator>& v)
  {
    s << indent << "velocity_ref_front_left: ";
    Printer<float>::stream(s, indent + "  ", v.velocity_ref_front_left);
    s << indent << "velocity_ref_front_right: ";
    Printer<float>::stream(s, indent + "  ", v.velocity_ref_front_right);
    s << indent << "velocity_ref_back_left: ";
    Printer<float>::stream(s, indent + "  ", v.velocity_ref_back_left);
    s << indent << "velocity_ref_back_right: ";
    Printer<float>::stream(s, indent + "  ", v.velocity_ref_back_right);
    s << indent << "steering_ref_front_left: ";
    Printer<float>::stream(s, indent + "  ", v.steering_ref_front_left);
    s << indent << "steering_ref_front_right: ";
    Printer<float>::stream(s, indent + "  ", v.steering_ref_front_right);
    s << indent << "steering_ref_back_left: ";
    Printer<float>::stream(s, indent + "  ", v.steering_ref_back_left);
    s << indent << "steering_ref_back_right: ";
    Printer<float>::stream(s, indent + "  ", v.steering_ref_back_right);
    s << indent << "stop_motors: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.stop_motors);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SDV_MSGS_MESSAGE_CONTROLREFERENCE_H
