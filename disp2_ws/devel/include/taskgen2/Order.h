// Generated by gencpp from file taskgen2/Order.msg
// DO NOT EDIT!


#ifndef TASKGEN2_MESSAGE_ORDER_H
#define TASKGEN2_MESSAGE_ORDER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace taskgen2
{
template <class ContainerAllocator>
struct Order_
{
  typedef Order_<ContainerAllocator> Type;

  Order_()
    : deadline(0.0)
    , x_position(0.0)
    , y_position(0.0)
    , z_orientation(0.0)  {
    }
  Order_(const ContainerAllocator& _alloc)
    : deadline(0.0)
    , x_position(0.0)
    , y_position(0.0)
    , z_orientation(0.0)  {
  (void)_alloc;
    }



   typedef double _deadline_type;
  _deadline_type deadline;

   typedef double _x_position_type;
  _x_position_type x_position;

   typedef double _y_position_type;
  _y_position_type y_position;

   typedef double _z_orientation_type;
  _z_orientation_type z_orientation;





  typedef boost::shared_ptr< ::taskgen2::Order_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::taskgen2::Order_<ContainerAllocator> const> ConstPtr;

}; // struct Order_

typedef ::taskgen2::Order_<std::allocator<void> > Order;

typedef boost::shared_ptr< ::taskgen2::Order > OrderPtr;
typedef boost::shared_ptr< ::taskgen2::Order const> OrderConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::taskgen2::Order_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::taskgen2::Order_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace taskgen2

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'taskgen2': ['/home/martin/disp2_ws/src/taskgen2/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::taskgen2::Order_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::taskgen2::Order_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::taskgen2::Order_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::taskgen2::Order_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::taskgen2::Order_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::taskgen2::Order_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::taskgen2::Order_<ContainerAllocator> >
{
  static const char* value()
  {
    return "260c63c5cf6dcaef40226fcd5deee22b";
  }

  static const char* value(const ::taskgen2::Order_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x260c63c5cf6dcaefULL;
  static const uint64_t static_value2 = 0x40226fcd5deee22bULL;
};

template<class ContainerAllocator>
struct DataType< ::taskgen2::Order_<ContainerAllocator> >
{
  static const char* value()
  {
    return "taskgen2/Order";
  }

  static const char* value(const ::taskgen2::Order_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::taskgen2::Order_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 deadline\n\
float64 x_position\n\
float64 y_position\n\
float64 z_orientation\n\
";
  }

  static const char* value(const ::taskgen2::Order_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::taskgen2::Order_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.deadline);
      stream.next(m.x_position);
      stream.next(m.y_position);
      stream.next(m.z_orientation);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Order_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::taskgen2::Order_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::taskgen2::Order_<ContainerAllocator>& v)
  {
    s << indent << "deadline: ";
    Printer<double>::stream(s, indent + "  ", v.deadline);
    s << indent << "x_position: ";
    Printer<double>::stream(s, indent + "  ", v.x_position);
    s << indent << "y_position: ";
    Printer<double>::stream(s, indent + "  ", v.y_position);
    s << indent << "z_orientation: ";
    Printer<double>::stream(s, indent + "  ", v.z_orientation);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TASKGEN2_MESSAGE_ORDER_H
