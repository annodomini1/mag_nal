// Generated by gencpp from file taskgen/Task.msg
// DO NOT EDIT!


#ifndef TASKGEN_MESSAGE_TASK_H
#define TASKGEN_MESSAGE_TASK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace taskgen
{
template <class ContainerAllocator>
struct Task_
{
  typedef Task_<ContainerAllocator> Type;

  Task_()
    : task_id(0)
    , deadline(0.0)
    , x_position(0.0)
    , y_position(0.0)
    , z_orientation(0.0)  {
    }
  Task_(const ContainerAllocator& _alloc)
    : task_id(0)
    , deadline(0.0)
    , x_position(0.0)
    , y_position(0.0)
    , z_orientation(0.0)  {
  (void)_alloc;
    }



   typedef uint16_t _task_id_type;
  _task_id_type task_id;

   typedef double _deadline_type;
  _deadline_type deadline;

   typedef double _x_position_type;
  _x_position_type x_position;

   typedef double _y_position_type;
  _y_position_type y_position;

   typedef double _z_orientation_type;
  _z_orientation_type z_orientation;





  typedef boost::shared_ptr< ::taskgen::Task_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::taskgen::Task_<ContainerAllocator> const> ConstPtr;

}; // struct Task_

typedef ::taskgen::Task_<std::allocator<void> > Task;

typedef boost::shared_ptr< ::taskgen::Task > TaskPtr;
typedef boost::shared_ptr< ::taskgen::Task const> TaskConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::taskgen::Task_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::taskgen::Task_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace taskgen

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'taskgen': ['/home/martin/disp2_ws/src/taskgen/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::taskgen::Task_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::taskgen::Task_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::taskgen::Task_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::taskgen::Task_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::taskgen::Task_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::taskgen::Task_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::taskgen::Task_<ContainerAllocator> >
{
  static const char* value()
  {
    return "61469da423d711030840b17fc9caf72f";
  }

  static const char* value(const ::taskgen::Task_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x61469da423d71103ULL;
  static const uint64_t static_value2 = 0x0840b17fc9caf72fULL;
};

template<class ContainerAllocator>
struct DataType< ::taskgen::Task_<ContainerAllocator> >
{
  static const char* value()
  {
    return "taskgen/Task";
  }

  static const char* value(const ::taskgen::Task_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::taskgen::Task_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint16 task_id\n\
float64 deadline\n\
float64 x_position\n\
float64 y_position\n\
float64 z_orientation\n\
";
  }

  static const char* value(const ::taskgen::Task_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::taskgen::Task_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.task_id);
      stream.next(m.deadline);
      stream.next(m.x_position);
      stream.next(m.y_position);
      stream.next(m.z_orientation);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Task_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::taskgen::Task_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::taskgen::Task_<ContainerAllocator>& v)
  {
    s << indent << "task_id: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.task_id);
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

#endif // TASKGEN_MESSAGE_TASK_H
