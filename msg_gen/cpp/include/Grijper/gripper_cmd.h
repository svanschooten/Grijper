/* Auto-generated by genmsg_cpp for file /home/stijn/ros/Grijper/msg/gripper_cmd.msg */
#ifndef GRIJPER_MESSAGE_GRIPPER_CMD_H
#define GRIJPER_MESSAGE_GRIPPER_CMD_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace Grijper
{
template <class ContainerAllocator>
struct gripper_cmd_ {
  typedef gripper_cmd_<ContainerAllocator> Type;

  gripper_cmd_()
  : cmd()
  , force(0.0)
  {
  }

  gripper_cmd_(const ContainerAllocator& _alloc)
  : cmd(_alloc)
  , force(0.0)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _cmd_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  cmd;

  typedef float _force_type;
  float force;


  typedef boost::shared_ptr< ::Grijper::gripper_cmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::Grijper::gripper_cmd_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct gripper_cmd
typedef  ::Grijper::gripper_cmd_<std::allocator<void> > gripper_cmd;

typedef boost::shared_ptr< ::Grijper::gripper_cmd> gripper_cmdPtr;
typedef boost::shared_ptr< ::Grijper::gripper_cmd const> gripper_cmdConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::Grijper::gripper_cmd_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::Grijper::gripper_cmd_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace Grijper

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::Grijper::gripper_cmd_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::Grijper::gripper_cmd_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::Grijper::gripper_cmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "eb00b428ff12135fc9b18d757044344c";
  }

  static const char* value(const  ::Grijper::gripper_cmd_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xeb00b428ff12135fULL;
  static const uint64_t static_value2 = 0xc9b18d757044344cULL;
};

template<class ContainerAllocator>
struct DataType< ::Grijper::gripper_cmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Grijper/gripper_cmd";
  }

  static const char* value(const  ::Grijper::gripper_cmd_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::Grijper::gripper_cmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string cmd\n\
float32 force\n\
";
  }

  static const char* value(const  ::Grijper::gripper_cmd_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::Grijper::gripper_cmd_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.cmd);
    stream.next(m.force);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct gripper_cmd_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::Grijper::gripper_cmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::Grijper::gripper_cmd_<ContainerAllocator> & v) 
  {
    s << indent << "cmd: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.cmd);
    s << indent << "force: ";
    Printer<float>::stream(s, indent + "  ", v.force);
  }
};


} // namespace message_operations
} // namespace ros

#endif // GRIJPER_MESSAGE_GRIPPER_CMD_H

