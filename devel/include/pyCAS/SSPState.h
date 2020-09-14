// Generated by gencpp from file pyCAS/SSPState.msg
// DO NOT EDIT!


#ifndef PYCAS_MESSAGE_SSPSTATE_H
#define PYCAS_MESSAGE_SSPSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <pyCAS/RobotStatus.h>
#include <pyCAS/ObstacleStatus.h>
#include <pyCAS/Interaction.h>

namespace pyCAS
{
template <class ContainerAllocator>
struct SSPState_
{
  typedef SSPState_<ContainerAllocator> Type;

  SSPState_()
    : header()
    , robot_status()
    , obstacle_status()
    , interaction_status()  {
    }
  SSPState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , robot_status(_alloc)
    , obstacle_status(_alloc)
    , interaction_status(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::pyCAS::RobotStatus_<ContainerAllocator>  _robot_status_type;
  _robot_status_type robot_status;

   typedef  ::pyCAS::ObstacleStatus_<ContainerAllocator>  _obstacle_status_type;
  _obstacle_status_type obstacle_status;

   typedef  ::pyCAS::Interaction_<ContainerAllocator>  _interaction_status_type;
  _interaction_status_type interaction_status;





  typedef boost::shared_ptr< ::pyCAS::SSPState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pyCAS::SSPState_<ContainerAllocator> const> ConstPtr;

}; // struct SSPState_

typedef ::pyCAS::SSPState_<std::allocator<void> > SSPState;

typedef boost::shared_ptr< ::pyCAS::SSPState > SSPStatePtr;
typedef boost::shared_ptr< ::pyCAS::SSPState const> SSPStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pyCAS::SSPState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pyCAS::SSPState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pyCAS::SSPState_<ContainerAllocator1> & lhs, const ::pyCAS::SSPState_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.robot_status == rhs.robot_status &&
    lhs.obstacle_status == rhs.obstacle_status &&
    lhs.interaction_status == rhs.interaction_status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pyCAS::SSPState_<ContainerAllocator1> & lhs, const ::pyCAS::SSPState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pyCAS

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::pyCAS::SSPState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pyCAS::SSPState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pyCAS::SSPState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pyCAS::SSPState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pyCAS::SSPState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pyCAS::SSPState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pyCAS::SSPState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ab0c5c9508d54efc9d177817d64efe0a";
  }

  static const char* value(const ::pyCAS::SSPState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xab0c5c9508d54efcULL;
  static const uint64_t static_value2 = 0x9d177817d64efe0aULL;
};

template<class ContainerAllocator>
struct DataType< ::pyCAS::SSPState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pyCAS/SSPState";
  }

  static const char* value(const ::pyCAS::SSPState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pyCAS::SSPState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"RobotStatus robot_status\n"
"ObstacleStatus obstacle_status\n"
"Interaction interaction_status\n"
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
"MSG: pyCAS/RobotStatus\n"
"Header header\n"
"int8 x_coord\n"
"int8 y_coord\n"
"float32 heading\n"
"================================================================================\n"
"MSG: pyCAS/ObstacleStatus\n"
"Header header\n"
"string obstacle_data\n"
"string door_status\n"
"\n"
"================================================================================\n"
"MSG: pyCAS/Interaction\n"
"Header header\n"
"string status\n"
;
  }

  static const char* value(const ::pyCAS::SSPState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pyCAS::SSPState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.robot_status);
      stream.next(m.obstacle_status);
      stream.next(m.interaction_status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SSPState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pyCAS::SSPState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pyCAS::SSPState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "robot_status: ";
    s << std::endl;
    Printer< ::pyCAS::RobotStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.robot_status);
    s << indent << "obstacle_status: ";
    s << std::endl;
    Printer< ::pyCAS::ObstacleStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.obstacle_status);
    s << indent << "interaction_status: ";
    s << std::endl;
    Printer< ::pyCAS::Interaction_<ContainerAllocator> >::stream(s, indent + "  ", v.interaction_status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PYCAS_MESSAGE_SSPSTATE_H
