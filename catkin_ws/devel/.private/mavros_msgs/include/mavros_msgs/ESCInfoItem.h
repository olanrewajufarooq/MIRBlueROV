// Generated by gencpp from file mavros_msgs/ESCInfoItem.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_ESCINFOITEM_H
#define MAVROS_MSGS_MESSAGE_ESCINFOITEM_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace mavros_msgs
{
template <class ContainerAllocator>
struct ESCInfoItem_
{
  typedef ESCInfoItem_<ContainerAllocator> Type;

  ESCInfoItem_()
    : header()
    , failure_flags(0)
    , error_count(0)
    , temperature(0)  {
    }
  ESCInfoItem_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , failure_flags(0)
    , error_count(0)
    , temperature(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint16_t _failure_flags_type;
  _failure_flags_type failure_flags;

   typedef uint32_t _error_count_type;
  _error_count_type error_count;

   typedef int32_t _temperature_type;
  _temperature_type temperature;





  typedef boost::shared_ptr< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> const> ConstPtr;

}; // struct ESCInfoItem_

typedef ::mavros_msgs::ESCInfoItem_<std::allocator<void> > ESCInfoItem;

typedef boost::shared_ptr< ::mavros_msgs::ESCInfoItem > ESCInfoItemPtr;
typedef boost::shared_ptr< ::mavros_msgs::ESCInfoItem const> ESCInfoItemConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mavros_msgs::ESCInfoItem_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mavros_msgs::ESCInfoItem_<ContainerAllocator1> & lhs, const ::mavros_msgs::ESCInfoItem_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.failure_flags == rhs.failure_flags &&
    lhs.error_count == rhs.error_count &&
    lhs.temperature == rhs.temperature;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mavros_msgs::ESCInfoItem_<ContainerAllocator1> & lhs, const ::mavros_msgs::ESCInfoItem_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mavros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> >
{
  static const char* value()
  {
    return "968983ff768dda90f04c5aa11caf6e74";
  }

  static const char* value(const ::mavros_msgs::ESCInfoItem_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x968983ff768dda90ULL;
  static const uint64_t static_value2 = 0xf04c5aa11caf6e74ULL;
};

template<class ContainerAllocator>
struct DataType< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros_msgs/ESCInfoItem";
  }

  static const char* value(const ::mavros_msgs::ESCInfoItem_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ESCInfoItem.msg\n"
"#\n"
"#\n"
"# See mavlink message documentation here:\n"
"# https://mavlink.io/en/messages/common.html#ESC_INFO\n"
"\n"
"std_msgs/Header header\n"
"\n"
"uint16 failure_flags\n"
"uint32 error_count\n"
"int32 temperature\n"
"\n"
"\n"
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
;
  }

  static const char* value(const ::mavros_msgs::ESCInfoItem_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.failure_flags);
      stream.next(m.error_count);
      stream.next(m.temperature);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ESCInfoItem_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mavros_msgs::ESCInfoItem_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "failure_flags: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.failure_flags);
    s << indent << "error_count: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.error_count);
    s << indent << "temperature: ";
    Printer<int32_t>::stream(s, indent + "  ", v.temperature);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_ESCINFOITEM_H
