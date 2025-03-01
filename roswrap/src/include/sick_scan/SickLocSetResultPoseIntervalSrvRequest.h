#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
// Generated by gencpp from file sick_scan/SickLocSetResultPoseIntervalSrvRequest.msg
// DO NOT EDIT!


#ifndef SICK_SCAN_MESSAGE_SICKLOCSETRESULTPOSEINTERVALSRVREQUEST_H
#define SICK_SCAN_MESSAGE_SICKLOCSETRESULTPOSEINTERVALSRVREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace sick_scan
{
template <class ContainerAllocator>
struct SickLocSetResultPoseIntervalSrvRequest_
{
  typedef SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> Type;

  SickLocSetResultPoseIntervalSrvRequest_()
    : interval(0)  {
    }
  SickLocSetResultPoseIntervalSrvRequest_(const ContainerAllocator& _alloc)
    : interval(0)  {
  (void)_alloc;
    }



   typedef int32_t _interval_type;
  _interval_type interval;





  typedef std::shared_ptr< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SickLocSetResultPoseIntervalSrvRequest_

typedef ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<std::allocator<void> > SickLocSetResultPoseIntervalSrvRequest;

typedef std::shared_ptr< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest > SickLocSetResultPoseIntervalSrvRequestPtr;
typedef std::shared_ptr< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest const> SickLocSetResultPoseIntervalSrvRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator1> & lhs, const ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator2> & rhs)
{
  return lhs.interval == rhs.interval;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator1> & lhs, const ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace sick_scan

namespace roswrap
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bebac32626d427b6cff00707fee72ffb";
  }

  static const char* value(const ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbebac32626d427b6ULL;
  static const uint64_t static_value2 = 0xcff00707fee72ffbULL;
};

template<class ContainerAllocator>
struct DataType< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sick_scan/SickLocSetResultPoseIntervalSrvRequest";
  }

  static const char* value(const ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Definition of ROS service SickLocSetResultPoseInterval for sick localization.\n"
"#\n"
"# ROS service SickLocSetResultPoseInterval sets the interval of the pose result output.\n"
"# by sending cola command (\"sMN LocSetResultPoseInterval <interval>\")\n"
"# with parameter\n"
"# <interval>:  0-255, interval in number of scans, 1: result with each processed scan, default: 1\n"
"#\n"
"# See Telegram-Listing-v1.1.0.241R.pdf for further details about \n"
"# Cola telegrams and this command.\n"
"\n"
"#\n"
"# Request (input)\n"
"#\n"
"\n"
"int32 interval # interval in number of scans, 0-255, 1 (default): result with each processed scan\n"
"\n"
;
  }

  static const char* value(const ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace roswrap

namespace roswrap
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.interval);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SickLocSetResultPoseIntervalSrvRequest_

} // namespace serialization
} // namespace roswrap

namespace roswrap
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sick_scan::SickLocSetResultPoseIntervalSrvRequest_<ContainerAllocator>& v)
  {
    s << indent << "interval: ";
    Printer<int32_t>::stream(s, indent + "  ", v.interval);
  }
};

} // namespace message_operations
} // namespace roswrap

#endif // SICK_SCAN_MESSAGE_SICKLOCSETRESULTPOSEINTERVALSRVREQUEST_H
