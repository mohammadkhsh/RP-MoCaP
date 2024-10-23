// Generated by gencpp from file custom_msg_f32/Float32MultiArrayPlusStamp.msg
// DO NOT EDIT!


#ifndef CUSTOM_MSG_F32_MESSAGE_FLOAT32MULTIARRAYPLUSSTAMP_H
#define CUSTOM_MSG_F32_MESSAGE_FLOAT32MULTIARRAYPLUSSTAMP_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float32MultiArray.h>

namespace custom_msg_f32
{
template <class ContainerAllocator>
struct Float32MultiArrayPlusStamp_
{
  typedef Float32MultiArrayPlusStamp_<ContainerAllocator> Type;

  Float32MultiArrayPlusStamp_()
    : header()
    , data()  {
    }
  Float32MultiArrayPlusStamp_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , data(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::std_msgs::Float32MultiArray_<ContainerAllocator>  _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> const> ConstPtr;

}; // struct Float32MultiArrayPlusStamp_

typedef ::custom_msg_f32::Float32MultiArrayPlusStamp_<std::allocator<void> > Float32MultiArrayPlusStamp;

typedef boost::shared_ptr< ::custom_msg_f32::Float32MultiArrayPlusStamp > Float32MultiArrayPlusStampPtr;
typedef boost::shared_ptr< ::custom_msg_f32::Float32MultiArrayPlusStamp const> Float32MultiArrayPlusStampConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator1> & lhs, const ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator1> & lhs, const ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace custom_msg_f32

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a2086c942cab182edbf821df15fe0401";
  }

  static const char* value(const ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa2086c942cab182eULL;
  static const uint64_t static_value2 = 0xdbf821df15fe0401ULL;
};

template<class ContainerAllocator>
struct DataType< ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "custom_msg_f32/Float32MultiArrayPlusStamp";
  }

  static const char* value(const ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"std_msgs/Float32MultiArray data\n"
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
"\n"
"================================================================================\n"
"MSG: std_msgs/Float32MultiArray\n"
"# Please look at the MultiArrayLayout message definition for\n"
"# documentation on all multiarrays.\n"
"\n"
"MultiArrayLayout  layout        # specification of data layout\n"
"float32[]         data          # array of data\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/MultiArrayLayout\n"
"# The multiarray declares a generic multi-dimensional array of a\n"
"# particular data type.  Dimensions are ordered from outer most\n"
"# to inner most.\n"
"\n"
"MultiArrayDimension[] dim # Array of dimension properties\n"
"uint32 data_offset        # padding elements at front of data\n"
"\n"
"# Accessors should ALWAYS be written in terms of dimension stride\n"
"# and specified outer-most dimension first.\n"
"# \n"
"# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n"
"#\n"
"# A standard, 3-channel 640x480 image with interleaved color channels\n"
"# would be specified as:\n"
"#\n"
"# dim[0].label  = \"height\"\n"
"# dim[0].size   = 480\n"
"# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n"
"# dim[1].label  = \"width\"\n"
"# dim[1].size   = 640\n"
"# dim[1].stride = 3*640 = 1920\n"
"# dim[2].label  = \"channel\"\n"
"# dim[2].size   = 3\n"
"# dim[2].stride = 3\n"
"#\n"
"# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/MultiArrayDimension\n"
"string label   # label of given dimension\n"
"uint32 size    # size of given dimension (in type units)\n"
"uint32 stride  # stride of given dimension\n"
;
  }

  static const char* value(const ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Float32MultiArrayPlusStamp_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::custom_msg_f32::Float32MultiArrayPlusStamp_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "data: ";
    s << std::endl;
    Printer< ::std_msgs::Float32MultiArray_<ContainerAllocator> >::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CUSTOM_MSG_F32_MESSAGE_FLOAT32MULTIARRAYPLUSSTAMP_H
