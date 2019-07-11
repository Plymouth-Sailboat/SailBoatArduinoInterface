#ifndef _ROS_SERVICE_Detect_h
#define _ROS_SERVICE_Detect_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dnn_detect/DetectedObjectArray.h"

namespace dnn_detect
{

static const char DETECT[] = "dnn_detect/Detect";

  class DetectRequest : public ros::Msg
  {
    public:

    DetectRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return DETECT; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class DetectResponse : public ros::Msg
  {
    public:
      typedef dnn_detect::DetectedObjectArray _result_type;
      _result_type result;

    DetectResponse():
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return DETECT; };
    const char * getMD5(){ return "5234e7b88c3a208257806e2aa3d4d67e"; };

  };

  class Detect {
    public:
    typedef DetectRequest Request;
    typedef DetectResponse Response;
  };

}
#endif
