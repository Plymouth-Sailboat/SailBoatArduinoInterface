#ifndef _ROS_dnn_detect_DetectedObject_h
#define _ROS_dnn_detect_DetectedObject_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dnn_detect
{

  class DetectedObject : public ros::Msg
  {
    public:
      typedef const char* _class_name_type;
      _class_name_type class_name;
      typedef float _confidence_type;
      _confidence_type confidence;
      typedef float _x_min_type;
      _x_min_type x_min;
      typedef float _x_max_type;
      _x_max_type x_max;
      typedef float _y_min_type;
      _y_min_type y_min;
      typedef float _y_max_type;
      _y_max_type y_max;

    DetectedObject():
      class_name(""),
      confidence(0),
      x_min(0),
      x_max(0),
      y_min(0),
      y_max(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_class_name = strlen(this->class_name);
      varToArr(outbuffer + offset, length_class_name);
      offset += 4;
      memcpy(outbuffer + offset, this->class_name, length_class_name);
      offset += length_class_name;
      union {
        float real;
        uint32_t base;
      } u_confidence;
      u_confidence.real = this->confidence;
      *(outbuffer + offset + 0) = (u_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence);
      union {
        float real;
        uint32_t base;
      } u_x_min;
      u_x_min.real = this->x_min;
      *(outbuffer + offset + 0) = (u_x_min.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_min.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_min.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_min.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_min);
      union {
        float real;
        uint32_t base;
      } u_x_max;
      u_x_max.real = this->x_max;
      *(outbuffer + offset + 0) = (u_x_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_max.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_max);
      union {
        float real;
        uint32_t base;
      } u_y_min;
      u_y_min.real = this->y_min;
      *(outbuffer + offset + 0) = (u_y_min.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_min.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_min.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_min.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_min);
      union {
        float real;
        uint32_t base;
      } u_y_max;
      u_y_max.real = this->y_max;
      *(outbuffer + offset + 0) = (u_y_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_max.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_max);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_class_name;
      arrToVar(length_class_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_class_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_class_name-1]=0;
      this->class_name = (char *)(inbuffer + offset-1);
      offset += length_class_name;
      union {
        float real;
        uint32_t base;
      } u_confidence;
      u_confidence.base = 0;
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->confidence = u_confidence.real;
      offset += sizeof(this->confidence);
      union {
        float real;
        uint32_t base;
      } u_x_min;
      u_x_min.base = 0;
      u_x_min.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_min.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_min.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_min.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_min = u_x_min.real;
      offset += sizeof(this->x_min);
      union {
        float real;
        uint32_t base;
      } u_x_max;
      u_x_max.base = 0;
      u_x_max.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_max.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_max.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_max.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_max = u_x_max.real;
      offset += sizeof(this->x_max);
      union {
        float real;
        uint32_t base;
      } u_y_min;
      u_y_min.base = 0;
      u_y_min.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_min.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_min.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_min.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y_min = u_y_min.real;
      offset += sizeof(this->y_min);
      union {
        float real;
        uint32_t base;
      } u_y_max;
      u_y_max.base = 0;
      u_y_max.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_max.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_max.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_max.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y_max = u_y_max.real;
      offset += sizeof(this->y_max);
     return offset;
    }

    const char * getType(){ return "dnn_detect/DetectedObject"; };
    const char * getMD5(){ return "c0b57b793d09a04a117353f677edfe58"; };

  };

}
#endif