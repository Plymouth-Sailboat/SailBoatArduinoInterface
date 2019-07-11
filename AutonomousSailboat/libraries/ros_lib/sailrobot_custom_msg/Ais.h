#ifndef _ROS_sailrobot_custom_msg_Ais_h
#define _ROS_sailrobot_custom_msg_Ais_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sailrobot_custom_msg
{

  class Ais : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int64_t _type_type;
      _type_type type;
      typedef int64_t _repeat_indic_type;
      _repeat_indic_type repeat_indic;
      typedef int64_t _mmsi_type;
      _mmsi_type mmsi;
      typedef int64_t _status_type;
      _status_type status;
      typedef int64_t _rate_of_turn_type;
      _rate_of_turn_type rate_of_turn;
      typedef int64_t _speed_over_ground_type;
      _speed_over_ground_type speed_over_ground;
      typedef const char* _position_accuracy_type;
      _position_accuracy_type position_accuracy;
      typedef float _longitude_type;
      _longitude_type longitude;
      typedef float _latitude_type;
      _latitude_type latitude;
      typedef int64_t _course_over_ground_type;
      _course_over_ground_type course_over_ground;
      typedef int64_t _heading_type;
      _heading_type heading;
      typedef float _distance_type;
      _distance_type distance;

    Ais():
      header(),
      type(0),
      repeat_indic(0),
      mmsi(0),
      status(0),
      rate_of_turn(0),
      speed_over_ground(0),
      position_accuracy(""),
      longitude(0),
      latitude(0),
      course_over_ground(0),
      heading(0),
      distance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int64_t real;
        uint64_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_type.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_type.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_type.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_type.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_type.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->type);
      union {
        int64_t real;
        uint64_t base;
      } u_repeat_indic;
      u_repeat_indic.real = this->repeat_indic;
      *(outbuffer + offset + 0) = (u_repeat_indic.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_repeat_indic.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_repeat_indic.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_repeat_indic.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_repeat_indic.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_repeat_indic.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_repeat_indic.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_repeat_indic.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->repeat_indic);
      union {
        int64_t real;
        uint64_t base;
      } u_mmsi;
      u_mmsi.real = this->mmsi;
      *(outbuffer + offset + 0) = (u_mmsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mmsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mmsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mmsi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mmsi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mmsi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mmsi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mmsi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mmsi);
      union {
        int64_t real;
        uint64_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_status.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_status.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_status.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_status.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_status.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_status.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->status);
      union {
        int64_t real;
        uint64_t base;
      } u_rate_of_turn;
      u_rate_of_turn.real = this->rate_of_turn;
      *(outbuffer + offset + 0) = (u_rate_of_turn.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rate_of_turn.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rate_of_turn.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rate_of_turn.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rate_of_turn.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rate_of_turn.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rate_of_turn.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rate_of_turn.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rate_of_turn);
      union {
        int64_t real;
        uint64_t base;
      } u_speed_over_ground;
      u_speed_over_ground.real = this->speed_over_ground;
      *(outbuffer + offset + 0) = (u_speed_over_ground.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_over_ground.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_over_ground.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_over_ground.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed_over_ground.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed_over_ground.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed_over_ground.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed_over_ground.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed_over_ground);
      uint32_t length_position_accuracy = strlen(this->position_accuracy);
      varToArr(outbuffer + offset, length_position_accuracy);
      offset += 4;
      memcpy(outbuffer + offset, this->position_accuracy, length_position_accuracy);
      offset += length_position_accuracy;
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      union {
        int64_t real;
        uint64_t base;
      } u_course_over_ground;
      u_course_over_ground.real = this->course_over_ground;
      *(outbuffer + offset + 0) = (u_course_over_ground.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_course_over_ground.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_course_over_ground.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_course_over_ground.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_course_over_ground.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_course_over_ground.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_course_over_ground.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_course_over_ground.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->course_over_ground);
      union {
        int64_t real;
        uint64_t base;
      } u_heading;
      u_heading.real = this->heading;
      *(outbuffer + offset + 0) = (u_heading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_heading.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_heading.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_heading.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_heading.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->heading);
      offset += serializeAvrFloat64(outbuffer + offset, this->distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int64_t real;
        uint64_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_type.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->type = u_type.real;
      offset += sizeof(this->type);
      union {
        int64_t real;
        uint64_t base;
      } u_repeat_indic;
      u_repeat_indic.base = 0;
      u_repeat_indic.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_repeat_indic.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_repeat_indic.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_repeat_indic.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_repeat_indic.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_repeat_indic.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_repeat_indic.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_repeat_indic.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->repeat_indic = u_repeat_indic.real;
      offset += sizeof(this->repeat_indic);
      union {
        int64_t real;
        uint64_t base;
      } u_mmsi;
      u_mmsi.base = 0;
      u_mmsi.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mmsi.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mmsi.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mmsi.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mmsi.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mmsi.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mmsi.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mmsi.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mmsi = u_mmsi.real;
      offset += sizeof(this->mmsi);
      union {
        int64_t real;
        uint64_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_status.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_status.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_status.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_status.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_status.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_status.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->status = u_status.real;
      offset += sizeof(this->status);
      union {
        int64_t real;
        uint64_t base;
      } u_rate_of_turn;
      u_rate_of_turn.base = 0;
      u_rate_of_turn.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rate_of_turn.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rate_of_turn.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rate_of_turn.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rate_of_turn.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rate_of_turn.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rate_of_turn.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rate_of_turn.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rate_of_turn = u_rate_of_turn.real;
      offset += sizeof(this->rate_of_turn);
      union {
        int64_t real;
        uint64_t base;
      } u_speed_over_ground;
      u_speed_over_ground.base = 0;
      u_speed_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed_over_ground = u_speed_over_ground.real;
      offset += sizeof(this->speed_over_ground);
      uint32_t length_position_accuracy;
      arrToVar(length_position_accuracy, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_position_accuracy; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_position_accuracy-1]=0;
      this->position_accuracy = (char *)(inbuffer + offset-1);
      offset += length_position_accuracy;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
      union {
        int64_t real;
        uint64_t base;
      } u_course_over_ground;
      u_course_over_ground.base = 0;
      u_course_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_course_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_course_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_course_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_course_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_course_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_course_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_course_over_ground.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->course_over_ground = u_course_over_ground.real;
      offset += sizeof(this->course_over_ground);
      union {
        int64_t real;
        uint64_t base;
      } u_heading;
      u_heading.base = 0;
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_heading.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->heading = u_heading.real;
      offset += sizeof(this->heading);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->distance));
     return offset;
    }

    const char * getType(){ return "sailrobot_custom_msg/Ais"; };
    const char * getMD5(){ return "4be53e55c36ee37d45ff17c6f5ad3256"; };

  };

}
#endif