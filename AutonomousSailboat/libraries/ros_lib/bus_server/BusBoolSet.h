#ifndef _ROS_SERVICE_BusBoolSet_h
#define _ROS_SERVICE_BusBoolSet_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bus_server
{

static const char BUSBOOLSET[] = "bus_server/BusBoolSet";

  class BusBoolSetRequest : public ros::Msg
  {
    public:
      typedef int16_t _address_type;
      _address_type address;
      typedef int8_t _command_type;
      _command_type command;
      typedef bool _value_type;
      _value_type value;
      typedef int8_t _retries_type;
      _retries_type retries;
      typedef int8_t _priority_type;
      _priority_type priority;

    BusBoolSetRequest():
      address(0),
      command(0),
      value(0),
      retries(0),
      priority(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_address;
      u_address.real = this->address;
      *(outbuffer + offset + 0) = (u_address.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_address.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->address);
      union {
        int8_t real;
        uint8_t base;
      } u_command;
      u_command.real = this->command;
      *(outbuffer + offset + 0) = (u_command.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->command);
      union {
        bool real;
        uint8_t base;
      } u_value;
      u_value.real = this->value;
      *(outbuffer + offset + 0) = (u_value.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->value);
      union {
        int8_t real;
        uint8_t base;
      } u_retries;
      u_retries.real = this->retries;
      *(outbuffer + offset + 0) = (u_retries.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->retries);
      union {
        int8_t real;
        uint8_t base;
      } u_priority;
      u_priority.real = this->priority;
      *(outbuffer + offset + 0) = (u_priority.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->priority);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_address;
      u_address.base = 0;
      u_address.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_address.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->address = u_address.real;
      offset += sizeof(this->address);
      union {
        int8_t real;
        uint8_t base;
      } u_command;
      u_command.base = 0;
      u_command.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->command = u_command.real;
      offset += sizeof(this->command);
      union {
        bool real;
        uint8_t base;
      } u_value;
      u_value.base = 0;
      u_value.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->value = u_value.real;
      offset += sizeof(this->value);
      union {
        int8_t real;
        uint8_t base;
      } u_retries;
      u_retries.base = 0;
      u_retries.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->retries = u_retries.real;
      offset += sizeof(this->retries);
      union {
        int8_t real;
        uint8_t base;
      } u_priority;
      u_priority.base = 0;
      u_priority.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->priority = u_priority.real;
      offset += sizeof(this->priority);
     return offset;
    }

    const char * getType(){ return BUSBOOLSET; };
    const char * getMD5(){ return "c956462d73c667e51e2dc42f81723318"; };

  };

  class BusBoolSetResponse : public ros::Msg
  {
    public:
      typedef int8_t _status_type;
      _status_type status;

    BusBoolSetResponse():
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return BUSBOOLSET; };
    const char * getMD5(){ return "581cc55c12abfc219e446416014f6d0e"; };

  };

  class BusBoolSet {
    public:
    typedef BusBoolSetRequest Request;
    typedef BusBoolSetResponse Response;
  };

}
#endif
