#ifndef _ROS_medibotv4_SensorState_h
#define _ROS_medibotv4_SensorState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace medibotv4
{

  class SensorState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _ir1_type;
      _ir1_type ir1;
      typedef float _ir2_type;
      _ir2_type ir2;
      typedef float _ir3_type;
      _ir3_type ir3;
      typedef float _sonar_type;
      _sonar_type sonar;
      typedef float _csens_type;
      _csens_type csens;
      typedef bool _laser1_type;
      _laser1_type laser1;
      typedef bool _laser2_type;
      _laser2_type laser2;
      typedef bool _laser3_type;
      _laser3_type laser3;
      typedef bool _switch_mode_type;
      _switch_mode_type switch_mode;
      typedef bool _estop_type;
      _estop_type estop;
      typedef bool _cs_stt_type;
      _cs_stt_type cs_stt;
      typedef bool _cs_stp_type;
      _cs_stp_type cs_stp;
      typedef bool _cs_fwd_type;
      _cs_fwd_type cs_fwd;
      typedef bool _cs_rvr_type;
      _cs_rvr_type cs_rvr;
      typedef bool _cs_lft_type;
      _cs_lft_type cs_lft;
      typedef bool _cs_rgt_type;
      _cs_rgt_type cs_rgt;

    SensorState():
      header(),
      ir1(0),
      ir2(0),
      ir3(0),
      sonar(0),
      csens(0),
      laser1(0),
      laser2(0),
      laser3(0),
      switch_mode(0),
      estop(0),
      cs_stt(0),
      cs_stp(0),
      cs_fwd(0),
      cs_rvr(0),
      cs_lft(0),
      cs_rgt(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_ir1;
      u_ir1.real = this->ir1;
      *(outbuffer + offset + 0) = (u_ir1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ir1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ir1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ir1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ir1);
      union {
        float real;
        uint32_t base;
      } u_ir2;
      u_ir2.real = this->ir2;
      *(outbuffer + offset + 0) = (u_ir2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ir2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ir2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ir2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ir2);
      union {
        float real;
        uint32_t base;
      } u_ir3;
      u_ir3.real = this->ir3;
      *(outbuffer + offset + 0) = (u_ir3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ir3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ir3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ir3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ir3);
      union {
        float real;
        uint32_t base;
      } u_sonar;
      u_sonar.real = this->sonar;
      *(outbuffer + offset + 0) = (u_sonar.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sonar.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sonar.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sonar.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sonar);
      union {
        float real;
        uint32_t base;
      } u_csens;
      u_csens.real = this->csens;
      *(outbuffer + offset + 0) = (u_csens.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_csens.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_csens.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_csens.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->csens);
      union {
        bool real;
        uint8_t base;
      } u_laser1;
      u_laser1.real = this->laser1;
      *(outbuffer + offset + 0) = (u_laser1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->laser1);
      union {
        bool real;
        uint8_t base;
      } u_laser2;
      u_laser2.real = this->laser2;
      *(outbuffer + offset + 0) = (u_laser2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->laser2);
      union {
        bool real;
        uint8_t base;
      } u_laser3;
      u_laser3.real = this->laser3;
      *(outbuffer + offset + 0) = (u_laser3.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->laser3);
      union {
        bool real;
        uint8_t base;
      } u_switch_mode;
      u_switch_mode.real = this->switch_mode;
      *(outbuffer + offset + 0) = (u_switch_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->switch_mode);
      union {
        bool real;
        uint8_t base;
      } u_estop;
      u_estop.real = this->estop;
      *(outbuffer + offset + 0) = (u_estop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->estop);
      union {
        bool real;
        uint8_t base;
      } u_cs_stt;
      u_cs_stt.real = this->cs_stt;
      *(outbuffer + offset + 0) = (u_cs_stt.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cs_stt);
      union {
        bool real;
        uint8_t base;
      } u_cs_stp;
      u_cs_stp.real = this->cs_stp;
      *(outbuffer + offset + 0) = (u_cs_stp.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cs_stp);
      union {
        bool real;
        uint8_t base;
      } u_cs_fwd;
      u_cs_fwd.real = this->cs_fwd;
      *(outbuffer + offset + 0) = (u_cs_fwd.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cs_fwd);
      union {
        bool real;
        uint8_t base;
      } u_cs_rvr;
      u_cs_rvr.real = this->cs_rvr;
      *(outbuffer + offset + 0) = (u_cs_rvr.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cs_rvr);
      union {
        bool real;
        uint8_t base;
      } u_cs_lft;
      u_cs_lft.real = this->cs_lft;
      *(outbuffer + offset + 0) = (u_cs_lft.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cs_lft);
      union {
        bool real;
        uint8_t base;
      } u_cs_rgt;
      u_cs_rgt.real = this->cs_rgt;
      *(outbuffer + offset + 0) = (u_cs_rgt.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cs_rgt);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_ir1;
      u_ir1.base = 0;
      u_ir1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ir1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ir1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ir1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ir1 = u_ir1.real;
      offset += sizeof(this->ir1);
      union {
        float real;
        uint32_t base;
      } u_ir2;
      u_ir2.base = 0;
      u_ir2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ir2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ir2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ir2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ir2 = u_ir2.real;
      offset += sizeof(this->ir2);
      union {
        float real;
        uint32_t base;
      } u_ir3;
      u_ir3.base = 0;
      u_ir3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ir3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ir3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ir3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ir3 = u_ir3.real;
      offset += sizeof(this->ir3);
      union {
        float real;
        uint32_t base;
      } u_sonar;
      u_sonar.base = 0;
      u_sonar.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sonar.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sonar.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sonar.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sonar = u_sonar.real;
      offset += sizeof(this->sonar);
      union {
        float real;
        uint32_t base;
      } u_csens;
      u_csens.base = 0;
      u_csens.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_csens.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_csens.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_csens.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->csens = u_csens.real;
      offset += sizeof(this->csens);
      union {
        bool real;
        uint8_t base;
      } u_laser1;
      u_laser1.base = 0;
      u_laser1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->laser1 = u_laser1.real;
      offset += sizeof(this->laser1);
      union {
        bool real;
        uint8_t base;
      } u_laser2;
      u_laser2.base = 0;
      u_laser2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->laser2 = u_laser2.real;
      offset += sizeof(this->laser2);
      union {
        bool real;
        uint8_t base;
      } u_laser3;
      u_laser3.base = 0;
      u_laser3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->laser3 = u_laser3.real;
      offset += sizeof(this->laser3);
      union {
        bool real;
        uint8_t base;
      } u_switch_mode;
      u_switch_mode.base = 0;
      u_switch_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->switch_mode = u_switch_mode.real;
      offset += sizeof(this->switch_mode);
      union {
        bool real;
        uint8_t base;
      } u_estop;
      u_estop.base = 0;
      u_estop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->estop = u_estop.real;
      offset += sizeof(this->estop);
      union {
        bool real;
        uint8_t base;
      } u_cs_stt;
      u_cs_stt.base = 0;
      u_cs_stt.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cs_stt = u_cs_stt.real;
      offset += sizeof(this->cs_stt);
      union {
        bool real;
        uint8_t base;
      } u_cs_stp;
      u_cs_stp.base = 0;
      u_cs_stp.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cs_stp = u_cs_stp.real;
      offset += sizeof(this->cs_stp);
      union {
        bool real;
        uint8_t base;
      } u_cs_fwd;
      u_cs_fwd.base = 0;
      u_cs_fwd.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cs_fwd = u_cs_fwd.real;
      offset += sizeof(this->cs_fwd);
      union {
        bool real;
        uint8_t base;
      } u_cs_rvr;
      u_cs_rvr.base = 0;
      u_cs_rvr.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cs_rvr = u_cs_rvr.real;
      offset += sizeof(this->cs_rvr);
      union {
        bool real;
        uint8_t base;
      } u_cs_lft;
      u_cs_lft.base = 0;
      u_cs_lft.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cs_lft = u_cs_lft.real;
      offset += sizeof(this->cs_lft);
      union {
        bool real;
        uint8_t base;
      } u_cs_rgt;
      u_cs_rgt.base = 0;
      u_cs_rgt.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cs_rgt = u_cs_rgt.real;
      offset += sizeof(this->cs_rgt);
     return offset;
    }

    const char * getType(){ return "medibotv4/SensorState"; };
    const char * getMD5(){ return "101d30c4b7d15fe45d7341ba60967c2f"; };

  };

}
#endif
