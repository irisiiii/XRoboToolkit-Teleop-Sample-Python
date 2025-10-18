#ifndef __JIAZHUA_H
#define __JIAZHUA_H

#include <iostream>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdint.h>
#include <math.h>


class jiazhua
{
private:
    // char *ifname = "can0";
    int s;
    struct sockaddr_can addr{};
    struct ifreq ifr{};
    struct can_frame frame{};
public:
    jiazhua(const char *ifname_);
    uint8_t float_to_uint8(float value);
    float clamp_float(float value, float min_val, float max_val);
    void set_jiazhua(canid_t id, float Pos_Cmd, float Force_Cmd, float Vel_Cmd, float Acc_Cmd, float Dec_Cmd);
    ~jiazhua();
};

#endif



