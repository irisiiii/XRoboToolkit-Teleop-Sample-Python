#include "jiazhua_driver/jiazhua.h"

jiazhua::jiazhua(const char *ifname_)
{
    // 创建套接字
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket");
        return;
    }

    // 指定接口名称
    strcpy(ifr.ifr_name, ifname_);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        return;
    }

    // 绑定套接字到 can0
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return;
    }
}
// 将 0.0~1.0 的 float 值映射为 uint8_t (0~255)
uint8_t jiazhua::float_to_uint8(float value)
{
    // 限制值在 0.0 到 1.0 之间
    if (value < 0.0f) value = 0.0f;
    if (value > 1.0f) value = 1.0f;

    // 映射并四舍五入（乘以 255）
    return (uint8_t)roundf(value * 255.0f);
}
float jiazhua::clamp_float(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}
/*
• id ： 夹爪的CANID
• Pos Cmd ： 目标位置，数值范围 0 - 1，0为夹紧，1为完全张开
• Force Cmd ： 目标力矩，数值范围 0 - 1，1为最大力矩
• Vel Cmd ： 目标速度，数值范围 0 - 1，1为最大速度
• Acc Cmd ： 目标加速度，数值范围 0 - 1，1为最大加速度
•  Dec Cmd ： 目标减速度，数值范围 0 - 1，1为最大减速度
*/
void jiazhua::set_jiazhua(canid_t id, float Pos_Cmd, float Force_Cmd, float Vel_Cmd, float Acc_Cmd, float Dec_Cmd)
{


    // 构造 CAN 帧
    frame.can_id = id;        // 标准帧 ID，3位十六进制
    frame.can_dlc = 8;           // 数据长度（字节数）
    // 你想发的数据 00 00 FF FF FF FF 00 00
    frame.data[0] = 0x00;       //保留数据，写0即可
    frame.data[1] = float_to_uint8(clamp_float(Pos_Cmd, 0, 1));       //目标位置，数值范围 0 - FF，0为夹紧，FF为完全张开
    frame.data[2] = float_to_uint8(clamp_float(Force_Cmd, 0, 1));     //目标力矩，数值范围 0 - FF，FF为最大力矩
    frame.data[3] = float_to_uint8(clamp_float(Vel_Cmd, 0, 1));       //目标速度，数值范围 0 - FF，FF为最大速度
    frame.data[4] = float_to_uint8(clamp_float(Acc_Cmd, 0, 1));       //目标加速度，数值范围 0 - FF，FF为最大加速度
    frame.data[5] = float_to_uint8(clamp_float(Dec_Cmd, 0, 1));       //目标减速度，数值范围 0 - FF，FF为最大减速度
    frame.data[6] = 0x00;       //保留数据，写0即可
    frame.data[7] = 0x00;       //保留数据，写0即可

    // 发送 CAN 帧
    int nbytes = write(s, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        perror("write");
        return;
    }
}

jiazhua::~jiazhua()
{
    close(s);
}
