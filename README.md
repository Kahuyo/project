1. 项目总体架构和功能概览
本项目实现的是：上位机（树莓派/PC，跑 Python脚本）通过串口→ESP32开发板（主控），ESP32跑FreeRTOS（实时操作系统，支持多任务），同时用两个UART分别控制两个VESC（电调），VESC驱动电机A/B（针/螺杆）。控制策略为"Host设定参数、ESP32实时控制、VESC电流环驱动"。

2. RTOS原理和项目中如何用 (小白向+源码细分)
什么是RTOS？
RTOS（Real-Time Operating System，实时操作系��），理解为“让一块芯片能像电脑那样多线程工作，而且谁更紧急谁抢先跑”的管理方案。主要概念：

任务（Task）：你代码中的每一个“功能循环”，都能被独立调度。
优先级：哪项工作更重要可以先干，如控制电机必需>发log高>后台存储。
任务切换：RTOS自动帮你准点切换，不用自己写while+delay。
本项目RTOS机制全景+源码
a) 任务（Task）的创建和生命周期
以针控制周期任务为例（needle_micro_task），任务就是个死循环：

C
void needle_micro_task(void *arg) {
    while(nm->micro_active){
        // 每次控制采样...
        vTaskDelayUntil(&last_wake, dt_ticks); // 周期调度
    }
    vTaskDelete(NULL); // 退出自毁
}
创建方式：

C
xTaskCreate(needle_micro_task, "needle_micro", 4096, arg, configMAX_PRIORITIES-2, &nm->micro_task);
// 含义： 入口函数  任务名   栈空间     参数      优先级               任务句柄
变量含义:

4096：分配的栈空间字节数。代码里需要开数组、调库时栈要大点。
configMAX_PRIORITIES-2：任务优先级，数字大越重要。
nm->micro_task：任务的RTOS句柄，便于后续操作（如暂停、删除）。
任务退出和删除（安全释放）

nm->micro_active为false时，自动退出。
vTaskDelete(NULL)删除自身（彻底销毁，不��CPU）。
b) 任务之间的数据通信
本项目采用的是“各任务模块化单独结构体互不干扰”。

例如针和螺杆分别有自己的结构体needle_motor_t和screw_motor_t。
Host串口命令由host_comm_task识别后直接调用框架函数needle_motor_start_micro(&needle, ...)，避免并发竞争。
扩展：如果以后有多个任务需要并行地发命令/传数据，应参考上文“队列”章节（xQueueCreate等，见FreeRTOS文档）。
c) RTOS信号量与互斥锁
示例：添加一个参数互斥

C
// 定义
SemaphoreHandle_t my_mutex = xSemaphoreCreateMutex();
// 写时加锁
xSemaphoreTake(my_mutex, portMAX_DELAY);
shared_var = val;
xSemaphoreGive(my_mutex);
// 读时也加锁
变量解释

SemaphoreHandle_t：就是锁的凭证，谁拿到了才能操作资源。
portMAX_DELAY：最长等多久。0表示不等立即抢，其他正整数可超时。
本案例不存在抢资源，目前可用函数指针直接传。

d) 看门狗机制
什么是看门狗？ 一种“你不定期告诉我你还活着，否则重启你！”的硬件安全机制。

代码实现：

C
#include "esp_task_wdt.h"
esp_task_wdt_init(10, true); // 10s，超时重启
esp_task_wdt_add(NULL); // 本任务加入监控队列
while (1) {
    // 主体代码...
    esp_task_wdt_reset(); // 喂狗！
    vTaskDelay(pdMS_TO_TICKS(10)); // 每隔10ms一轮
}
每个关键任务都该加，有些平台也会全局自动加

3. 串口通信协议
3.1 Host ↔ ESP32 （ASCII行协议）
流程
Host向ESP32 USB串口发一行指令（如START NEEDLE 1.1 8.0\n）

ESP32解析处理核心源码（handle_line函数）

C
static void handle_line(const char *ln){
    char cmd[64];
    sscanf(ln, "%63s", cmd);
    if(strcmp(cmd, "START")==0){
        char target[32];
        sscanf(ln, "START %31s", target);
        if(strcmp(target,"NEEDLE")==0){
            float f, a;
            sscanf(ln, "START NEEDLE %f %f", &f, &a);
            needle_motor_start_micro(g_nm, f, a);
            uart_write_bytes(g_uart_num, "ACK START NEEDLE\n", 17);
        }
    }
    // ...同理STOP、SET KP、STATUS...
}
字符序列样例

START NEEDLE 2.0 5.0\n（代表让needle按2Hz、5deg振荡）
SET KP 0.01\n（改P参数）
STOP NEEDLE\n（停止针）
逻辑说明： 用C标准库sscanf分出命令，实际执行api，返回文本ACK，串口工具直接可见。

3.2 ESP32 ↔ VESC（VESC二进制通讯协议）
帧结构

字节	内容
1	0x02 (header头)
2	payload长度L
3...	CMD+data (实际内容)
L+3,L+4	CRC16校验
L+5	0x03 (尾)
真实发包（如：控制needle的角度）代码片段：

C
uint8_t buf[64];
size_t len = cmd_set_pos_frame(deg, buf, sizeof(buf));
uart_write_bytes(m->uart_num, (const char*)buf, len);

// 实现：cmd_set_pos_frame
size_t cmd_set_pos_frame(double pos_deg, uint8_t *out, size_t out_sz){
    uint8_t payload[5];
    payload[0] = 9; // 9=SET_POS
    int32_t val = (int32_t)(pos_deg * 1000000.0); // 放大处理
    payload[1] = (val >> 24) & 0xFF;
    payload[2] = (val >> 16) & 0xFF;
    payload[3] = (val >> 8) & 0xFF;
    payload[4] = (val) & 0xFF;
    return pack_short_frame(payload, 5, out, out_sz);
}
pack_short_frame解读:

C
size_t pack_short_frame(const uint8_t *payload, size_t payload_len, uint8_t *out, size_t out_sz){
    out[idx++] = 0x02;
    out[idx++] = (uint8_t)payload_len;
    memcpy(&out[idx], payload, payload_len); idx += payload_len;
    uint16_t crc = crc16_ccitt(payload, payload_len);
    out[idx++] = (crc >> 8) & 0xFF;
    out[idx++] = crc & 0xFF;
    out[idx++] = 0x03;
    return idx;
}
这样就生成完整协议包，直接写串口即可

4. 控制算法源码详细注释（两轴详细分开！！！！）
4.1 针（needle）的控制算法 + 代码全过程
核心调度任务：

C
static void needle_micro_task(void *arg){
    ... 
    while(nm->micro_active){
        float phase = 2.0f*M_PI*freq*t;
        float raw_target = nm->center + amp*sinf(phase); // 正弦波目标
        float desired_vel = amp*2.0f*M_PI*freq*cosf(phase); // 目标速度
        float pred_target = raw_target + desired_vel * nm->ff_scale * NEEDLE_FEEDBACK_LATENCY_S; // 预测补偿
        double fb_d = vesc_motor_get_position_deg(&nm->vesc, 0.02);
        float fb = isnan(fb_d) ? nm->last_target : (float)fb_d;
        float err = pred_target - fb;
        float corr = nm->kp * err;
        float max_corr = amp * 0.6f;
        corr = (corr < -max_corr ? -max_corr : (corr > max_corr ? max_corr : corr));
        float cmd_target = raw_target + corr;
        float cmd = _limit_step(nm, cmd_target, NEEDLE_OSC_DT); // 做速度加速度限幅
        vesc_motor_set_pos_deg(&nm->vesc, cmd);
        nm->last_target = cmd;
        t += NEEDLE_OSC_DT;
        vTaskDelayUntil(&last_wake, dt_ticks);
    }
}
KEY变量解释说明：

phase：时刻对应的波形进度，正弦运动轨迹
amp：幅值（单位deg），主控Host下发
freq：频率（单位Hz），主控Host下发
center：机械中心，初始由config.h给定，支持实时自校准
pred_target：目标轨迹点+运动速度*延迟，前馈补偿防止滞后
err：实时误差，P参数可调，实际调KP最常用
_limit_step 限幅函数，保证不会机械冲击/跳动过快
vesc_motor_set_pos_deg 本质上就是“发VESC协议包，指定新机械目标”
你问为什么这样做？

P环+前馈是最直观的方案，小白能看懂，高级可拓展为PID/自适应。
物理系统多为延迟环系统，加前馈减弱抖动和延迟误差。
4.2 螺杆（screw）的控制算法 + 代码全过程
螺杆采用P+前馈直接控制速度（rpm为单位），基本流程如下：

C
static void screw_micro_task(void *arg){
    ...
    while (sm->micro_active) {
        float phase = 2.0f * M_PI * freq * t;
        float target_mm = amp_mm * sinf(phase);
        float desired_vel_mm_s = amp_mm * 2.0f * M_PI * freq * cosf(phase);
        double pos_deg_d = vesc_motor_get_position_deg(&sm->vesc, 0.05);
        float current_mm = deg_to_mm(isnan(pos_deg_d) ? zero_deg : ((float)pos_deg_d - zero_deg));
        float err_mm = target_mm - current_mm;
        float rpm_ff = -SCREW_OSC_VEL_FF_GAIN * (desired_vel_mm_s * 60.0f / SCREW_LEAD_MM_PER_REV); // 速度前馈，力图直接驱动
        float rpm_p = -SCREW_OSC_KP_RPM_PER_MM * err_mm; // 位置P环
        float rpm_cmd_f = rpm_ff + rpm_p;
        float rpm_cmd = fmaxf(-SCREW_OSC_MAX_RPM, fminf(SCREW_OSC_MAX_RPM, rpm_cmd_f)); // 限幅
        vesc_motor_set_rpm(&sm->vesc, (int)roundf(rpm_cmd));
        ...
        vTaskDelayUntil(&last_wake, dt_ticks);
    }
}
KEY变量注释：

单位mm为主，便��人理解，所有实际输出要转化为电机角度、VESC的目标速度（rpm）
错误量为（目标-实际），乘以增益得到控制输出量
前馈项根据目标速度，直接指定应该给多少输出
最后一定要限幅rpm，防止暴力/故障
