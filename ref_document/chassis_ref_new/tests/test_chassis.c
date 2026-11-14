/* Fake HAL/CAN integration only. Never links the physical board drivers. */
#include "board.h"
#include "chassis.h"
#include "fdcan.h"
#include "imu.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
uint32_t SystemCoreClock=1000000;
volatile uint32_t g_can_tx_fail,g_can_rx_overflow;
Imu g_imu;
RemoteControl g_remote;
UART_HandleTypeDef huart5;
static uint32_t now,cycle_count,nonzero_frames,enable_frames,cancel_count;
static bool tx_fail,enabled;
uint32_t HAL_GetTick(void) { return now; }
uint32_t board_cycles(void) { cycle_count+=10; return cycle_count; }
void board_remote_snapshot(RemoteControl *r) { *r=g_remote; }
void board_watchdog_feed(void) {}
bool can2_healthy(void) { return true; }
void can2_cancel_pending(void) { ++cancel_count; }
bool can2_send(uint16_t id,const uint8_t d[8]) {
    if (tx_fail) { ++g_can_tx_fail; return false; }
    if (id==0x3fe || id==0x200) {
        for (unsigned i=0;i<8;++i) if (d[i]) { ++nonzero_frames; break; }
    } else if (id>=1 && id<=4) {
        if (d[7]==0xfc) { ++enable_frames; enabled=true; }
        if (d[7]==0xfd) enabled=false;
    }
    return true;
}
static void reset(void) {
    now=cycle_count=nonzero_frames=enable_frames=cancel_count=0;
    tx_fail=enabled=false; g_can_tx_fail=g_can_rx_overflow=0;
    memset(&g_remote,0,sizeof(g_remote)); memset(&g_imu,0,sizeof(g_imu));
    g_imu.initialized=g_imu.calibrated=true; g_imu.quaternion[0]=1; g_imu.gravity[2]=-1;
    g_remote.valid=true; g_remote.count=1; g_remote.s2=3;
    chassis_init();
}
static void step(uint32_t t,uint8_t s1,bool motors,bool remote) {
    now=t; g_remote.s1=s1;
    if (remote) g_remote.tick=t;
    g_imu.tick=t; g_imu.samples=t+1;
    if (motors) {
        for (unsigned i=0;i<6;++i) {
            uint8_t d[8]={0,0,0,0,0,0,30,0}; d[7]=enabled ? 1 : 0;
            chassis_receive((uint16_t)((i==2 || i==5 ? 0x200 : 0x300)+motor_id[i]),d,t);
        }
    }
    chassis_service(t);
}
static int arm(void) {
    for (uint32_t t=1;t<=20;++t) step(t,2,true,true);
    for (uint32_t t=21;t<=380;++t) step(t,1,true,true);
    CHECK(g_chassis.mode==CONTROL_ACTIVE && enable_frames>0);
    return 0;
}
int test_chassis(void) {
    reset();
    for (uint32_t t=1;t<=400;++t) step(t,1,true,true);
    CHECK(nonzero_frames==0 && enable_frames==0 && g_chassis.mode==CONTROL_STOP);
    if (!ROBOT_ENABLE_OUTPUT) {
        CHECK(g_chassis.inhibit&1); return 0;
    }
    reset(); int error=arm(); if (error) return error;
    CHECK(nonzero_frames>0 && g_chassis.policy_runs>0);
    uint32_t before=nonzero_frames;
    step(381,2,true,true); step(382,2,true,true);
    CHECK(nonzero_frames==before && g_chassis.mode==CONTROL_STOP && cancel_count>0);
    for (uint32_t t=383;t<=440;++t) step(t,1,true,true);
    CHECK(nonzero_frames>before); /* UP runs again; DOWN is the only stop. */

    reset(); error=arm(); if (error) return error;
    for (uint32_t t=381;t<=482;++t) step(t,1,true,false);
    CHECK(g_chassis.mode==CONTROL_STOP && (g_chassis.inhibit&2));
    before=nonzero_frames;
    for (uint32_t t=483;t<=500;++t) step(t,1,true,true);
    CHECK(nonzero_frames==before && g_chassis.mode==CONTROL_STOP);

    reset(); error=arm(); if (error) return error;
    for (uint32_t t=381;t<=412;++t) step(t,1,false,true);
    CHECK(g_chassis.mode==CONTROL_FAULT && (g_chassis.inhibit&4));
    before=nonzero_frames;
    step(413,1,true,true); step(414,1,true,true); CHECK(nonzero_frames==before);

    reset(); error=arm(); if (error) return error;
    g_imu.gravity[2]=0; before=nonzero_frames;
    step(381,1,true,true); step(382,1,true,true);
    CHECK(g_chassis.mode==CONTROL_FAULT && nonzero_frames==before);

    reset(); error=arm(); if (error) return error;
    tx_fail=true; step(381,1,true,true); step(382,1,true,true);
    CHECK(g_chassis.mode==CONTROL_FAULT && g_chassis.fault==1);

    reset(); error=arm(); if (error) return error;
    before=nonzero_frames; step(390,1,true,true);
    CHECK(g_chassis.mode==CONTROL_FAULT && g_chassis.fault==2 && nonzero_frames==before);
    return 0;
}
