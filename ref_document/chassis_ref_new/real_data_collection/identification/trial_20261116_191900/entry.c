/* Separate diagnostic application. Reuse reviewed clock initialization verbatim.
 * This file does not modify or add test motion to the RL control application.
 * Never auto-arm: a new RAM token AND a fresh S1-non-down permit are required.
 * A 2000ms zero-output delay lets the debugger disconnect before activation.
 * S1 down cancels pending requests. Do not read during an active sequence.
 */
#define main production_main_not_used
#include "../../Core/Src/main.c"
#undef main
#include "limited_current.h"
#include "imu_filter.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>
#define LOG_CAPACITY 384U
typedef struct {
    uint32_t begin,tick,phase,reason,s1,remote_age,imu_age,submitted;
    float requested_a,requested_nm,arm[6],velocity[6],feedback[6];
    uint32_t motor_tick[6],motor_received[6];
    uint16_t raw[6]; uint8_t temperature[6],state[6],dm[8],dji[8];
    uint32_t trial_index,loop_us,end;
} IdentFrame;
typedef struct {
    uint32_t magic,schema,frame_bytes,capacity,published,frozen,loop_max_us;
    IdentFrame frame[LOG_CAPACITY];
} IdentLog;
volatile uint32_t g_ident_request_token;
volatile uint32_t g_ident_healthy, g_ident_tick;
volatile LimitedCurrent g_ident;
volatile IdentLog g_ident_log;
MotorBank g_motor_snapshot;
static MotorBank received;
static bool logging;
static uint32_t tail_frames,last_tick,last_special;
void chassis_receive(uint16_t id,const uint8_t data[8],uint32_t now) {
    (void)motor_receive(&received,id,data,8,now);
}
static void special_selected(uint8_t tail) {
    uint8_t d[8]; memset(d,0xff,8); d[7]=tail;
    (void)can2_send(motor_id[IDENT_SELECTED_INDEX],d);
}
static void stop_output(uint32_t reason) {
    ident_stop((LimitedCurrent *)&g_ident,reason);
    can2_cancel_pending();
    const uint8_t z[8]={0}; (void)can2_send(0x3fe,z); (void)can2_send(0x200,z);
    special_selected(0xfd);
}
static void publish(IdentFrame *f) {
    if (!logging || g_ident_log.frozen) return;
    uint32_t n=g_ident_log.published;
    if (n>=LOG_CAPACITY) { stop_output(TIME_STOP); g_ident_log.frozen=1; return; }
    volatile IdentFrame *slot=&g_ident_log.frame[n];
    uint32_t seq=(n+1)*2; slot->begin=seq|1U;
    __asm volatile("" ::: "memory");
    memcpy((void *)&slot->tick,&f->tick,sizeof(*f)-8);
    slot->end=seq;
    __asm volatile("" ::: "memory"); slot->begin=seq;
    g_ident_log.published=n+1;
}
int main(void) {
    SCB_EnableICache(); HAL_Init(); SystemClock_Config(); PeriphCommonClock_Config();
    if (!board_init()) Error_Handler();
    HAL_Delay(100); if (!can2_init()) Error_Handler();
    const uint8_t z[8]={0}; (void)can2_send(0x3fe,z); (void)can2_send(0x200,z);
    (void)imu_init(); board_watchdog_start();
    g_ident_log.magic=0x1D261116U; g_ident_log.schema=2;
    g_ident_log.frame_bytes=sizeof(IdentFrame); g_ident_log.capacity=LOG_CAPACITY;
    last_tick=HAL_GetTick();
    for (;;) {
        uint32_t now=HAL_GetTick();
        if (now==last_tick) { __WFI(); continue; }
        uint32_t start=board_cycles(),gap=now-last_tick; last_tick=now;
        imu_service(now);
        if (now%2) { board_watchdog_feed(); continue; }
        RemoteControl rc; board_remote_snapshot(&rc);
        uint32_t mask=__get_PRIMASK(); __disable_irq(); g_motor_snapshot=received; __set_PRIMASK(mask);
        IdentInput in={.now=now,.token=g_ident_request_token,.remote_age=now-rc.tick,
                       .s1=rc.s1,.remote_valid=rc.valid};
        bool bus=can2_healthy();
        in.healthy=bus && motor_feedback_healthy(&g_motor_snapshot,now) &&
                   g_imu.calibrated && now-g_imu.tick<=ROBOT_IMU_TIMEOUT_MS;
        g_imu_fusion.stationary_allowed=remote_fresh(&rc,now) && rc.s1==2;
        for (unsigned i=0;i<6;++i) {
            in.arm[i]=g_motor_snapshot.motor[i].arm; in.velocity[i]=g_motor_snapshot.motor[i].velocity;
            if (g_motor_snapshot.motor[i].temperature>=45)in.healthy=false;
        }
        if (!isfinite(g_imu.gravity[2]) || g_imu.gravity[2]>-0.9659258f)in.healthy=false;
        g_ident_healthy=in.healthy;g_ident_tick=now;
        uint32_t old_phase=g_ident.phase; float current=0;
        ident_step((LimitedCurrent *)&g_ident,&in,&current);
        if (!bus && (old_phase==ENABLING || old_phase==RUNNING)) stop_output(BUS_STOP);
        if (gap>3 && (g_ident.phase==ENABLING || g_ident.phase==RUNNING)) stop_output(TIME_STOP);
        if (old_phase==ARMED && g_ident.phase==ENABLING) {
            memset((void *)&g_ident_log,0,sizeof(g_ident_log));
            g_ident_log.magic=0x1D261116U;g_ident_log.schema=2;
            g_ident_log.frame_bytes=sizeof(IdentFrame);g_ident_log.capacity=LOG_CAPACITY;
            logging=true;tail_frames=0;last_special=now-20;
        }
        if (old_phase==COOLDOWN && g_ident.phase==ENABLING)last_special=now-20;
        if (g_ident.phase!=RUNNING) current=0;
        if (!isfinite(current) || fabsf(current)>IDENT_CURRENT_LIMIT_A) {stop_output(INVALID_INPUT);current=0;}
        IdentFrame f={.tick=now,.phase=g_ident.phase,.reason=g_ident.reason,.s1=rc.s1,
                      .remote_age=now-rc.tick,.imu_age=now-g_imu.tick,
                      .requested_a=current,.requested_nm=current*ROBOT_DM_KT_NM_A};
        float torque[6]={0};torque[IDENT_SELECTED_INDEX]=f.requested_nm;
        if (!motor_encode(torque,f.dm,f.dji)) {stop_output(INVALID_INPUT);memset(f.dm,0,8);memset(f.dji,0,8);}
        uint32_t spent=(board_cycles()-start)/(SystemCoreClock/1000000U);
        if (spent>=1500) {stop_output(TIME_STOP);memset(f.dm,0,8);memset(f.dji,0,8);}
        RemoteControl final;board_remote_snapshot(&final);
        if ((g_ident.phase==ENABLING || g_ident.phase==RUNNING) &&
            (!remote_fresh(&final,HAL_GetTick()) || final.s1==2)) {
            stop_output(REMOTE_STOP);memset(f.dm,0,8);memset(f.dji,0,8);
        }
        if ((old_phase!=DONE && g_ident.phase==DONE) || (old_phase==RUNNING && g_ident.phase==COOLDOWN)) {
            can2_cancel_pending();memset(f.dm,0,8);memset(f.dji,0,8);
            if(g_ident.phase==DONE)special_selected(0xfd);
        }
        if (can2_send(0x3fe,f.dm))f.submitted|=1;
        if (can2_send(0x200,f.dji))f.submitted|=2;
        if (f.submitted!=3)stop_output(BUS_STOP);
        if (now-last_special>=20) {
            last_special=now;
            if (g_ident.phase!=RUNNING && g_ident.phase!=COOLDOWN)special_selected(g_ident.phase==ENABLING ? 0xfc : 0xfd);
        }
        for (unsigned i=0;i<6;++i) {
            const MotorFeedback *m=&g_motor_snapshot.motor[i];f.arm[i]=m->arm;f.velocity[i]=m->velocity;
            f.feedback[i]=m->current;f.motor_tick[i]=m->tick;f.motor_received[i]=m->received;
            f.raw[i]=m->raw;f.temperature[i]=m->temperature;f.state[i]=m->state;
        }
        f.phase=g_ident.phase;f.reason=g_ident.reason;f.trial_index=g_ident.trial_index;
        f.loop_us=(board_cycles()-start)/(SystemCoreClock/1000000U);
        if (f.loop_us>g_ident_log.loop_max_us)g_ident_log.loop_max_us=f.loop_us;
        if (((g_ident.phase!=COOLDOWN || now-g_ident.since<=32) && now%6==0) || old_phase!=g_ident.phase)publish(&f);
        if ((board_cycles()-start)/(SystemCoreClock/1000000U)>=2000 &&
            (g_ident.phase==ENABLING || g_ident.phase==RUNNING))stop_output(TIME_STOP);
        if (g_ident.phase==DONE && logging && ++tail_frames>=16)g_ident_log.frozen=1;
        board_watchdog_feed();
    }
}
