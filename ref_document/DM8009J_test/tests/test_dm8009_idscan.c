#include <assert.h>
#include <stdio.h>
#include "dm8009_app.h"
static uint8_t request[8];
static uint32_t sent, pending;
static int Send(void *ctx, uint16_t id, const uint8_t data[8]) {
    (void)ctx;
    assert(id==0x7FF && data[2]==0x33 && data[3]==7);
    for(unsigned i=0;i<8;++i)request[i]=data[i];
    ++sent; pending=1; return 1;
}
static void Cancel(void *ctx) {(void)ctx;}
int main(void) {
    DM_App a; DM_AppInit(&a,0);
    DM_IO io={.send=Send,.cancel_pending=Cancel};
    DBUS_State remote={.valid=1,.right_switch=DBUS_UP,.left_switch=DBUS_UP};
    for(uint32_t now=0;now<50000;now+=2) {
        remote.last_rx_ms=now;
        remote.right_switch=remote.left_switch=(now/100U)%2U?DBUS_UP:DBUS_DOWN;
        DM_AppTick(&a,now,&remote,1,&io);
        if(pending) {
            pending=0;
            uint16_t id=(uint16_t)(request[0]|(uint16_t)request[1]<<8);
            if(id==0x123)DM_AppReceive(&a,now,0,request);
        }
    }
    assert(a.stage==DM_IDENTIFIED && a.motor_id==0x123 && a.master_id==0);
    assert(sent==2*(DM_SCAN_LAST_ID-DM_SCAN_FIRST_ID+1));
    DM_AppTick(&a,50000,&remote,0,&io);
    DM_AppTick(&a,50002,&remote,1,&io);
    assert(sent==2*(DM_SCAN_LAST_ID-DM_SCAN_FIRST_ID+1)); /* fault path also cannot write */
    puts("DM8009 ID-only image: exclusively RID7 reads, including after discovery/fault PASS");
    return 0;
}
