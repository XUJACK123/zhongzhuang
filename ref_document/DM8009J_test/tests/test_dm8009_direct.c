#include <assert.h>
#include <string.h>
#include "dm8009_direct.h"
int main(void) {
    DM_App a={.stage=DM_READY,.feedback_fresh=1,.multi_count=1,.multi_state=1};
    DM_DirectTest t={0};DBUS_State r;
    DM_DirectTick(&t,&a,0,0,0,&r);assert(t.phase==0 && r.right_switch==DBUS_DOWN);
    DM_DirectTick(&t,&a,1,1,0,&r);assert(t.phase==1);
    DM_DirectTick(&t,&a,151,0,0,&r);assert(t.phase==2 && r.right_switch==DBUS_UP);
    DM_DirectTick(&t,&a,DM_STEP_TIMEOUT_MS+1U,0,0,&r);assert(t.phase==3 && t.result==2 && r.right_switch==DBUS_DOWN);
    DM_DirectTick(&t,&a,13000,1,0,&r);assert(t.phase==3 && r.right_switch==DBUS_DOWN);
    memset(&t,0,sizeof(t));DM_DirectTick(&t,&a,0,1,0,&r);
    DM_DirectTick(&t,&a,2,0,1,&r);assert(t.result==5 && r.right_switch==DBUS_DOWN);
    memset(&t,0,sizeof(t));DM_DirectTick(&t,&a,0,1,0,&r);a.step.completed_count=1;
    DM_DirectTick(&t,&a,2,0,0,&r);assert(t.result==1 && r.right_switch==(DM_STEP_HOLD_AT_TARGET?DBUS_UP:DBUS_DOWN));
    if(DM_STEP_HOLD_AT_TARGET) {
        assert(t.phase==4);DM_DirectTick(&t,&a,DM_STEP_TIMEOUT_MS+100U,0,0,&r);
        assert(t.phase==4 && t.result==1 && r.right_switch==DBUS_UP);
        DM_DirectTick(&t,&a,DM_STEP_TIMEOUT_MS+101U,0,1,&r);
        assert(t.phase==3 && t.result==5 && r.right_switch==DBUS_DOWN);
    }
    memset(&t,0,sizeof(t));a.stage=DM_FAULT;DM_DirectTick(&t,&a,0,1,0,&r);
    assert(t.result==6 && r.right_switch==DBUS_DOWN);
    if(DM_USE_MULTI) {
        a.stage=DM_READY;a.multi_speed_deg_s=2.04f;memset(&t,0,sizeof(t));
        DM_DirectTick(&t,&a,0,1,0,&r);assert(t.phase==1 && !t.precheck_flags);
        a.multi_speed_deg_s=6.1f;memset(&t,0,sizeof(t));
        DM_DirectTick(&t,&a,0,1,0,&r);assert(t.result==6 && t.precheck_flags==16U);
    }
    return 0;
}
