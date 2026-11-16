#include "../../user/can_tx_age.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    CanTxAge s={0};
    can_tx_age_record(&s,1,100,true);
    assert(can_tx_age_expired(&s,1,103)==0);
    assert(can_tx_age_expired(&s,1,104)==1);
    assert(can_tx_age_expired(&s,0,1000)==0); /* completed requests never aborted */
    can_tx_age_record(&s,1,104,false);
    assert(can_tx_age_expired(&s,1,5000)==0); /* zero/stop packets remain deliverable */
    can_tx_age_record(&s,1,200,true);can_tx_age_record(&s,2,202,true);
    assert(can_tx_age_expired(&s,3,204)==1);
    can_tx_age_record(&s,1,205,true);
    assert(can_tx_age_expired(&s,3,206)==2); /* a reused buffer has a new age */
    can_tx_age_record(&s,0x80000000U,UINT32_MAX-1,true);
    assert(can_tx_age_expired(&s,0x80000000U,1)==0);
    assert(can_tx_age_expired(&s,0x80000000U,2)==0x80000000U);
    can_tx_age_clear(&s);assert(can_tx_age_expired(&s,0xffffffffU,9999)==0);
    puts("Pending nonzero expiry, zero preservation, reuse and tick-wrap tests passed");
    return 0;
}
