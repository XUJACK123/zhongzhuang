#ifndef CAN_TX_AGE_H
#define CAN_TX_AGE_H
#include <stdbool.h>
#include <stdint.h>
#define CAN_TX_NONZERO_ABORT_AGE_MS 4U
typedef struct {uint32_t born_ms[32],nonzero_mask;} CanTxAge;
static inline void can_tx_age_clear(CanTxAge *s) {s->nonzero_mask=0;}
static inline void can_tx_age_record(CanTxAge *s,uint32_t mask,uint32_t now,bool nonzero) {
    for(unsigned i=0;i<32;++i)if(mask&(1U<<i))s->born_ms[i]=now;
    if(nonzero)s->nonzero_mask|=mask;else s->nonzero_mask&=~mask;
}
static inline uint32_t can_tx_age_expired(const CanTxAge *s,uint32_t pending,uint32_t now) {
    uint32_t candidates=pending&s->nonzero_mask,result=0;
    for(unsigned i=0;i<32;++i)if((candidates&(1U<<i)) && (uint32_t)(now-s->born_ms[i])>=CAN_TX_NONZERO_ABORT_AGE_MS)result|=1U<<i;
    return result;
}
#endif
