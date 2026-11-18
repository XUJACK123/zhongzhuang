#include "algorithm/rl.h"
int candidate_actor(const float* o,float* a){RlNetwork n;return rl_actor(&n,o,a);}
