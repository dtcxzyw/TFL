
#include "config.h"

#include "atomic.h"


extern inline void InitRef(RefCount *ptr, uint value);
extern inline uint ReadRef(RefCount *ptr);
extern inline uint IncrementRef(RefCount *ptr);
extern inline uint DecrementRef(RefCount *ptr);

extern inline int ExchangeInt(int *ptr, int newval);
extern inline void *ExchangePtr(XchgPtr *ptr, void *newval);
extern inline int64_t nativeSwap(int64_t* a, int64_t b, int size);
