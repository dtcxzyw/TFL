#ifndef AL_ATOMIC_H
#define AL_ATOMIC_H

#include "static_assert.h"
#include "bool.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void* XchgPtr;

inline void* ExchangePtr(XchgPtr* a, void* b) {
    XchgPtr c = *a;
    *a = b;
    return c;
}

inline int ExchangeInt(int* a, int b) {
    int c = *a;
    *a = b;
    return c;
}

inline int64_t nativeSwap(int64_t* a,int64_t b,int size) {
#define SWAP(s) { int##s##_t old = *(int##s##_t*)a;*(int##s##_t*)a = b;return old;}
    switch (size) {
    case 1: SWAP(8)
    case 2: SWAP(16)
    case 4: SWAP(32)
    case 8: SWAP(64)
    }
    return 0;
#undef SWAP
}

#define ATOMIC(T)  T

#define ATOMIC_INIT(a,b) (*(a)=(b))

#define ATOMIC_INIT_STATIC(_newval) (_newval)

#define ATOMIC_LOAD_UNSAFE(_val)  (*_val)
#define ATOMIC_STORE_UNSAFE(_val, _newval)  (*(_val)=(_newval))

#define ATOMIC_LOAD(_val)  (*_val)
#define ATOMIC_STORE(_val, _newval)  (*(_val)=(_newval))

#define ATOMIC_EXCHANGE(T, _val, _newval) ((T)nativeSwap((_val),(_newval),sizeof(T)))

#define ATOMIC_COMPARE_EXCHANGE_STRONG(T, _val, _oldval, _newval) ATOMIC_EXCHANGE(T,_val,_newval)

/* If no weak cmpxchg is provided (not all systems will have one), substitute a
 * strong cmpxchg. */
#ifndef ATOMIC_COMPARE_EXCHANGE_WEAK
#define ATOMIC_COMPARE_EXCHANGE_WEAK(a, b, c, d) ATOMIC_COMPARE_EXCHANGE_STRONG(a, b, c, d)
#endif

/* This is *NOT* atomic, but is a handy utility macro to compare-and-swap non-
 * atomic variables. */
#define COMPARE_EXCHANGE(_val, _oldval, _newval)  ((*(_val) == *(_oldval)) ? ((*(_val)=(_newval)),true) : ((*(_oldval)=*(_val)),false))

typedef unsigned int uint;
typedef ATOMIC(uint) RefCount;

inline void InitRef(RefCount *ptr, uint value)
{ ATOMIC_INIT(ptr, value); }
inline uint ReadRef(RefCount *ptr)
{ return ATOMIC_LOAD(ptr); }
inline uint IncrementRef(RefCount *ptr)
{ return ++(*ptr); }
inline uint DecrementRef(RefCount *ptr)
{ return --(*ptr); }

#ifdef __cplusplus
}
#endif

#endif /* AL_ATOMIC_H */
