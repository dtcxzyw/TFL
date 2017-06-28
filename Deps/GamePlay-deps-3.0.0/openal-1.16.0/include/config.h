/* API declaration export attribute */
#define AL_API  
#define ALC_API 

/* Define to the library version */
#define ALSOFT_VERSION "1.16.0"

#ifdef IN_IDE_PARSER
/* KDevelop's parser doesn't recognize the C99-standard restrict keyword, but
 * recent versions (at least 4.5.1) do recognize GCC's __restrict. */
#define restrict __restrict
#endif 

/* Define any available alignment declaration */
#define ALIGN(x) 

/* Define if we have the posix_memalign function */
#define HAVE_POSIX_MEMALIGN

/* Define if we have the _aligned_malloc function */
#define HAVE__ALIGNED_MALLOC

/* Define if we have the OpenSL backend */
#define HAVE_OPENSL

/* Define if we have the stat function */
#define HAVE_STAT

/* Define if we have the lrintf function */
#define HAVE_LRINTF

/* Define if we have the strtof function */
#define HAVE_STRTOF

/* Define if we have the __int64 type */
#define HAVE___INT64

/* Define to the size of a long int type */
#define SIZEOF_LONG 4

/* Define to the size of a long long int type */
#define SIZEOF_LONG_LONG 8

/* Define if we have C99 variable-length array support */
#define HAVE_C99_VLA

/* Define if we have C99 _Bool support */
#define HAVE_C99_BOOL

/* Define if we have C11 _Static_assert support */
#define HAVE_C11_STATIC_ASSERT

/* Define if we have C11 _Alignas support */
#define HAVE_C11_ALIGNAS

/* Define if we have GCC's destructor attribute */
#define HAVE_GCC_DESTRUCTOR

/* Define if we have GCC's format attribute */
#define HAVE_GCC_FORMAT

/* Define if we have stdint.h */
#define HAVE_STDINT_H

/* Define if we have stdbool.h */
#define HAVE_STDBOOL_H

/* Define if we have stdalign.h */
#define HAVE_STDALIGN_H

/* Define if we have dlfcn.h */
#define HAVE_DLFCN_H

/* Define if we have alloca.h */
#define HAVE_ALLOCA_H

/* Define if we have malloc.h */
#define HAVE_MALLOC_H

/* Define if we have ftw.h */
#define HAVE_FTW_H

/* Define if we have io.h */
#define HAVE_IO_H

/* Define if we have strings.h */
#define HAVE_STRINGS_H

/* Define if we have sys/sysconf.h */
#define HAVE_SYS_SYSCONF_H

/* Define if we have float.h */
#define HAVE_FLOAT_H

/* Define if we have fenv.h */
#define HAVE_FENV_H

/* Define if we have _controlfp() */
#define HAVE__CONTROLFP

/* Define if we have __control87_2() */
#define HAVE___CONTROL87_2

/* Define if we have ftw() */
#define HAVE_FTW

/* Define if we have _wfindfirst() */
#define HAVE__WFINDFIRST

/* Define if we have pthread_setschedparam() */
#define HAVE_PTHREAD_SETSCHEDPARAM

/* Define if we have pthread_setname_np() */
#define HAVE_PTHREAD_SETNAME_NP

/* Define if we have pthread_set_name_np() */
#define HAVE_PTHREAD_SET_NAME_NP

#define AL_ALEXT_PROTOTYPES