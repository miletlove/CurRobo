#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 *  平台兼容宏
 * ================================================================ */
/* GCC  packed 属性 (替代 IAR/KEIL 的 __packed) */
#ifndef __packed
  #define __packed   __attribute__((packed))
#endif

/* ================================================================
 *  自定义类型别名 (不重复定义 stdint.h 已有类型)
 * ================================================================ */
typedef float  fp32;
typedef double fp64;
typedef bool   bool_t;

#endif /* STRUCT_TYPEDEF_H */



