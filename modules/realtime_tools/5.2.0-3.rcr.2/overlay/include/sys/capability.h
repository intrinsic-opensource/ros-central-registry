/*
 * Custom minimal sys/capability.h stub for hermetic Bazel builds of realtime_tools.
 * This file declares the standard types and prototypes for libcap.
 * The implementation is linked dynamically at runtime via -lcap on Linux.
 */

#ifndef CUSTOM_SYS_CAPABILITY_H
#define CUSTOM_SYS_CAPABILITY_H

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned int cap_value_t;
typedef struct _cap_struct *cap_t;

typedef enum {
    CAP_EFFECTIVE = 0,
    CAP_PERMITTED = 1,
    CAP_INHERITABLE = 2
} cap_flag_t;

typedef enum {
    CAP_CLEAR = 0,
    CAP_SET = 1
} cap_flag_value_t;

#define CAP_IPC_LOCK 14

cap_t cap_get_proc(void);
int cap_set_flag(cap_t cap_d, cap_flag_t flag, int ncap, const cap_value_t *value_list, cap_flag_value_t state);
int cap_set_proc(cap_t cap_d);
int cap_free(void *obj_d);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_SYS_CAPABILITY_H */
