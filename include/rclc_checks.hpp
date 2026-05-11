#ifndef BENTO_BOX_ARM_RCLC_CHECKS_HPP
#define BENTO_BOX_ARM_RCLC_CHECKS_HPP

#include "rcl/types.h"

#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK)) {                                                                                     \
      return false;                                                                                                    \
    }                                                                                                                  \
  }
#define RCNOCHECK(fn)                                                                                                  \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    (void)temp_rc;                                                                                                     \
  }

#endif // BENTO_BOX_ARM_RCLC_CHECKS_HPP
