#ifndef AMP_COMMON_H
#define AMP_COMMON_H

#include <stddef.h>
#include <stdint.h>
#include "openamp.h"

void amp_bind_cb(struct rpmsg_device *rdev, const char *name, uint32_t dest);
int amp_service_cb(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv);

#endif /* AMP_COMMON_H */
