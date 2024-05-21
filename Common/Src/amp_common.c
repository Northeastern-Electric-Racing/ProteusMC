#include "amp_common.h"
#include <stdio.h>
#include "openamp.h"

void amp_bind_cb(struct rpmsg_device *rdev, const char *name, uint32_t dest)
{
    (void)rdev;
    (void)dest;

    printf("New service bound: %s\n", name);

    // TODO actually implement endpoint handling
    // struct rpmsg_endpoint *new_ept = malloc(sizeof(struct rpmsg_endpoint));
    // if (new_ept != NULL)
    // {
    //     rpmsg_create_ept(new_ept, rdev, name, RPMSG_ADDR_ANY, dest, rpmsg_service_cb, NULL);
    // }
    // else
    // {
    //     // Handle memory allocation failure
    //     printf("Failed to allocate memory for new endpoint\n");
    // }
}

int amp_service_cb(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv)
{
    (void)ept;
    (void)src;
    (void)priv;

    // Handle incoming message
    printf("Received message: %s\n", (char *)data);

    return 0;
}
