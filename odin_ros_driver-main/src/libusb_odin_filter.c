#define _GNU_SOURCE

#include <dlfcn.h>
#include <stdint.h>
#include <stdlib.h>
#include <libusb-1.0/libusb.h>

static int env_int(const char *name)
{
    const char *value = getenv(name);
    if (!value || !*value) {
        return -1;
    }
    return atoi(value);
}

static int filter_enabled(void)
{
    return env_int("ODIN_USB_BUS") >= 0 && env_int("ODIN_USB_ADDR") >= 0;
}

static int is_target_device(libusb_device *dev)
{
    const int bus = env_int("ODIN_USB_BUS");
    const int addr = env_int("ODIN_USB_ADDR");
    return (int)libusb_get_bus_number(dev) == bus &&
           (int)libusb_get_device_address(dev) == addr;
}

typedef int (*real_get_desc_fn)(libusb_device *, struct libusb_device_descriptor *);
typedef int (*real_open_fn)(libusb_device *, libusb_device_handle **);

int libusb_get_device_descriptor(libusb_device *dev, struct libusb_device_descriptor *desc)
{
    real_get_desc_fn real_get_desc =
        (real_get_desc_fn)dlsym(RTLD_NEXT, "libusb_get_device_descriptor");

    int rc = real_get_desc(dev, desc);
    if (rc != 0 || !filter_enabled()) {
        return rc;
    }

    if (desc->idVendor == 0x2207 && desc->idProduct == 0x0019 && !is_target_device(dev)) {
        desc->idVendor = 0xffff;
        desc->idProduct = 0xffff;
    }

    return rc;
}

int libusb_open(libusb_device *dev, libusb_device_handle **handle)
{
    real_open_fn real_open = (real_open_fn)dlsym(RTLD_NEXT, "libusb_open");

    if (filter_enabled() && !is_target_device(dev)) {
        struct libusb_device_descriptor desc;
        real_get_desc_fn real_get_desc =
            (real_get_desc_fn)dlsym(RTLD_NEXT, "libusb_get_device_descriptor");
        if (real_get_desc(dev, &desc) == 0 &&
            desc.idVendor == 0x2207 && desc.idProduct == 0x0019) {
            return LIBUSB_ERROR_NO_DEVICE;
        }
    }

    return real_open(dev, handle);
}
