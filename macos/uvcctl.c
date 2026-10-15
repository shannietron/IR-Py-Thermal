/*
 * uvcctl.c - minimal UVC Camera-Terminal control access on macOS via IOKit.
 *
 * Xtherm / HT-301 / InfiRay thermal cameras use the UVC "Zoom Absolute" control
 * (CT_ZOOM_ABSOLUTE_CONTROL, selector 0x0B) as a command register. On Linux this
 * is reached through V4L2 / cv2.CAP_PROP_ZOOM; OpenCV's AVFoundation backend
 * does not expose it, and libusb needs root on modern macOS. This helper talks to
 * the Video-Control interface through IOUSBInterfaceInterface->ControlRequest,
 * which works even while Apple's UVC driver is streaming the camera (same
 * technique as jtfrey/uvc-util and itaybre/CameraController).
 *
 * Build: see Makefile (clang -shared -framework IOKit -framework CoreFoundation)
 * API:   see uvcctl.h
 */
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/usb/IOUSBLib.h>
#include <IOKit/usb/USB.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "uvcctl.h"

#define UVC_SET_CUR 0x01
#define UVC_GET_CUR 0x81
#define UVC_GET_MIN 0x82
#define UVC_GET_MAX 0x83
#define UVC_GET_RES 0x84
#define UVC_GET_INFO 0x86
#define CT_ZOOM_ABSOLUTE_CONTROL 0x0B

#define CS_INTERFACE 0x24
#define VC_HEADER 0x01
#define VC_INPUT_TERMINAL 0x02
#define VC_PROCESSING_UNIT 0x05

struct uvcctl_handle {
    IOUSBInterfaceInterface500 **intf;
    uint8_t interface_number;
    uint8_t camera_terminal_id;
    uint8_t processing_unit_id;
    int is_open;
    int exclusive;
    char last_error[256];
};

static int g_debug = 0;

static void set_err(uvcctl_handle *h, const char *fmt, IOReturn rc) {
    if (!h) return;
    snprintf(h->last_error, sizeof(h->last_error), fmt, (unsigned)rc);
    if (g_debug) fprintf(stderr, "uvcctl: %s\n", h->last_error);
}

void uvcctl_set_debug(int on) { g_debug = on; }

const char *uvcctl_last_error(uvcctl_handle *h) {
    return h ? h->last_error : "null handle";
}

static int32_t prop_int(io_service_t svc, const char *key) {
    CFTypeRef v = IORegistryEntryCreateCFProperty(
        svc, CFStringCreateWithCString(NULL, key, kCFStringEncodingUTF8),
        kCFAllocatorDefault, 0);
    int32_t out = -1;
    if (v) {
        if (CFGetTypeID(v) == CFNumberGetTypeID()) CFNumberGetValue(v, kCFNumberSInt32Type, &out);
        CFRelease(v);
    }
    return out;
}

/* Walk the class-specific VC descriptors to find terminal/unit IDs. */
static void parse_vc_descriptors(uvcctl_handle *h) {
    IOUSBDescriptorHeader *desc = NULL;
    desc = (*h->intf)->FindNextAssociatedDescriptor(h->intf, NULL, kUSBAnyDesc);
    while (desc) {
        if (desc->bDescriptorType == CS_INTERFACE && desc->bLength >= 3) {
            uint8_t *p = (uint8_t *)desc;
            uint8_t subtype = p[2];
            if (subtype == VC_INPUT_TERMINAL && desc->bLength >= 8) {
                uint16_t wTerminalType = p[4] | (p[5] << 8);
                /* 0x0201 = ITT_CAMERA */
                if (wTerminalType == 0x0201 || h->camera_terminal_id == 0) {
                    h->camera_terminal_id = p[3];
                    if (g_debug) fprintf(stderr, "uvcctl: input terminal id=%u type=0x%04x\n", p[3], wTerminalType);
                }
            } else if (subtype == VC_PROCESSING_UNIT && desc->bLength >= 4) {
                h->processing_unit_id = p[3];
                if (g_debug) fprintf(stderr, "uvcctl: processing unit id=%u\n", p[3]);
            }
        }
        desc = (*h->intf)->FindNextAssociatedDescriptor(h->intf, desc, kUSBAnyDesc);
    }
}

static io_service_t find_vc_interface(uint16_t vid, uint16_t pid, const char *class_name) {
    CFMutableDictionaryRef match = IOServiceMatching(class_name);
    if (!match) return IO_OBJECT_NULL;
    io_iterator_t it = IO_OBJECT_NULL;
    if (IOServiceGetMatchingServices(kIOMainPortDefault, match, &it) != kIOReturnSuccess)
        return IO_OBJECT_NULL;
    io_service_t svc, found = IO_OBJECT_NULL;
    while ((svc = IOIteratorNext(it))) {
        int32_t v = prop_int(svc, "idVendor");
        int32_t p = prop_int(svc, "idProduct");
        int32_t cls = prop_int(svc, "bInterfaceClass");
        int32_t sub = prop_int(svc, "bInterfaceSubClass");
        if (g_debug)
            fprintf(stderr, "uvcctl: %s vid=%04x pid=%04x class=%d sub=%d num=%d\n",
                    class_name, v, p, cls, sub, prop_int(svc, "bInterfaceNumber"));
        if ((vid == 0 || v == vid) && (pid == 0 || p == pid) && cls == 14 && sub == 1) {
            found = svc;
            break;
        }
        IOObjectRelease(svc);
    }
    IOObjectRelease(it);
    return found;
}

int uvcctl_open(uint16_t vid, uint16_t pid, uvcctl_handle **out) {
    if (!out) return UVCCTL_EINVAL;
    *out = NULL;
    uvcctl_handle *h = calloc(1, sizeof(*h));
    if (!h) return UVCCTL_ENOMEM;
    h->camera_terminal_id = 0;
    h->processing_unit_id = 0;

    io_service_t svc = find_vc_interface(vid, pid, "IOUSBHostInterface");
    if (!svc) svc = find_vc_interface(vid, pid, "IOUSBInterface");
    if (!svc) {
        set_err(h, "no UVC video-control interface found for device (rc=%u)", 0);
        *out = h;
        return UVCCTL_ENODEV;
    }

    IOCFPlugInInterface **plugin = NULL;
    SInt32 score = 0;
    IOReturn rc = IOCreatePlugInInterfaceForService(svc, kIOUSBInterfaceUserClientTypeID,
                                                    kIOCFPlugInInterfaceID, &plugin, &score);
    IOObjectRelease(svc);
    if (rc != kIOReturnSuccess || !plugin) {
        set_err(h, "IOCreatePlugInInterfaceForService failed rc=0x%08x", rc);
        *out = h;
        return UVCCTL_EIO;
    }
    HRESULT hr = (*plugin)->QueryInterface(plugin, CFUUIDGetUUIDBytes(kIOUSBInterfaceInterfaceID500),
                                           (LPVOID *)&h->intf);
    (*plugin)->Release(plugin);
    if (hr != S_OK || !h->intf) {
        set_err(h, "QueryInterface(IOUSBInterfaceInterface500) failed hr=0x%08x", (IOReturn)hr);
        *out = h;
        return UVCCTL_EIO;
    }
    (*h->intf)->GetInterfaceNumber(h->intf, &h->interface_number);
    parse_vc_descriptors(h);
    if (h->camera_terminal_id == 0) h->camera_terminal_id = 1;
    if (h->processing_unit_id == 0) h->processing_unit_id = 2;

    rc = (*h->intf)->USBInterfaceOpen(h->intf);
    if (rc == kIOReturnSuccess) {
        h->is_open = 1;
    } else if (rc == kIOReturnExclusiveAccess) {
        /* Apple's driver holds it; control requests still go through (uvc-util does this). */
        h->is_open = 1;
        h->exclusive = 1;
        if (g_debug) fprintf(stderr, "uvcctl: interface held by another client, continuing\n");
    } else {
        set_err(h, "USBInterfaceOpen failed rc=0x%08x", rc);
        *out = h;
        return UVCCTL_EACCESS;
    }
    *out = h;
    return UVCCTL_OK;
}

void uvcctl_close(uvcctl_handle *h) {
    if (!h) return;
    if (h->intf) {
        if (h->is_open && !h->exclusive) (*h->intf)->USBInterfaceClose(h->intf);
        (*h->intf)->Release(h->intf);
    }
    free(h);
}

uint8_t uvcctl_interface_number(uvcctl_handle *h) { return h ? h->interface_number : 0; }
uint8_t uvcctl_camera_terminal_id(uvcctl_handle *h) { return h ? h->camera_terminal_id : 0; }
uint8_t uvcctl_processing_unit_id(uvcctl_handle *h) { return h ? h->processing_unit_id : 0; }
void uvcctl_set_camera_terminal_id(uvcctl_handle *h, uint8_t id) { if (h) h->camera_terminal_id = id; }

int uvcctl_request(uvcctl_handle *h, uint8_t request, uint8_t unit_id, uint8_t selector,
                   void *data, uint16_t len) {
    if (!h || !h->intf) return UVCCTL_EINVAL;
    int is_in = (request & 0x80) != 0;
    IOUSBDevRequest req;
    memset(&req, 0, sizeof(req));
    req.bmRequestType = USBmakebmRequestType(is_in ? kUSBIn : kUSBOut, kUSBClass, kUSBInterface);
    req.bRequest = request;
    req.wValue = (uint16_t)(selector << 8);
    req.wIndex = (uint16_t)((unit_id << 8) | h->interface_number);
    req.wLength = len;
    req.pData = data;
    IOReturn rc = (*h->intf)->ControlRequest(h->intf, 0, &req);
    if (rc != kIOReturnSuccess) {
        set_err(h, "ControlRequest failed rc=0x%08x", rc);
        return UVCCTL_EIO;
    }
    return UVCCTL_OK;
}

int uvcctl_set_zoom(uvcctl_handle *h, uint16_t value) {
    uint8_t buf[2] = { (uint8_t)(value & 0xFF), (uint8_t)(value >> 8) };
    return uvcctl_request(h, UVC_SET_CUR, h ? h->camera_terminal_id : 1,
                          CT_ZOOM_ABSOLUTE_CONTROL, buf, 2);
}

int uvcctl_get_zoom(uvcctl_handle *h, uint16_t *value) {
    uint8_t buf[2] = {0, 0};
    int rc = uvcctl_request(h, UVC_GET_CUR, h ? h->camera_terminal_id : 1,
                            CT_ZOOM_ABSOLUTE_CONTROL, buf, 2);
    if (rc == UVCCTL_OK && value) *value = (uint16_t)(buf[0] | (buf[1] << 8));
    return rc;
}
