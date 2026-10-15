#ifndef UVCCTL_H
#define UVCCTL_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    UVCCTL_OK = 0,
    UVCCTL_EINVAL = -1,
    UVCCTL_ENOMEM = -2,
    UVCCTL_ENODEV = -3,
    UVCCTL_EIO = -4,
    UVCCTL_EACCESS = -5,
};

typedef struct uvcctl_handle uvcctl_handle;

/* vid/pid of 0 match any device; the first UVC video-control interface wins. */
int uvcctl_open(uint16_t vid, uint16_t pid, uvcctl_handle **out);
void uvcctl_close(uvcctl_handle *h);
const char *uvcctl_last_error(uvcctl_handle *h);
void uvcctl_set_debug(int on);

uint8_t uvcctl_interface_number(uvcctl_handle *h);
uint8_t uvcctl_camera_terminal_id(uvcctl_handle *h);
uint8_t uvcctl_processing_unit_id(uvcctl_handle *h);
void uvcctl_set_camera_terminal_id(uvcctl_handle *h, uint8_t id);

/* Generic class request to the VC interface. request: 0x01 SET_CUR, 0x81 GET_CUR, ... */
int uvcctl_request(uvcctl_handle *h, uint8_t request, uint8_t unit_id, uint8_t selector,
                   void *data, uint16_t len);

/* CT_ZOOM_ABSOLUTE_CONTROL, the command register used by Xtherm/HT-301 firmware. */
int uvcctl_set_zoom(uvcctl_handle *h, uint16_t value);
int uvcctl_get_zoom(uvcctl_handle *h, uint16_t *value);

#ifdef __cplusplus
}
#endif
#endif
