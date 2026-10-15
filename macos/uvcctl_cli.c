/* Tiny CLI for debugging: uvcctl_cli [vid pid] get | set <value> */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uvcctl.h"

int main(int argc, char **argv) {
    uint16_t vid = 0, pid = 0;
    int i = 1;
    if (argc >= 4) { vid = (uint16_t)strtol(argv[1], NULL, 16); pid = (uint16_t)strtol(argv[2], NULL, 16); i = 3; }
    if (argc <= i) { fprintf(stderr, "usage: %s [vid_hex pid_hex] get | set <value>\n", argv[0]); return 2; }
    uvcctl_set_debug(1);
    uvcctl_handle *h = NULL;
    int rc = uvcctl_open(vid, pid, &h);
    if (rc != UVCCTL_OK) { fprintf(stderr, "open failed (%d): %s\n", rc, uvcctl_last_error(h)); return 1; }
    printf("interface=%u camera_terminal=%u processing_unit=%u\n",
           uvcctl_interface_number(h), uvcctl_camera_terminal_id(h), uvcctl_processing_unit_id(h));
    if (!strcmp(argv[i], "get")) {
        uint16_t v = 0;
        rc = uvcctl_get_zoom(h, &v);
        printf("GET_CUR zoom rc=%d value=0x%04x (%u)\n", rc, v, v);
    } else if (!strcmp(argv[i], "set") && argc > i + 1) {
        uint16_t v = (uint16_t)strtol(argv[i + 1], NULL, 0);
        rc = uvcctl_set_zoom(h, v);
        printf("SET_CUR zoom 0x%04x rc=%d %s\n", v, rc, rc ? uvcctl_last_error(h) : "ok");
    }
    uvcctl_close(h);
    return rc == UVCCTL_OK ? 0 : 1;
}
