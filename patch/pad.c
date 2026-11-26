/* q2pad: a Bluetooth Classic game controller as the Q2's own wheel and buttons.
 *
 * The kernel has HIDP and hid-input but no hid-generic, so a controller's HID device never gets a
 * driver or an input node. q2pad leaves the kernel's HID alone and is the HID host itself: it holds
 * the two HID channels (L2CAP PSM 17, control, and 19, interrupt), listening for a controller that
 * reconnects and connecting out to one that links from the Bluetooth menu, decodes the reports, and
 * writes what they mean into the Q2's own input devices, as tools/q2remote.py does: Centre, Return,
 * Play/Pause and the side buttons into md-gpio-keys, and the wheel as a finger on hyn_sliderbar.
 * demo, its menus and every card game see the Q2's hardware and need no change.
 *
 * bluetoothd's input plugin binds PSMs 17 and 19 at start, so q2pad starts first (S90play, before
 * demo starts Bluetooth) and holds them; while anything else has them it retries.
 *
 *   q2pad [-v] [-n] [MAC ...]
 *   -v  print every report and what it maps to
 *   -n  print only: inject nothing (a first look at a new controller's reports)
 *   MAC controllers to connect out to when they link (an incoming controller is learned)
 *
 * Decoded: the Xbox One S / Xbox Wireless Controller's Classic HID reports: report 1, 16 bytes on
 * older firmware (buttons packed, as read from a 1708's log) or 17 (SDL's HIDAPI Xbox One layout);
 * report 2, the Xbox button. Other reports are printed
 * under -v and otherwise ignored. */

#define PSM_CTRL 17
#define PSM_INTR 19
#define FRAME_MS 20    /* a wheel sample, as q2remote's SAMPLE_US */
#define WHEEL_STEP 11  /* units a sample, as q2remote's WHEEL_STEP */
#define WHEEL_SETTLE 4 /* last position repeated before the lift: tslib's dejitter */
#define WHEEL_UNITS 28 /* travel a tick at 50% Wheel Sensitivity */
#define REPEAT_FIRST_MS 380
#define REPEAT_MS 110
#define STICK_ON 0x5000       /* stick past this from centre is a direction (of 0x8000) */
#define STICK_OFF 0x3800      /* and back under this lets it go */
#define CONNECT_DELAY_MS 1500 /* after a link, the controller's own channels first */
#define MAX_PADS 8
#define RETRY_MS 3000 /* the HID PSMs and hci0, while missing */
#define SEEN_MS 2000  /* PAD_SEEN's touch, while a game reads the controller */
#ifndef PAD_SEEN
#define PAD_SEEN "/tmp/q2pad.seen"
#endif
#ifndef CONFIG_INI
#define CONFIG_INI "/mnt/data/config.ini"
#endif
#ifndef MAC_FILE
#define MAC_FILE "/mnt/data/q2pad.macs"
#endif
#ifndef INPUT_DEVICES
#define INPUT_DEVICES "/proc/bus/input/devices"
#endif

#ifdef PAD_HOST
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#define E_AGAIN EAGAIN
#define MSG_NOWAIT MSG_DONTWAIT
#define ERRNO errno
#define E_ADDRINUSE EADDRINUSE
#define SOCK_SEQ SOCK_SEQPACKET
#define SOCK_RAW_ SOCK_RAW
#define O_APPEND_W (O_WRONLY | O_CREAT | O_APPEND)
#define O_TRUNC_W (O_WRONLY | O_CREAT | O_TRUNC)
typedef va_list va_t;
#define va_start_(a, b) va_start(a, b)
#define va_end_(a) va_end(a)
#else
/* MIPS o32 values */
#define O_RDONLY 0
#define O_WRONLY 1
#define O_APPEND_W 0x109 /* O_WRONLY | O_APPEND | O_CREAT */
#define O_TRUNC_W 0x301  /* O_WRONLY | O_CREAT | O_TRUNC */
#define SOCK_SEQ 5
#define SOCK_RAW_ 3
#define E_ADDRINUSE 125
#define CLOCK_MONOTONIC 1
#define POLLIN 1
#define POLLERR 8
#define POLLHUP 0x10
struct pollfd {
    int fd;
    short events, revents;
};
struct timespec {
    long tv_sec, tv_nsec;
};
typedef __builtin_va_list va_t;
#define va_start_(a, b) __builtin_va_start(a, b)
#define va_end_(a) __builtin_va_end(a)
int socket(int, int, int), bind(int, const void *, unsigned), listen(int, int),
    accept(int, void *, void *), connect(int, const void *, unsigned),
    setsockopt(int, int, int, const void *, unsigned),
    getsockopt(int, int, int, void *, unsigned *);
int open(const char *, int, ...), close(int), read(int, void *, unsigned),
    write(int, const void *, unsigned);
int poll(struct pollfd *, unsigned long, int), clock_gettime(int, struct timespec *);
int vsnprintf(char *, unsigned, const char *, va_t), snprintf(char *, unsigned, const char *, ...);
char *strstr(const char *, const char *);
void *memset(void *, int, unsigned), *memcpy(void *, const void *, unsigned);
int memcmp(const void *, const void *, unsigned), strcmp(const char *, const char *);
int *__errno_location(void);
int sendto(int, const void *, unsigned, int, const void *, unsigned);
#define AF_UNIX 1
#define SOCK_DGRAM 1 /* MIPS */
#define MSG_NOWAIT 0x40
#define E_AGAIN 11
struct sockaddr_un {
    unsigned short sun_family;
    char sun_path[108];
};
#define ERRNO (*__errno_location())
#endif

#define AF_BT 31
#define BTPROTO_L2CAP 0
#define BTPROTO_HCI 1
#define SOL_BT 274
#define BT_SECURITY 4
#define BT_SECURITY_MEDIUM 2
#define SOL_HCI 0
#define HCI_FILTER 2
#define EV_SYN 0
#define EV_KEY 1
#define EV_ABS 3
#define BTN_TOUCH 0x14a
#define ABS_X 0
#define ABS_Y 1
#define ABS_PRESSURE 0x18

typedef unsigned char u8;
typedef unsigned short u16;

struct sockaddr_l2 {
    u16 family, psm;
    u8 mac[6];
    u16 cid;
    u8 type, pad;
};
struct sockaddr_hci {
    u16 family, dev, channel;
};
struct hci_filter {
    unsigned type_mask, event_mask[2];
    u16 opcode, pad;
};
struct input_event { /* 32-bit timeval */
    unsigned sec, usec;
    u16 type, code;
    int value;
};

static int verbose, dry;

static long now_ms(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

static void say(const char *fmt, ...) {
    char s[512];
    va_t a;
    va_start_(a, fmt);
    int n = vsnprintf(s, sizeof s - 1, fmt, a);
    va_end_(a);
    if (n < 0) return;
    if (n > (int)sizeof s - 2) n = (int)sizeof s - 2;
    s[n++] = '\n';
    char t[24];
    long ms = now_ms();
    int k = snprintf(t, sizeof t, "q2pad %ld.%03ld: ", ms / 1000 % 100000, ms % 1000);
    write(2, t, (unsigned)k);
    write(2, s, (unsigned)n);
}

static void mac_text(const u8 *m, char *s) { /* bdaddr is little-endian */
    snprintf(s, 18, "%02X:%02X:%02X:%02X:%02X:%02X", m[5], m[4], m[3], m[2], m[1], m[0]);
}

static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int mac_parse(const char *s, u8 *m) {
    for (int i = 5; i >= 0; --i) {
        int h = hexval(s[0]), l = h < 0 ? -1 : hexval(s[1]);
        if (l < 0 || (i && s[2] != ':') || (!i && s[2] > ' ')) return -1;
        m[i] = (u8)(h << 4 | l);
        s += 3;
    }
    return 0;
}

/* ---- the controller's state ---- */

enum {
    B_A = 1 << 0,
    B_B = 1 << 1,
    B_X = 1 << 2,
    B_Y = 1 << 3,
    B_LB = 1 << 4,
    B_RB = 1 << 5,
    B_VIEW = 1 << 6,
    B_MENU = 1 << 7,
    B_GUIDE = 1 << 8,
    B_LS = 1 << 9,
    B_RS = 1 << 10,
    D_UP = 1 << 11,
    D_DOWN = 1 << 12,
    D_LEFT = 1 << 13,
    D_RIGHT = 1 << 14,
};

struct pad {
    unsigned btn;
    u16 lx, ly, rx, ry, lt, rt;
};

static u16 le16(const u8 *p) { return (u16)(p[0] | p[1] << 8); }

/* r: the report from its ID, n bytes. 1 when it changed the state. */
static int xbox_decode(const u8 *r, int n, struct pad *p) {
    static const unsigned hat[9] = {
        0,      D_UP,         D_UP | D_RIGHT, D_RIGHT, D_DOWN | D_RIGHT, D_DOWN, D_DOWN | D_LEFT,
        D_LEFT, D_UP | D_LEFT
    };
    if (n >= 16 && r[0] == 0x01) {
        unsigned b = p->btn & B_GUIDE; /* report 2's */
        p->lx = le16(r + 1), p->ly = le16(r + 3), p->rx = le16(r + 5), p->ry = le16(r + 7);
        p->lt = le16(r + 9), p->rt = le16(r + 11);
        if (r[13] <= 8) b |= hat[r[13]];
        if (n == 16) { /* older firmware (16 bytes): packed A B X Y LB RB View Menu, then LS RS */
            static const unsigned packed[10] = { B_A,  B_B,    B_X,    B_Y,  B_LB,
                                                 B_RB, B_VIEW, B_MENU, B_LS, B_RS };
            unsigned bits = (unsigned)(r[14] | r[15] << 8);
            for (int i = 0; i < 10; ++i)
                if (bits & 1u << i) b |= packed[i];
            p->btn = b;
            return 1;
        }
        if (r[14] & 0x01) b |= B_A;
        if (r[14] & 0x02) b |= B_B;
        if (r[14] & 0x08) b |= B_X;
        if (r[14] & 0x10) b |= B_Y;
        if (r[14] & 0x40) b |= B_LB;
        if (r[14] & 0x80) b |= B_RB;
        if (r[15] & 0x08) b |= B_MENU;
        if (r[15] & 0x10) b |= B_GUIDE;
        if (r[15] & 0x20) b |= B_LS;
        if (r[15] & 0x40) b |= B_RS;
        if (n >= 17 && (r[16] & 0x01)) b |= B_VIEW; /* newer firmware: Consumer AC Back */
        p->btn = b;
        return 1;
    }
    if (n >= 2 && r[0] == 0x02) {
        p->btn = (p->btn & ~(unsigned)B_GUIDE) | (r[1] & 0x01 ? (unsigned)B_GUIDE : 0);
        return 1;
    }
    return 0;
}

/* ---- what it means on a Q2 ---- */

/* md-gpio-keys codes, as tools/q2remote.py's KEYS: Centre is KEY_POWER, Return KEY_UP. */
static const u16 KEYCODE[] = { 116, 103, 108, 165, 163 };
enum { K_CENTRE = 1 << 0, K_RETURN = 1 << 1, K_PLAY = 1 << 2, K_PREV = 1 << 3, K_NEXT = 1 << 4 };
#define NKEYS 5

struct want {
    int wheel; /* -1 up (back), +1 down (next) */
    unsigned keys;
};

/* A stick's axis as -1, 0 or 1, with hysteresis kept in *held. */
static int stick(u16 v, int *held) {
    int d = (int)v - 0x8000, m = d < 0 ? -d : d;
    if (m >= STICK_ON)
        *held = d < 0 ? -1 : 1;
    else if (m < STICK_OFF)
        *held = 0;
    return *held;
}

static void map_pad(const struct pad *p, int *sx, int *sy, struct want *w) {
    int x = stick(p->lx, sx), y = stick(p->ly, sy); /* y: 0 is up */
    w->wheel = 0;
    if ((p->btn & D_UP) || y < 0) w->wheel = -1;
    if ((p->btn & D_DOWN) || y > 0) w->wheel = w->wheel ? 0 : 1;
    w->keys = 0;
    if (p->btn & B_A) w->keys |= K_CENTRE;
    if (p->btn & (B_B | B_VIEW)) w->keys |= K_RETURN;
    if (p->btn & (B_MENU | B_Y)) w->keys |= K_PLAY;
    if ((p->btn & (D_LEFT | B_LB)) || x < 0) w->keys |= K_PREV;
    if ((p->btn & (D_RIGHT | B_RB)) || x > 0) w->keys |= K_NEXT;
}

/* ---- the Q2's input devices ---- */

struct out {
    int keyfd, wheelfd;
    unsigned keys;
    int touching, dir, last, travel16, owed16, settle, flip, units16, raw;
    long next_frame, next_tick;
};

static struct input_event evs[16];
static int nev;

static void ev(u16 type, u16 code, int value) {
    if (nev < 16) evs[nev].type = type, evs[nev].code = code, evs[nev].value = value, ++nev;
}

static void flush(int fd) {
    if (nev && fd >= 0 && !dry) write(fd, evs, (unsigned)nev * sizeof evs[0]);
    nev = 0;
}

/* /dev/input/eventN of the device whose name is name, opened for writing, or -1. */
static int input_open(const char *name) {
    static char buf[8192];
    char want[64], path[32];
    int fd = open(INPUT_DEVICES, O_RDONLY), n = 0, r;
    if (fd < 0) return -1;
    while (n < (int)sizeof buf - 1 && (r = read(fd, buf + n, sizeof buf - 1 - (unsigned)n)) > 0)
        n += r;
    close(fd);
    buf[n] = 0;
    snprintf(want, sizeof want, "N: Name=\"%s\"", name);
    char *at = strstr(buf, want), *h = at ? strstr(at, "H: Handlers=") : 0,
         *e = h ? strstr(h, "event") : 0;
    if (!e) return -1;
    int num = 0;
    for (e += 5; *e >= '0' && *e <= '9'; ++e) num = num * 10 + *e - '0';
    snprintf(path, sizeof path, "/dev/input/event%d", num);
    return open(path, O_WRONLY);
}

/* Travel a tick, in sixteenths, from Q2POD WHEELSENSITIVITY as tools/q2remote.py's wheel_units. */
static int wheel_units16(void) {
    char buf[4096], *k;
    int fd = open(CONFIG_INI, O_RDONLY), n = fd < 0 ? 0 : read(fd, buf, sizeof buf - 1), pct = 0;
    if (fd >= 0) close(fd);
    buf[n < 0 ? 0 : n] = 0;
    if ((k = strstr(buf, "\nWHEELSENSITIVITY=")))
        for (k += 18; *k >= '0' && *k <= '9' && pct < 1000; ++k) pct = pct * 10 + *k - '0';
    if (pct < 10 || pct > 200 || pct % 10) pct = 50;
    return (WHEEL_UNITS * 16 * 50 + pct / 2) / pct;
}

static void keys_apply(struct out *o, unsigned keys) {
    unsigned d = o->keys ^ keys;
    for (int i = 0; i < NKEYS; ++i)
        if (d & 1u << i) ev(EV_KEY, KEYCODE[i], keys & 1u << i ? 1 : 0);
    if (!d) return;
    ev(EV_SYN, 0, 0);
    flush(o->keyfd);
    o->keys = keys;
}

static int wheel_x(const struct out *o) {
    /* As q2remote's wheel_positions: 1-200 a turn from 100, falling for next. */
    int x = (100 + o->travel16 / 16 - 1) % 200;
    return (x < 0 ? x + 200 : x) + 1;
}

static void wheel_sample(struct out *o) {
    ev(EV_ABS, ABS_X, wheel_x(o));
    ev(EV_ABS, ABS_Y, 100 + (o->flip ^= 1)); /* the input core drops a frame with nothing changed */
    ev(EV_SYN, 0, 0);
    flush(o->wheelfd);
}

static void wheel_lift(struct out *o) {
    ev(EV_ABS, ABS_PRESSURE, 0);
    ev(EV_KEY, BTN_TOUCH, 0);
    ev(EV_SYN, 0, 0);
    flush(o->wheelfd);
    o->touching = o->owed16 = o->settle = o->last = 0;
}

/* The wheel direction wanted now: start, extend, reverse or let go of the finger. The first tick a
 * way gets half a tick over, so rounding never loses one (q2remote's wheel_positions); later ticks
 * the same way ride on that carry. */
static void wheel_want(struct out *o, int dir, long now) {
    if (dir == o->dir) return;
    o->dir = dir;
    if (!dir) return; /* the owed travel runs out, then settles and lifts */
    int first = o->units16 * 3 / 2;
    if (!o->touching) {
        o->units16 = wheel_units16();
        first = o->units16 * 3 / 2;
        o->travel16 = 0;
        ev(EV_ABS, ABS_X, 100);
        ev(EV_ABS, ABS_Y, 100);
        ev(EV_ABS, ABS_PRESSURE, 1);
        ev(EV_KEY, BTN_TOUCH, 1);
        ev(EV_SYN, 0, 0);
        flush(o->wheelfd);
        o->touching = 1;
        o->owed16 = dir * first;
    } else if (o->last != dir) {
        o->owed16 = dir * first; /* reversed */
    } else {
        o->owed16 += dir * o->units16;
    }
    o->last = dir;
    o->settle = 0;
    o->next_frame = now;
    o->next_tick = now + REPEAT_FIRST_MS;
}

/* One step of the finger, when due. The ms until the next, or -1 with the finger lifted. */
static int wheel_run(struct out *o, long now) {
    if (!o->touching) return -1;
    if (o->dir && now >= o->next_tick) {
        int cap = o->units16 * 5 / 2;
        o->owed16 += o->dir * o->units16;
        if (o->owed16 > cap) o->owed16 = cap;
        if (o->owed16 < -cap) o->owed16 = -cap;
        o->next_tick += REPEAT_MS;
    }
    if (now < o->next_frame) return (int)(o->next_frame - now);
    o->next_frame = now + FRAME_MS;
    if (o->owed16) {
        int s = o->owed16 > 0 ? WHEEL_STEP * 16 : -WHEEL_STEP * 16;
        if ((s > 0 && s > o->owed16) || (s < 0 && s < o->owed16)) s = o->owed16;
        o->owed16 -= s;
        o->travel16 -= s; /* next (+) falls */
        wheel_sample(o);
        o->settle = 0;
    } else if (!o->dir) {
        if (o->settle++ < WHEEL_SETTLE)
            wheel_sample(o);
        else
            wheel_lift(o);
    }
    return o->touching ? FRAME_MS : -1;
}

static void out_release(struct out *o) {
    keys_apply(o, 0);
    o->dir = 0;
    if (o->touching) wheel_lift(o);
}

/* ---- a game that reads the controller itself ----
 * A game binds a datagram socket at CLIENT_SOCK; while one is there q2pad sends it every report as
 * one text line and presses none of the Q2's controls, except the Xbox button as Return (held, it
 * leaves the game, as Return's hold does). The line:
 *   p BUTTONS LX LY RX RY LT RT
 * BUTTONS in hex as B_* above (A 1, B 2, X 4, Y 8, LB 10, RB 20, View 40, Menu 80, Xbox 100,
 * LS 200, RS 400, D-pad up 800, down 1000, left 2000, right 4000); the sticks -32768..32767 from
 * centre, x rising right and y rising down; the triggers 0..1023. A disconnect sends all zeros. */
#ifndef CLIENT_SOCK
#define CLIENT_SOCK "/tmp/q2pad.client"
#endif
static int ufd = -1;

static int raw_send(const struct pad *p) {
    struct sockaddr_un a;
    char s[96];
    if (ufd < 0) return 0;
    memset(&a, 0, sizeof a);
    a.sun_family = AF_UNIX;
    memcpy(a.sun_path, CLIENT_SOCK, sizeof CLIENT_SOCK);
    int n = snprintf(s, sizeof s, "p %x %d %d %d %d %u %u\n", p->btn, p->lx - 0x8000,
                     p->ly - 0x8000, p->rx - 0x8000, p->ry - 0x8000, p->lt, p->rt);
    if (sendto(ufd, s, (unsigned)n, MSG_NOWAIT, &a, sizeof a) == n) return 1;
    return ERRNO == E_AGAIN; /* a full queue: the game is there, just behind */
}

/* ---- Bluetooth ---- */

struct link {
    int ctrl, intr, sx, sy;
    u8 mac[6];
    struct pad pad;
};

static u8 known[MAX_PADS][6];
static int nknown;

static int is_known(const u8 *m) {
    for (int i = 0; i < nknown; ++i)
        if (!memcmp(known[i], m, 6)) return 1;
    return 0;
}

static void learn(const u8 *m, int save) {
    if (is_known(m) || nknown >= MAX_PADS) return;
    memcpy(known[nknown++], m, 6);
    if (!save) return;
    char s[20];
    mac_text(m, s);
    s[17] = '\n';
    int fd = open(MAC_FILE, O_APPEND_W, 0644);
    if (fd >= 0) write(fd, s, 18), close(fd);
}

static void load_known(void) {
    char buf[MAX_PADS * 18 + 1];
    int fd = open(MAC_FILE, O_RDONLY), n = fd < 0 ? 0 : read(fd, buf, sizeof buf - 1);
    if (fd >= 0) close(fd);
    buf[n < 0 ? 0 : n] = 0;
    u8 m[6];
    for (char *s = buf; s + 17 <= buf + (n < 0 ? 0 : n); s += 18)
        if (!mac_parse(s, m)) learn(m, 0);
}

/* Xbox One S controllers on Classic firmware drop the control channel when the host offers ERTM,
 * L2CAP's enhanced retransmission (the disable_ertm workaround xpadneo documents): basic mode on
 * every channel, and ERTM off for the stack, which is what it advertises in its features. */
#define SOL_L2CAP 6
#define L2CAP_OPTIONS 1
#define ERTM_PARAM "/sys/module/bluetooth/parameters/disable_ertm"
struct l2cap_options {
    u16 omtu, imtu, flush_to;
    u8 mode, fcs, max_tx, pad;
    u16 txwin_size;
};

static void ertm_off(void) {
    char c = 0;
    int fd = open(ERTM_PARAM, O_RDONLY);
    if (fd >= 0) read(fd, &c, 1), close(fd);
    if (c == 'Y') return say("ERTM already off");
    fd = open(ERTM_PARAM, O_WRONLY);
    int ok = fd >= 0 && write(fd, "1", 1) == 1;
    if (fd >= 0) close(fd);
    say(ok ? "ERTM turned off"
           : "can't turn ERTM off (errno %d); channels still ask for basic mode",
        ERRNO);
}

static int l2_socket(void) {
    int fd = socket(AF_BT, SOCK_SEQ, BTPROTO_L2CAP);
    u8 sec[2] = { BT_SECURITY_MEDIUM, 0 };
    if (fd < 0) return fd;
    setsockopt(fd, SOL_BT, BT_SECURITY, sec, sizeof sec);
    struct l2cap_options o;
    unsigned len = sizeof o;
    memset(&o, 0, sizeof o);
    if (!getsockopt(fd, SOL_L2CAP, L2CAP_OPTIONS, &o, &len) && o.mode) {
        o.mode = 0; /* L2CAP_MODE_BASIC */
        setsockopt(fd, SOL_L2CAP, L2CAP_OPTIONS, &o, len);
    }
    return fd;
}

static int l2_listen(u16 psm) {
    struct sockaddr_l2 a;
    memset(&a, 0, sizeof a);
    a.family = AF_BT, a.psm = psm;
    int fd = l2_socket();
    if (fd < 0) return -1;
    if (bind(fd, (void *)&a, sizeof a) || listen(fd, 2)) {
        int e = ERRNO;
        close(fd);
        return e == E_ADDRINUSE ? -2 : -1;
    }
    return fd;
}

static int l2_connect(const u8 *mac, u16 psm) {
    struct sockaddr_l2 a;
    memset(&a, 0, sizeof a);
    a.family = AF_BT, a.psm = psm;
    memcpy(a.mac, mac, 6);
    int fd = l2_socket();
    if (fd >= 0 && connect(fd, (void *)&a, sizeof a)) {
        say("connect PSM %d: errno %d", psm, ERRNO);
        close(fd);
        return -1;
    }
    return fd;
}

static int hci_open(void) {
    struct sockaddr_hci a = { AF_BT, 0, 0 };
    struct hci_filter f;
    memset(&f, 0, sizeof f);
    f.type_mask = 1u << 4;                     /* events */
    f.event_mask[0] = 1u << 0x03 | 1u << 0x05; /* Connection Complete, Disconnection Complete */
    int fd = socket(AF_BT, SOCK_RAW_, BTPROTO_HCI);
    if (fd < 0) return -1;
    if (setsockopt(fd, SOL_HCI, HCI_FILTER, &f, sizeof f) || bind(fd, (void *)&a, sizeof a)) {
        close(fd);
        return -1;
    }
    return fd;
}

static void link_reset(struct link *l) {
    l->ctrl = l->intr = -1;
    memset(&l->pad, 0, sizeof l->pad);
    l->pad.lx = l->pad.ly = l->pad.rx = l->pad.ry = 0x8000; /* sticks centred until a report */
    l->sx = l->sy = 0;
}

static void link_close(struct link *l, struct out *o, const char *why) {
    char s[20];
    mac_text(l->mac, s);
    if (l->ctrl >= 0 || l->intr >= 0) say("%s gone (%s)", s, why);
    if (l->ctrl >= 0) close(l->ctrl);
    if (l->intr >= 0) close(l->intr);
    link_reset(l);
    raw_send(&l->pad); /* a reading game lets go too */
    out_release(o);    /* nothing stays held */
}

static void report(struct link *l, struct out *o, const u8 *buf, int n, long now) {
    if (n < 2 || buf[0] != 0xa1) return; /* DATA | Input */
    int changed = xbox_decode(buf + 1, n - 1, &l->pad);
    struct want w;
    map_pad(&l->pad, &l->sx, &l->sy, &w);
    if (verbose || dry) {
        char s[3 * 64 + 1];
        int k = 0;
        for (int i = 1; i < n && i <= 64; ++i) k += snprintf(s + k, 4, "%02x ", buf[i]);
        s[k] = 0;
        if (changed)
            say("%s| btn %04x L %04x,%04x R %04x,%04x T %u,%u -> wheel %d keys %x", s, l->pad.btn,
                l->pad.lx, l->pad.ly, l->pad.rx, l->pad.ry, l->pad.lt, l->pad.rt, w.wheel, w.keys);
        else
            say("%s| (not decoded)", s);
    }
    if (!changed) return;
    if (o->keyfd < 0) o->keyfd = input_open("md-gpio-keys");
    if (o->wheelfd < 0) o->wheelfd = input_open("hyn_sliderbar");
    if (raw_send(&l->pad)) {
        static long seen_at;
        if (!o->raw) out_release(o), o->raw = 1, say("a game reads the controller");
        /* the game's input never reaches demo, whose game host ends a game idle for 2 minutes
         * (addon.c GAME_IDLE_MS): PAD_SEEN's mtime tells it the controller is in use */
        if (!seen_at || now - seen_at >= SEEN_MS) {
            int fd = open(PAD_SEEN, O_TRUNC_W, 0644);
            if (fd >= 0) write(fd, "1", 1), close(fd);
            seen_at = now;
        }
        keys_apply(o, l->pad.btn & B_GUIDE ? K_RETURN : 0);
        return;
    }
    if (o->raw)
        out_release(o), o->raw = 0, say("no game reads the controller: driving the Q2's controls");
    keys_apply(o, w.keys);
    wheel_want(o, w.wheel, now);
}

#ifndef PAD_NO_MAIN
int main(int argc, char **argv) {
    struct out o;
    struct link l;
    memset(&o, 0, sizeof o);
    memset(&l, 0, sizeof l);
    link_reset(&l);
    load_known();
    for (int i = 1; i < argc; ++i) {
        u8 m[6];
        if (!strcmp(argv[i], "-v"))
            verbose = 1;
        else if (!strcmp(argv[i], "-n"))
            dry = 1;
        else if (!mac_parse(argv[i], m))
            learn(m, 1);
        else
            return say("usage: q2pad [-v] [-n] [MAC ...]"), 2;
    }
    ufd = socket(AF_UNIX, SOCK_DGRAM, 0);
    o.keyfd = o.wheelfd = -1; /* opened at the first report (out_open) */
    ertm_off();
    /* Started by S90play before demo starts Bluetooth: the HID PSMs are held from here on, so
     * bluetoothd's input plugin finds them taken (its server fails, BlueZ goes on without it) and
     * the kernel's HIDP, which has no driver for a controller, never gets one. hci0 comes later
     * (rtk_hciattach), and goes and comes with Bluetooth off and on: the HCI socket is retried. */
    int lc = -1, li = -1, hci = -1, said = 0;
    long retry_at = 0;
    say("started, %d known controller(s)%s", nknown, dry ? ", printing only" : "");

    int pend = 0; /* a known controller linked: connect out at pend_at unless it connects in */
    u8 pend_mac[6] = { 0 };
    long pend_at = 0;
    if (nknown) pend = 1, memcpy(pend_mac, known[0], 6), pend_at = now_ms(); /* already linked? */

    for (;;) {
        long now = now_ms();
        if ((lc < 0 || li < 0 || hci < 0) && now >= retry_at) {
            if (lc < 0) lc = l2_listen(PSM_CTRL);
            if (li < 0) li = l2_listen(PSM_INTR);
            if (lc == -2 || li == -2) {
                if (!(said & 1))
                    say("PSM 17/19 taken (bluetoothd's input plugin started first?): retrying");
                said |= 1;
            } else if (lc >= 0 && li >= 0 && !(said & 2)) {
                say("listening for controllers");
                said |= 2;
            }
            if (lc < -1 || li < -1) lc = lc < 0 ? -1 : lc, li = li < 0 ? -1 : li;
            if (hci < 0 && (hci = hci_open()) >= 0) say("Bluetooth is up");
            retry_at = now + RETRY_MS;
        }
        if (pend && now >= pend_at) {
            pend = 0;
            if (l.ctrl < 0) {
                char s[20];
                mac_text(pend_mac, s);
                int c = l2_connect(pend_mac, PSM_CTRL),
                    d = c < 0 ? -1 : l2_connect(pend_mac, PSM_INTR);
                if (d >= 0)
                    l.ctrl = c, l.intr = d, memcpy(l.mac, pend_mac, 6), say("%s connected", s);
                else if (c >= 0)
                    close(c);
            }
        }
        int wait = wheel_run(&o, now);
        if (lc < 0 || li < 0 || hci < 0) {
            long t = retry_at - now;
            if (wait < 0 || t < wait) wait = t < 0 ? 0 : (int)t;
        }
        if (pend) {
            long t = pend_at - now;
            if (wait < 0 || t < wait) wait = t < 0 ? 0 : (int)t;
        }
        struct pollfd p[5] = { { lc, POLLIN, 0 },
                               { li, POLLIN, 0 },
                               { hci, POLLIN, 0 },
                               { l.ctrl, POLLIN, 0 },
                               { l.intr, POLLIN, 0 } };
        if (poll(p, 5, wait) < 0) continue;
        now = now_ms();
        u8 buf[256];
        struct sockaddr_l2 a;
        unsigned al = sizeof a;
        if (p[0].revents & POLLIN) { /* a controller reconnecting: control first */
            int fd = accept(lc, (void *)&a, (void *)&al);
            if (fd >= 0) {
                if (l.ctrl >= 0) link_close(&l, &o, "replaced");
                l.ctrl = fd, memcpy(l.mac, a.mac, 6);
                char s[20];
                mac_text(a.mac, s);
                say("%s opened control", s);
                learn(a.mac, 1);
                pend = 0;
            }
        }
        if (p[1].revents & POLLIN) {
            al = sizeof a;
            int fd = accept(li, (void *)&a, (void *)&al);
            if (fd >= 0 && l.ctrl >= 0 && l.intr < 0 && !memcmp(a.mac, l.mac, 6)) {
                char s[20];
                mac_text(l.mac, s);
                l.intr = fd, say("%s connected in", s);
            } else if (fd >= 0) {
                char s[20];
                mac_text(a.mac, s);
                say("%s opened interrupt with no control: closed", s);
                close(fd);
            }
        }
        if (p[2].revents & POLLIN) {
            int n = read(hci, buf, sizeof buf);
            char s[20];
            if (n <= 0) { /* hci0 went (Bluetooth off): retry */
                say("Bluetooth is down");
                close(hci), hci = -1, retry_at = now + RETRY_MS;
                continue;
            }
            if (n >= 14 && buf[0] == 4 && buf[1] == 0x03) {
                mac_text(buf + 6, s);
                say("link %s: status 0x%02x handle %d type %d%s", s, buf[3],
                    buf[4] | (buf[5] & 0x0f) << 8, buf[12], is_known(buf + 6) ? " (known)" : "");
            }
            if (n >= 7 && buf[0] == 4 && buf[1] == 0x05) /* status handle(2) reason */
                say("link handle %d dropped: reason 0x%02x%s", buf[4] | (buf[5] & 0x0f) << 8,
                    buf[6],
                    buf[6] == 0x13   ? " (the controller ended it)"
                    : buf[6] == 0x16 ? " (this Q2 ended it)"
                    : buf[6] == 0x08 ? " (timeout)"
                                     : "");
            /* 04 03 0b status handle(2) bdaddr(6) link_type enc: a new ACL link */
            if (n >= 14 && buf[0] == 4 && buf[1] == 0x03 && !buf[3] && buf[12] == 1 &&
                is_known(buf + 6) && l.ctrl < 0) {
                pend = 1, memcpy(pend_mac, buf + 6, 6), pend_at = now + CONNECT_DELAY_MS;
            }
        }
        if (p[3].revents & (POLLIN | POLLERR | POLLHUP)) {
            int n = read(l.ctrl, buf, sizeof buf);
            if (n <= 0) {
                char why[40];
                snprintf(why, sizeof why, "control closed, read %d errno %d", n, n < 0 ? ERRNO : 0);
                link_close(&l, &o, why);
            } else if (verbose || dry) {
                say("control: %d bytes, first %02x", n, buf[0]);
            }
            if (n > 0 && buf[0] == 0x15) link_close(&l, &o, "unplugged");
        }
        if (l.intr >= 0 && (p[4].revents & (POLLIN | POLLERR | POLLHUP))) {
            int n = read(l.intr, buf, sizeof buf);
            if (n <= 0) {
                char why[40];
                snprintf(why, sizeof why, "interrupt closed, read %d errno %d", n,
                         n < 0 ? ERRNO : 0);
                link_close(&l, &o, why);
            } else
                report(&l, &o, buf, n, now);
        }
    }
}
#endif
