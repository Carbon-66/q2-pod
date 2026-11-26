/* Host check of patch/pad.c's decoder, mapping and wheel finger (no Bluetooth, no Q2).
 *   cc -DPAD_HOST -DPAD_NO_MAIN -DCLIENT_SOCK='"/tmp/q2pad-test.client"'
 * -DPAD_SEEN='"/tmp/q2pad-test.seen"' -I patch test/pad_host.c -o /tmp/pad_host && /tmp/pad_host
 * The firmware side is modelled as tools/q2remote.py drives it: a tick per WHEEL_UNITS (28) of
 * travel at 50%, positive (next) as x falls, carried across samples while the finger is down. */
#include "../patch/pad.c"
#include <sys/un.h>

static int fails;
#define CHECK(c, ...)                                                                              \
    do {                                                                                           \
        if (!(c)) {                                                                                \
            ++fails;                                                                               \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);                                            \
            printf(__VA_ARGS__);                                                                   \
            printf("\n");                                                                          \
        }                                                                                          \
    } while (0)

static int wheel_pipe[2], key_pipe[2];

/* The firmware's view: ticks from the samples written since the last call. */
static int fw_ticks(int *down, int *x0, int *acc) {
    struct input_event e[256];
    int n = (int)read(wheel_pipe[0], e, sizeof e) / (int)sizeof e[0], ticks = 0;
    for (int i = 0; i < n; ++i) {
        if (e[i].type == EV_KEY && e[i].code == BTN_TOUCH) {
            *down = e[i].value;
            if (!*down) *acc = 0, *x0 = -1;
        }
        if (e[i].type == EV_ABS && e[i].code == ABS_X) {
            if (*x0 >= 0) {
                int d = *x0 - e[i].value; /* falling x is next (+) */
                if (d > 100) d -= 200;
                if (d < -100) d += 200;
                *acc += d * 16;
                while (*acc >= 28 * 16) *acc -= 28 * 16, ++ticks;
                while (*acc <= -28 * 16) *acc += 28 * 16, --ticks;
            }
            *x0 = e[i].value;
        }
    }
    return ticks;
}

static int key_log(char *s) { /* "+116 -116 " */
    struct input_event e[64];
    int n = (int)read(key_pipe[0], e, sizeof e) / (int)sizeof e[0], k = 0;
    s[0] = 0;
    for (int i = 0; i < n; ++i)
        if (e[i].type == EV_KEY) k += sprintf(s + k, "%c%d ", e[i].value ? '+' : '-', e[i].code);
    return k;
}

static u8 rep[18];
static void set_report(int hat, u8 b14, u8 b15, u16 lx, u16 ly) {
    memset(rep, 0, sizeof rep);
    rep[0] = 0xa1, rep[1] = 0x01;
    rep[2] = (u8)lx, rep[3] = (u8)(lx >> 8), rep[4] = (u8)ly, rep[5] = (u8)(ly >> 8);
    rep[6] = rep[8] = 0x00, rep[7] = rep[9] = 0x80;
    rep[14] = (u8)hat, rep[15] = b14, rep[16] = b15;
}

/* Run the finger from t to t+ms in 1 ms steps, as the main loop would, and count ticks. */
static int run(struct out *o, long *t, int ms, int *down, int *x0, int *acc) {
    int ticks = 0;
    for (long end = *t + ms; *t < end; ++*t) {
        wheel_run(o, *t);
        ticks += fw_ticks(down, x0, acc);
    }
    return ticks;
}

int main(void) {
    if (pipe(wheel_pipe) || pipe(key_pipe)) return 1;
    fcntl(wheel_pipe[0], F_SETFL, O_NONBLOCK);
    fcntl(key_pipe[0], F_SETFL, O_NONBLOCK);
    struct out o;
    struct link l;
    memset(&o, 0, sizeof o);
    link_reset(&l);
    o.keyfd = key_pipe[1], o.wheelfd = wheel_pipe[1];
    long t = 1000;
    int down = 0, x0 = -1, acc = 0, ticks;
    char s[256];

    /* decoder */
    set_report(0, 0x01 | 0x80, 0x08, 0x8000, 0x8000);
    CHECK(xbox_decode(rep + 1, 17, &l.pad) && l.pad.btn == (B_A | B_RB | B_MENU), "btn %x",
          l.pad.btn);
    /* 16 bytes (older firmware, from a real 1708's log): packed A B X Y LB RB View Menu | LS RS */
    static const struct {
        u8 b14, b15;
        unsigned want;
    } packed[] = { { 0x01, 0, B_A },  { 0x04, 0, B_X },  { 0x08, 0, B_Y },    { 0x02, 0, B_B },
                   { 0x20, 0, B_RB }, { 0x10, 0, B_LB }, { 0x40, 0, B_VIEW }, { 0x80, 0, B_MENU },
                   { 0, 0x01, B_LS }, { 0, 0x02, B_RS } };
    for (unsigned i = 0; i < sizeof packed / sizeof packed[0]; ++i) {
        set_report(7, packed[i].b14, packed[i].b15, 0x8000, 0x8000);
        xbox_decode(rep + 1, 16, &l.pad);
        CHECK(l.pad.btn == (D_LEFT | packed[i].want), "packed %u: btn %x", i, l.pad.btn);
    }
    /* the log's own bytes: LB, RT full, left stick far left */
    u8 real[] = {
        0x01, 0x18, 0x84, 0x66, 0x7c, 0xb5, 0x7f, 0x1d, 0x82, 0, 0, 0xff, 0x03, 0, 0x10, 0
    };
    xbox_decode(real, 16, &l.pad);
    CHECK(l.pad.btn == B_LB && l.pad.rt == 1023, "real: btn %x rt %u", l.pad.btn, l.pad.rt);
    u8 g[] = { 0x02, 0x01 };
    xbox_decode(g, 2, &l.pad);
    CHECK(l.pad.btn & B_GUIDE, "guide");
    CHECK(!xbox_decode((const u8 *)"\x05\x00", 2, &l.pad), "unknown report decoded");
    link_reset(&l);

    /* A press and release: Centre down, then up */
    set_report(0, 0x01, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    set_report(0, 0, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    key_log(s);
    CHECK(!strcmp(s, "+116 -116 "), "A: %s", s);

    /* LB and D-pad left both Prev: one down, one up when both are let go */
    set_report(7, 0x40, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    set_report(0, 0x40, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    set_report(0, 0, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    key_log(s);
    CHECK(!strcmp(s, "+165 -165 "), "Prev: %s", s);

    /* D-pad down tapped (60 ms): exactly one tick next, then the finger lifts */
    set_report(5, 0, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    ticks = run(&o, &t, 60, &down, &x0, &acc);
    set_report(0, 0, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    ticks += run(&o, &t, 300, &down, &x0, &acc);
    CHECK(ticks == 1 && !down && !o.touching, "tap down: %d ticks, down %d", ticks, down);

    /* D-pad up tapped: one tick back */
    set_report(1, 0, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    ticks = run(&o, &t, 40, &down, &x0, &acc);
    set_report(0, 0, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    ticks += run(&o, &t, 300, &down, &x0, &acc);
    CHECK(ticks == -1 && !down, "tap up: %d ticks", ticks);

    /* held 1 s: 1 + (1000-380)/110 repeats = 6 or 7 ticks, smooth, then lifts */
    set_report(5, 0, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    ticks = run(&o, &t, 1000, &down, &x0, &acc);
    set_report(0, 0, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    ticks += run(&o, &t, 400, &down, &x0, &acc);
    CHECK(ticks >= 6 && ticks <= 7 && !down, "hold 1 s: %d ticks", ticks);

    /* left stick pushed up past the threshold, then back to centre */
    set_report(0, 0, 0, 0x8000, 0x1000);
    report(&l, &o, rep, 18, t);
    ticks = run(&o, &t, 50, &down, &x0, &acc);
    set_report(0, 0, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    ticks += run(&o, &t, 300, &down, &x0, &acc);
    CHECK(ticks == -1, "stick up: %d ticks", ticks);

    /* down, then reversed to up while the finger is still on: one each way */
    set_report(5, 0, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    ticks = run(&o, &t, 60, &down, &x0, &acc);
    CHECK(ticks == 1, "before reversal: %d", ticks);
    set_report(1, 0, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    ticks = run(&o, &t, 60, &down, &x0, &acc);
    set_report(0, 0, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    ticks += run(&o, &t, 300, &down, &x0, &acc);
    CHECK(ticks == -1, "after reversal: %d", ticks);

    /* two quick taps the same way, the second during the first's settle: two ticks */
    for (int i = 0; i < 2; ++i) {
        set_report(5, 0, 0, 0x8000, 0x8000);
        report(&l, &o, rep, 18, t);
        ticks = run(&o, &t, 50, &down, &x0, &acc);
        set_report(0, 0, 0, 0x8000, 0x8000);
        report(&l, &o, rep, 18, t);
        ticks += run(&o, &t, i ? 300 : 30, &down, &x0, &acc);
        CHECK(ticks == 1, "quick tap %d: %d", i, ticks);
    }

    /* a disconnect mid-hold lets everything go */
    set_report(5, 0x01, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    run(&o, &t, 100, &down, &x0, &acc);
    key_log(s);
    link_close(&l, &o, "test");
    fw_ticks(&down, &x0, &acc);
    key_log(s);
    CHECK(!down && !o.touching && !strcmp(s, "-116 "), "disconnect: down %d keys %s", down, s);

    /* a game binds CLIENT_SOCK: it gets the lines, the Q2's controls are left alone but for Xbox */
    ufd = socket(AF_UNIX, SOCK_DGRAM, 0);
    unlink(CLIENT_SOCK);
    unlink(PAD_SEEN);
    int game = socket(AF_UNIX, SOCK_DGRAM, 0);
    struct sockaddr_un ga = { AF_UNIX, CLIENT_SOCK };
    CHECK(!bind(game, (void *)&ga, sizeof ga), "bind client");
    fcntl(game, F_SETFL, O_NONBLOCK);
    char line[128];
    link_reset(&l);
    set_report(5, 0x01, 0, 0x8000, 0x1000);
    report(&l, &o, rep, 18, t); /* D-pad down, A, stick up */
    run(&o, &t, 200, &down, &x0, &acc);
    int ln = (int)recv(game, line, sizeof line - 1, 0);
    line[ln > 0 ? ln : 0] = 0;
    CHECK(!strcmp(line, "p 1001 0 -28672 0 0 0 0\n"), "line: %s", line);
    key_log(s);
    CHECK(o.raw && !o.touching && !s[0], "raw: keys '%s' touching %d", s, o.touching);
    CHECK(!access(PAD_SEEN, 0), "no %s for Q2 Pod's idle timer", PAD_SEEN);
    u8 xb[] = { 0xa1, 0x02, 0x01 };
    report(&l, &o, xb, 3, t);
    recv(game, line, sizeof line, 0);
    key_log(s);
    CHECK(!strcmp(s, "+103 "), "Xbox as Return: %s", s);
    u8 xb0[] = { 0xa1, 0x02, 0x00 };
    report(&l, &o, xb0, 3, t);
    recv(game, line, sizeof line, 0);
    key_log(s);
    CHECK(!strcmp(s, "-103 "), "Xbox up: %s", s);
    link_close(&l, &o, "test");
    ln = (int)recv(game, line, sizeof line - 1, 0);
    line[ln > 0 ? ln : 0] = 0;
    CHECK(!strcmp(line, "p 0 0 0 0 0 0 0\n"), "disconnect line: %s", line);
    /* the game ends (its socket file stays): q2pad drives the Q2 again */
    close(game);
    link_reset(&l);
    set_report(0, 0x01, 0, 0x8000, 0x8000);
    report(&l, &o, rep, 18, t);
    key_log(s);
    CHECK(!o.raw && !strcmp(s, "+116 "), "after the game: raw %d keys %s", o.raw, s);
    unlink(CLIENT_SOCK);
    unlink(PAD_SEEN);

    printf(fails ? "%d failed\n" : "all passed\n", fails);
    return fails != 0;
}
