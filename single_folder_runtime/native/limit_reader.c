#include <errno.h>
#include <gpiod.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static volatile sig_atomic_t g_stop_requested = 0;

static void handle_signal(int signum) {
    (void)signum;
    g_stop_requested = 1;
}

static int parse_int_arg(const char* text, const char* name) {
    char* end = NULL;
    long value = strtol(text, &end, 10);
    if (end == text || *end != '\0') {
        fprintf(stderr, "Invalid integer for %s: %s\n", name, text);
        exit(2);
    }
    return (int)value;
}

static void sleep_ms(int interval_ms) {
    struct timespec ts;
    ts.tv_sec = interval_ms / 1000;
    ts.tv_nsec = (long)(interval_ms % 1000) * 1000000L;
    while (!g_stop_requested && nanosleep(&ts, &ts) < 0) {
        if (errno != EINTR) {
            return;
        }
    }
}

static void usage(const char* prog) {
    fprintf(
        stderr,
        "Usage: %s --pin1 PIN --pin2 PIN [--chip gpiochip0] "
        "[--interval-ms 50] [--active-low 1]\n",
        prog
    );
}

int main(int argc, char** argv) {
    const char* chip_name = "gpiochip0";
    int pin1 = -1;
    int pin2 = -1;
    int interval_ms = 50;
    int active_low = 1;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--chip") == 0 && i + 1 < argc) {
            chip_name = argv[++i];
        } else if (strcmp(argv[i], "--pin1") == 0 && i + 1 < argc) {
            pin1 = parse_int_arg(argv[++i], "--pin1");
        } else if (strcmp(argv[i], "--pin2") == 0 && i + 1 < argc) {
            pin2 = parse_int_arg(argv[++i], "--pin2");
        } else if (strcmp(argv[i], "--interval-ms") == 0 && i + 1 < argc) {
            interval_ms = parse_int_arg(argv[++i], "--interval-ms");
        } else if (strcmp(argv[i], "--active-low") == 0 && i + 1 < argc) {
            active_low = parse_int_arg(argv[++i], "--active-low");
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    if (pin1 < 0 || pin2 < 0 || interval_ms <= 0 || (active_low != 0 && active_low != 1)) {
        usage(argv[0]);
        return 2;
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    struct gpiod_chip* chip = NULL;
    if (strncmp(chip_name, "/dev/", 5) == 0) {
        chip = gpiod_chip_open(chip_name);
    } else {
        chip = gpiod_chip_open_by_name(chip_name);
    }
    if (chip == NULL) {
        fprintf(stderr, "Failed to open GPIO chip %s\n", chip_name);
        return 1;
    }

    struct gpiod_line* line1 = gpiod_chip_get_line(chip, (unsigned int)pin1);
    struct gpiod_line* line2 = gpiod_chip_get_line(chip, (unsigned int)pin2);
    if (line1 == NULL || line2 == NULL) {
        fprintf(stderr, "Failed to get GPIO input lines.\n");
        gpiod_chip_close(chip);
        return 1;
    }

    int flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
    if (gpiod_line_request_input_flags(line1, "pi-bubble-limit-reader", flags) < 0) {
        fprintf(stderr, "Failed to request GPIO%d as input.\n", pin1);
        gpiod_chip_close(chip);
        return 1;
    }
    if (gpiod_line_request_input_flags(line2, "pi-bubble-limit-reader", flags) < 0) {
        fprintf(stderr, "Failed to request GPIO%d as input.\n", pin2);
        gpiod_line_release(line1);
        gpiod_chip_close(chip);
        return 1;
    }

    while (!g_stop_requested) {
        int value1 = gpiod_line_get_value(line1);
        int value2 = gpiod_line_get_value(line2);
        if (value1 < 0 || value2 < 0) {
            fprintf(stderr, "Failed to read GPIO input values.\n");
            break;
        }

        int triggered1 = active_low ? (value1 == 0) : (value1 != 0);
        int triggered2 = active_low ? (value2 == 0) : (value2 != 0);
        printf("%d %d\n", triggered1, triggered2);
        fflush(stdout);
        sleep_ms(interval_ms);
    }

    gpiod_line_release(line2);
    gpiod_line_release(line1);
    gpiod_chip_close(chip);
    return 0;
}
