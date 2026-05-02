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

static long long monotonic_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((long long)ts.tv_sec * 1000000000LL) + (long long)ts.tv_nsec;
}

static void sleep_until_ns(long long target_ns) {
    struct timespec ts;
    ts.tv_sec = (time_t)(target_ns / 1000000000LL);
    ts.tv_nsec = (long)(target_ns % 1000000000LL);
    while (!g_stop_requested) {
        int rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        if (rc == 0) {
            return;
        }
        if (rc != EINTR) {
            return;
        }
    }
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

static double parse_double_arg(const char* text, const char* name) {
    char* end = NULL;
    double value = strtod(text, &end);
    if (end == text || *end != '\0') {
        fprintf(stderr, "Invalid number for %s: %s\n", name, text);
        exit(2);
    }
    return value;
}

static void usage(const char* prog) {
    fprintf(
        stderr,
        "Usage: %s --pul PIN --dir PIN --ena PIN --dir-state 0|1 "
        "--ena-active 0|1 --ena-inactive 0|1 --frequency HZ --seconds SEC "
        "[--chip gpiochip0]\n",
        prog
    );
}

int main(int argc, char** argv) {
    const char* chip_name = "gpiochip0";
    int pul_pin = -1;
    int dir_pin = -1;
    int ena_pin = -1;
    int dir_state = -1;
    int ena_active = -1;
    int ena_inactive = -1;
    double frequency = 0.0;
    double seconds = 0.0;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--chip") == 0 && i + 1 < argc) {
            chip_name = argv[++i];
        } else if (strcmp(argv[i], "--pul") == 0 && i + 1 < argc) {
            pul_pin = parse_int_arg(argv[++i], "--pul");
        } else if (strcmp(argv[i], "--dir") == 0 && i + 1 < argc) {
            dir_pin = parse_int_arg(argv[++i], "--dir");
        } else if (strcmp(argv[i], "--ena") == 0 && i + 1 < argc) {
            ena_pin = parse_int_arg(argv[++i], "--ena");
        } else if (strcmp(argv[i], "--dir-state") == 0 && i + 1 < argc) {
            dir_state = parse_int_arg(argv[++i], "--dir-state");
        } else if (strcmp(argv[i], "--ena-active") == 0 && i + 1 < argc) {
            ena_active = parse_int_arg(argv[++i], "--ena-active");
        } else if (strcmp(argv[i], "--ena-inactive") == 0 && i + 1 < argc) {
            ena_inactive = parse_int_arg(argv[++i], "--ena-inactive");
        } else if (strcmp(argv[i], "--frequency") == 0 && i + 1 < argc) {
            frequency = parse_double_arg(argv[++i], "--frequency");
        } else if (strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
            seconds = parse_double_arg(argv[++i], "--seconds");
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    if (
        pul_pin < 0 || dir_pin < 0 || ena_pin < 0 ||
        (dir_state != 0 && dir_state != 1) ||
        (ena_active != 0 && ena_active != 1) ||
        (ena_inactive != 0 && ena_inactive != 1) ||
        frequency <= 0.0 || seconds <= 0.0
    ) {
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

    struct gpiod_line* pul = gpiod_chip_get_line(chip, (unsigned int)pul_pin);
    struct gpiod_line* dir = gpiod_chip_get_line(chip, (unsigned int)dir_pin);
    struct gpiod_line* ena = gpiod_chip_get_line(chip, (unsigned int)ena_pin);
    if (pul == NULL || dir == NULL || ena == NULL) {
        fprintf(stderr, "Failed to get GPIO lines.\n");
        gpiod_chip_close(chip);
        return 1;
    }

    int pul_requested = 0;
    int dir_requested = 0;
    int ena_requested = 0;
    if (gpiod_line_request_output(pul, "pi-bubble-stepper", 0) < 0) {
        fprintf(stderr, "Failed to request GPIO output lines.\n");
        gpiod_chip_close(chip);
        return 1;
    }
    pul_requested = 1;
    if (gpiod_line_request_output(dir, "pi-bubble-stepper", dir_state) < 0) {
        fprintf(stderr, "Failed to request GPIO output lines.\n");
        gpiod_line_release(pul);
        gpiod_chip_close(chip);
        return 1;
    }
    dir_requested = 1;
    if (gpiod_line_request_output(ena, "pi-bubble-stepper", ena_inactive) < 0) {
        fprintf(stderr, "Failed to request GPIO output lines.\n");
        gpiod_line_release(dir);
        gpiod_line_release(pul);
        gpiod_chip_close(chip);
        return 1;
    }
    ena_requested = 1;

    int rc = 0;
    const long long half_period_ns = (long long)(1000000000.0 / (frequency * 2.0));
    const long long end_ns = monotonic_ns() + (long long)(seconds * 1000000000.0);
    long long next_ns = monotonic_ns();

    if (
        gpiod_line_set_value(ena, ena_active) < 0 ||
        gpiod_line_set_value(dir, dir_state) < 0
    ) {
        fprintf(stderr, "Failed to arm stepper outputs.\n");
        rc = 1;
        goto cleanup;
    }

    while (!g_stop_requested && monotonic_ns() < end_ns) {
        if (gpiod_line_set_value(pul, 1) < 0) {
            rc = 1;
            break;
        }
        next_ns += half_period_ns;
        sleep_until_ns(next_ns);

        if (gpiod_line_set_value(pul, 0) < 0) {
            rc = 1;
            break;
        }
        next_ns += half_period_ns;
        sleep_until_ns(next_ns);
    }

cleanup:
    gpiod_line_set_value(pul, 0);
    gpiod_line_set_value(ena, ena_inactive);
    if (pul_requested) {
        gpiod_line_release(pul);
    }
    if (dir_requested) {
        gpiod_line_release(dir);
    }
    if (ena_requested) {
        gpiod_line_release(ena);
    }
    gpiod_chip_close(chip);
    return rc;
}
