/*
 ============================================================================
 Name        : clevo-fan-control.c
 Author      : Laurens Bruins <l.bruins@laptopwithlinux.com>
 Version     : 01
 Description : Ubuntu fan control driver for Clevo laptops

 Based on https://github.com/SkyLandTW/clevo-indicator by AqD 
 (iiiaqd@gmail.com)

 ============================================================================

 Auto fan control algorithm:

 The algorithm is to replace the builtin auto fan-control algorithm in Clevo
 laptops. It's purpose is keeping the fan volume at a reasonable level when 
 operating the laptop at normal levels.
 When the laptop is under heavy load the fans will still produce a fair amount
 of noise to keep the temperatures down.
 This clevo-indicator based program doesn't run the UI element that
 clevo-indicator supplied. However monitoring the state of the driver can be
 done by setting "Terminal=" to "true" in the .desktop file

 ============================================================================
 */

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/io.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#define NAME "clevo-fan-control"

#define EC_SC 0x66
#define EC_DATA 0x62

#define IBF 1
#define OBF 0
#define EC_SC_READ_CMD 0x80

/* EC registers can be read by EC_SC_READ_CMD or /sys/kernel/debug/ec/ec0/io:
 *
 * 1. modprobe ec_sys
 * 2. od -Ax -t x1 /sys/kernel/debug/ec/ec0/io
 */

#define EC_REG_SIZE 0x100
#define EC_REG_CPU_TEMP 0x07
#define EC_REG_GPU_TEMP 0xCD
#define EC_REG_FAN_DUTY_CPU 0xCE
#define EC_REG_FAN_DUTY_GPU 0xCF
#define EC_REG_FAN_RPMS_HI 0xD0
#define EC_REG_FAN_RPMS_LO 0xD1

#define MAX_FAN_RPM 4400.0
#define MAX(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

typedef enum {
    NA = 0, AUTO = 1, MANUAL = 2
} MenuItemType;

static void main_init_share(void);
static int main_ec_worker(void);
static void main_on_sigchld(int signum);
static void main_on_sigterm(int signum);
static int main_dump_fan(void);
static int main_test_fan(int duty_percentage);
static void ec_on_sigterm(int signum);
static int ec_init(void);
static int ec_auto_duty_adjust(void);
static int ec_query_cpu_temp(void);
static int ec_query_gpu_temp(void);
static int ec_query_fan_duty(void);
static int ec_query_fan_rpms(void);
static int ec_write_cpu_fan_duty(int duty_percentage);
static int ec_write_gpu_fan_duty(int duty_percentage);
static int ec_io_wait(const uint32_t port, const uint32_t flag,
        const char value);
static uint8_t ec_io_read(const uint32_t port);
static int ec_io_do(const uint32_t cmd, const uint32_t port,
        const uint8_t value);
static int calculate_fan_duty(int raw_duty);
static int calculate_fan_rpms(int raw_rpm_high, int raw_rpm_low);
static int check_proc_instances(const char* proc_name);
static void signal_term(__sighandler_t handler);

struct {
    volatile int exit;
    volatile int cpu_temp;
    volatile int gpu_temp;
    volatile int fan_duty;
    volatile int fan_rpms;
    volatile int auto_duty;
    volatile int auto_duty_val;
    volatile int manual_next_fan_duty;
    volatile int manual_prev_fan_duty;
}static *share_info = NULL;

static pid_t parent_pid = 0;

int main(int argc, char* argv[]) {
    printf("Simple fan control utility for Clevo laptops\n");
    if (check_proc_instances(NAME) > 1) {
        printf("Multiple running instances!\n");
        char* display = getenv("DISPLAY");
        if (display != NULL && strlen(display) > 0) {
            int desktop_uid = getuid();
            setuid(desktop_uid);
            //
        }
        return EXIT_FAILURE;
    }
    if (ec_init() != EXIT_SUCCESS) {
        printf("unable to control EC: %s\n", strerror(errno));
        return EXIT_FAILURE;
    }
    if (argc <= 1) {
        char* display = getenv("DISPLAY");
        if (display == NULL || strlen(display) == 0) {
            return main_dump_fan();
        } else {
            parent_pid = getpid();
            main_init_share();
            signal(SIGCHLD, &main_on_sigchld);
            signal_term(&main_on_sigterm);
            pid_t worker_pid = 0;
            if (worker_pid == 0) {
                signal(SIGCHLD, SIG_DFL);
                signal_term(&ec_on_sigterm);
                return main_ec_worker();
            } else if (worker_pid > 0) {
                share_info->exit = 1;
                waitpid(worker_pid, NULL, 0);
            } else {
                printf("unable to create worker: %s\n", strerror(errno));
                return EXIT_FAILURE;
            }
        }
    } else {
        if (argv[1][0] == '-') {
            printf(
                    "\n\
Usage: clevo-indicator [fan-duty-percentage]\n\
\n\
Dump/Control fan duty on Clevo laptops. Display indicator by default.\n\
\n\
Arguments:\n\
  [fan-duty-percentage]\t\tTarget fan duty in percentage, from 40 to 100\n\
  -?\t\t\t\tDisplay this help and exit\n\
\n\
Note any fan duty change should take 1-2 seconds to come into effect - you\n\
can verify by the fan speed displayed on indicator icon and also louder fan\n\
noise.\n\
\n\
This program would always attempt to load kernel\n\
module 'ec_sys', in order to query EC information from\n\
'/sys/kernel/debug/ec/ec0/io' instead of polling EC ports for readings,\n\
which may be more risky if interrupted or concurrently operated during the\n\
process.\n\
\n\
DO NOT MANIPULATE OR QUERY EC I/O PORTS WHILE THIS PROGRAM IS RUNNING.\n\
\n");
            return main_dump_fan();
        } else {
            int val = atoi(argv[1]);
            if (val < 20 || val > 100)
                    {
                printf("invalid fan duty %d!\n", val);
                return EXIT_FAILURE;
            }
            return main_test_fan(val);
        }
    }
    return EXIT_SUCCESS;
}

static void main_init_share(void) {
    void* shm = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_ANON | MAP_SHARED,
            -1, 0);
    share_info = shm;
    share_info->exit = 0;
    share_info->cpu_temp = 0;
    share_info->gpu_temp = 0;
    share_info->fan_duty = 0;
    share_info->fan_rpms = 0;
    share_info->auto_duty = 1;
    share_info->auto_duty_val = 0;
    share_info->manual_next_fan_duty = 0;
    share_info->manual_prev_fan_duty = 0;
}

static int main_ec_worker(void) {
    setuid(0);
    system("modprobe ec_sys");
    while (share_info->exit == 0) {
        // check parent
        if (parent_pid != 0 && kill(parent_pid, 0) == -1) {
            printf("worker on parent death\n");
            break;
        }
        // write EC
        int new_fan_duty = share_info->manual_next_fan_duty;
        if (new_fan_duty != 0
                && new_fan_duty != share_info->manual_prev_fan_duty) {
            ec_write_cpu_fan_duty(new_fan_duty);
            ec_write_gpu_fan_duty(new_fan_duty);
            share_info->manual_prev_fan_duty = new_fan_duty;
        }
        // read EC
        int io_fd = open("/sys/kernel/debug/ec/ec0/io", O_RDONLY, 0);
        if (io_fd < 0) {
            printf("unable to read EC from sysfs: %s\n", strerror(errno));
            exit(EXIT_FAILURE);
        }
        unsigned char buf[EC_REG_SIZE];
        ssize_t len = read(io_fd, buf, EC_REG_SIZE);
        switch (len) {
        case -1:
            printf("unable to read EC from sysfs: %s\n", strerror(errno));
            break;
        case 0x100:
            share_info->cpu_temp = buf[EC_REG_CPU_TEMP];
            share_info->gpu_temp = buf[EC_REG_GPU_TEMP];
            share_info->fan_duty = calculate_fan_duty(buf[EC_REG_FAN_DUTY_CPU]);
            share_info->fan_rpms = calculate_fan_rpms(buf[EC_REG_FAN_RPMS_HI],
                    buf[EC_REG_FAN_RPMS_LO]);
            /*
             printf("temp=%d, duty=%d, rpms=%d\n", share_info->cpu_temp,
             share_info->fan_duty, share_info->fan_rpms);
             */
            break;
        default:
            printf("wrong EC size from sysfs: %ld\n", len);
        }
        close(io_fd);
        // auto EC
        if (share_info->auto_duty == 1) {
            int next_duty = ec_auto_duty_adjust();
            int diff;
            if (next_duty > share_info->auto_duty_val) {
            	diff=next_duty-share_info->auto_duty_val;
            	if (diff > 3)
            	    next_duty=share_info->auto_duty_val+2;
            	if (next_duty < 20)
            	    next_duty=20;
            }
            if (next_duty < share_info->auto_duty_val) {
            	diff=share_info->auto_duty_val-next_duty;
            	if (diff > 1)
            	    next_duty=share_info->auto_duty_val-1;
            	if (next_duty < 20)
            	    next_duty=20;
            }
            if (next_duty != 0 && next_duty != share_info->auto_duty_val) {
                char s_time[256];
                printf("%s CPU=%d°C, GPU=%d°C, auto fan duty to %d%%\n", s_time,
                        share_info->cpu_temp, share_info->gpu_temp, next_duty);
                ec_write_cpu_fan_duty(next_duty);
                ec_write_gpu_fan_duty(next_duty);
                share_info->auto_duty_val = next_duty;
            }
        }
        //
        usleep(200 * 1000);
    }
    printf("worker quit\n");
    return EXIT_SUCCESS;
}

static void main_on_sigchld(int signum) {
    printf("main on worker quit signal\n");
    exit(EXIT_SUCCESS);
}

static void main_on_sigterm(int signum) {
    printf("main on signal: %s\n", strsignal(signum));
    if (share_info != NULL)
        share_info->exit = 1;
    exit(EXIT_SUCCESS);
}

static int main_dump_fan(void) {
    printf("Dump fan information\n");
    printf("  FAN Duty: %d%%\n", ec_query_fan_duty());
    printf("  FAN RPMs: %d RPM\n", ec_query_fan_rpms());
    printf("  CPU Temp: %d°C\n", ec_query_cpu_temp());
    printf("  GPU Temp: %d°C\n", ec_query_gpu_temp());
    return EXIT_SUCCESS;
}

static int main_test_fan(int duty_percentage) {
    printf("Change fan duty to %d%%\n", duty_percentage);
    ec_write_cpu_fan_duty(duty_percentage);
    ec_write_gpu_fan_duty(duty_percentage);
    printf("\n");
    main_dump_fan();
    return EXIT_SUCCESS;
}

static int ec_init(void) {
    if (ioperm(EC_DATA, 1, 1) != 0)
        return EXIT_FAILURE;
    if (ioperm(EC_SC, 1, 1) != 0)
        return EXIT_FAILURE;
    return EXIT_SUCCESS;
}

static void ec_on_sigterm(int signum) {
    printf("ec on signal: %s\n", strsignal(signum));
    if (share_info != NULL)
        share_info->exit = 1;
}

static int ec_auto_duty_adjust(void) {
    int temp = MAX(share_info->cpu_temp, share_info->gpu_temp);
    // Fan Table
    // EC doesn't report above 95 degrees C
    if (temp >= 95)
    	return 100;
    if (temp == 94)
    	return 95;
    if (temp == 93)
    	return 95;
    if (temp == 92)
    	return 93;
    if (temp == 91)
    	return 90;
    if (temp == 90)
    	return 87;
    if (temp == 89)
    	return 85;
    if (temp == 88)
    	return 82;
    if (temp == 87)
    	return 80;
    if (temp == 86)
    	return 75;
    if (temp == 85)
    	return 70;
    if (temp == 84)
    	return 67;
    if (temp == 83)
    	return 65;
    if (temp == 82)
    	return 62;
    if (temp == 81)
    	return 60;
    if (temp == 80)
    	return 57;
    if (temp == 79)
    	return 55;
    if (temp == 78)
    	return 53;
    if (temp == 77)
    	return 52;
    if (temp == 76)
    	return 48;
    if (temp == 75)
    	return 46;
    if (temp == 74)
    	return 44;
    if (temp == 73)
    	return 43;
    if (temp == 72)
    	return 40;
    if (temp == 71)
    	return 37;
    if (temp == 70)
    	return 33;
    if (temp == 69)
    	return 30;
    if (temp == 68)
    	return 28;
    if (temp == 67)
    	return 26;
    if (temp == 66)
    	return 25;
    if (temp == 65)
    	return 24;
    if (temp == 64)
    	return 23;
    if (temp == 63)
    	return 23;
    if (temp == 62)
    	return 22;
    if (temp == 61)
    	return 22;
    if (temp == 60)
    	return 22;
    if (temp == 59)
    	return 22;
    if (temp == 58)
    	return 21;
    if (temp == 57)
    	return 21;
    if (temp <= 56)
    	return 20;
    	
    // catch
    return 20;
}

static int ec_query_cpu_temp(void) {
    return ec_io_read(EC_REG_CPU_TEMP);
}

static int ec_query_gpu_temp(void) {
    return ec_io_read(EC_REG_GPU_TEMP);
}

static int ec_query_fan_duty(void) {
    int raw_duty = ec_io_read(EC_REG_FAN_DUTY_CPU);
    return calculate_fan_duty(raw_duty);
}

static int ec_query_fan_rpms(void) {
    int raw_rpm_hi = ec_io_read(EC_REG_FAN_RPMS_HI);
    int raw_rpm_lo = ec_io_read(EC_REG_FAN_RPMS_LO);
    return calculate_fan_rpms(raw_rpm_hi, raw_rpm_lo);
}

static int ec_write_cpu_fan_duty(int duty_percentage) {
    if (duty_percentage < 20 || duty_percentage > 100) {
        printf("Wrong fan duty to write: %d\n", duty_percentage);
        return EXIT_FAILURE;
    }
    double v_d = ((double) duty_percentage) / 100.0 * 255.0;
    int v_i = (int) v_d;
    return ec_io_do(0x99, 0x01, v_i);
}


static int ec_write_gpu_fan_duty(int duty_percentage) {
    if (duty_percentage < 0 || duty_percentage > 100) {
        printf("Wrong fan duty to write: %d\n", duty_percentage);
        return EXIT_FAILURE;
    }
    double v_d = ((double) duty_percentage) / 100.0 * 255.0 + 0.5;
    int v_i = (int) v_d;
    return ec_io_do(0x99, 0x02, v_i);
}

static int ec_io_wait(const uint32_t port, const uint32_t flag,
        const char value) {
    uint8_t data = inb(port);
    int i = 0;
    while ((((data >> flag) & 0x1) != value) && (i++ < 100)) {
        usleep(1000);
        data = inb(port);
    }
    if (i >= 100) {
        printf("wait_ec error on port 0x%x, data=0x%x, flag=0x%x, value=0x%x\n",
                port, data, flag, value);
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

static uint8_t ec_io_read(const uint32_t port) {
    ec_io_wait(EC_SC, IBF, 0);
    outb(EC_SC_READ_CMD, EC_SC);

    ec_io_wait(EC_SC, IBF, 0);
    outb(port, EC_DATA);

    //wait_ec(EC_SC, EC_SC_0xCEIBF_FREE);
    ec_io_wait(EC_SC, OBF, 1);
    uint8_t value = inb(EC_DATA);

    return value;
}

static int ec_io_do(const uint32_t cmd, const uint32_t port,
        const uint8_t value) {
    ec_io_wait(EC_SC, IBF, 0);
    outb(cmd, EC_SC);

    ec_io_wait(EC_SC, IBF, 0);
    outb(port, EC_DATA);

    ec_io_wait(EC_SC, IBF, 0);
    outb(value, EC_DATA);

    return ec_io_wait(EC_SC, IBF, 0);
}

static int calculate_fan_duty(int raw_duty) {
    return (int) ((double) raw_duty / 255.0 * 100.0);
}

static int calculate_fan_rpms(int raw_rpm_high, int raw_rpm_low) {
    int raw_rpm = (raw_rpm_high << 8) + raw_rpm_low;
    return raw_rpm > 0 ? (2156220 / raw_rpm) : 0;
}

static int check_proc_instances(const char* proc_name) {
    int proc_name_len = strlen(proc_name);
    pid_t this_pid = getpid();
    DIR* dir;
    if (!(dir = opendir("/proc"))) {
        perror("can't open /proc");
        return -1;
    }
    int instance_count = 0;
    struct dirent* ent;
    while ((ent = readdir(dir)) != NULL) {
        char* endptr;
        long lpid = strtol(ent->d_name, &endptr, 10);
        if (*endptr != '\0')
            continue;
        if (lpid == this_pid)
            continue;
        char buf[512];
        snprintf(buf, sizeof(buf), "/proc/%ld/comm", lpid);
        FILE* fp = fopen(buf, "r");
        if (fp) {
            if (fgets(buf, sizeof(buf), fp) != NULL) {
                if ((buf[proc_name_len] == '\n' || buf[proc_name_len] == '\0')
                        && strncmp(buf, proc_name, proc_name_len) == 0) {
                    fprintf(stderr, "Process: %ld\n", lpid);
                    instance_count += 1;
                }
            }
            fclose(fp);
        }
    }
    closedir(dir);
    return instance_count;
}

static void signal_term(__sighandler_t handler) {
    signal(SIGHUP, handler);
    signal(SIGINT, handler);
    signal(SIGQUIT, handler);
    signal(SIGPIPE, handler);
    signal(SIGALRM, handler);
    signal(SIGTERM, handler);
    signal(SIGUSR1, handler);
    signal(SIGUSR2, handler);
}
