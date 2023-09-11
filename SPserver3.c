#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h> // Eklenen kütüphane

struct termios tty;
int serial_port;
char read_buffer[256];
pthread_mutex_t mutex_read_buffer; // Veri okuma işlemi için mutex

void *readThread(void *arg) {
    while (1) {
        sleep(1); // Her saniye kontrol et

        pthread_mutex_lock(&mutex_read_buffer);
        int num_bytes = read(serial_port, read_buffer, sizeof(read_buffer));
        pthread_mutex_unlock(&mutex_read_buffer);

        if (num_bytes > 0) {
            printf("Received: %s", read_buffer);
            memset(read_buffer, '\0', sizeof(read_buffer));
        }
    }
    return NULL;
}

void *writeThread(void *arg) {
    while (1) {
        char input[256];
        fgets(input, sizeof(input), stdin);

        pthread_mutex_lock(&mutex_read_buffer);
        write(serial_port, input, strlen(input));
        pthread_mutex_unlock(&mutex_read_buffer);

        if (strcmp(input, "bye\n") == 0) {
            printf("Closing serial port.\n");
            close(serial_port);
            break;
        }
    }
    return NULL;
}

int main() {
    serial_port = open("/dev/ttyS0", O_RDWR);
    tcgetattr(serial_port, &tty);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    cfsetspeed(&tty, B9600);
    tcsetattr(serial_port, TCSANOW, &tty);

    pthread_t read_tid, write_tid;
    pthread_mutex_init(&mutex_read_buffer, NULL);

    pthread_create(&read_tid, NULL, readThread, NULL);
    pthread_create(&write_tid, NULL, writeThread, NULL);

    pthread_join(read_tid, NULL);
    pthread_join(write_tid, NULL);

    pthread_mutex_destroy(&mutex_read_buffer);

    return 0;
}
