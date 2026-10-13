#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <poll.h>

// 配置
#define VIDEO_DEV       "/dev/video0"
#define TARGET_IP       "192.168.1.250"
#define TARGET_PORT     9000
#define WIDTH           640
#define HEIGHT          480
#define PIXFORMAT       V4L2_PIX_FMT_MJPEG

#define FRAME_HEADER    0xAA55
typedef struct {
    uint16_t header;
    uint32_t length;
} FrameProto;

static int fd_video = -1;
static int udp_sock = -1;
static struct sockaddr_in target_addr;
static void *buffer_start = NULL;
static unsigned int buffer_length = 0;
static volatile int g_exit = 0;

static void sigint_handler(int sig)
{
    (void)sig;
    g_exit = 1;
    printf("\nReceive SIGINT, prepare exit...\n");
}

static int udp_init(const char *ip, uint16_t port)
{
    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) {
        perror("socket");
        return -1;
    }
    memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &target_addr.sin_addr) <= 0) {
        perror("inet_pton");
        close(udp_sock);
        udp_sock = -1;
        return -1;
    }
    return 0;
}

static int v4l2_init(const char *dev)
{
    fd_video = open(dev, O_RDWR);
    if (fd_video < 0) {
        perror("open video");
        return -1;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = PIXFORMAT;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(fd_video, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT");
        close(fd_video);
        fd_video = -1;
        return -1;
    }

    struct v4l2_requestbuffers req = {0};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd_video, VIDIOC_REQBUFS, &req) < 0) {
        perror("REQBUFS");
        close(fd_video);
        fd_video = -1;
        return -1;
    }

    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if (ioctl(fd_video, VIDIOC_QUERYBUF, &buf) < 0) {
        perror("QUERYBUF");
        close(fd_video);
        fd_video = -1;
        return -1;
    }
    buffer_length = buf.length;
    buffer_start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_video, buf.m.offset);
    if (buffer_start == MAP_FAILED) {
        perror("mmap");
        close(fd_video);
        fd_video = -1;
        return -1;
    }

    if (ioctl(fd_video, VIDIOC_QBUF, &buf) < 0) {
        perror("QBUF");
        munmap(buffer_start, buffer_length);
        buffer_start = NULL;
        close(fd_video);
        fd_video = -1;
        return -1;
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_video, VIDIOC_STREAMON, &type) < 0) {
        perror("STREAMON");
        munmap(buffer_start, buffer_length);
        buffer_start = NULL;
        close(fd_video);
        fd_video = -1;
        return -1;
    }
    printf("Camera start %dx%d MJPEG\n", WIDTH, HEIGHT);
    return 0;
}

static int capture_and_send(void)
{
    struct pollfd pfd = {
        .fd = fd_video,
        .events = POLLIN
    };
    // poll超时200ms，定期检查g_exit
    int ret = poll(&pfd, 1, 200);
    if (ret < 0) {
        if (errno == EINTR)
            return 0;
        perror("poll");
        return -1;
    }
    if (ret == 0) {
        // 超时，无帧到达
        return 0;
    }

    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd_video, VIDIOC_DQBUF, &buf) < 0) {
        perror("DQBUF");
        return -1;
    }

    FrameProto proto;
    proto.header = FRAME_HEADER;
    proto.length = buf.bytesused;

    struct iovec iov[2];
    iov[0].iov_base = &proto;
    iov[0].iov_len  = sizeof(proto);
    iov[1].iov_base = buffer_start;
    iov[1].iov_len  = buf.bytesused;

    struct msghdr msg = {
        .msg_name = &target_addr,
        .msg_namelen = sizeof(target_addr),
        .msg_iov = iov,
        .msg_iovlen = 2
    };

    sendmsg(udp_sock, &msg, 0);
    printf("Send frame len: %u\n", buf.bytesused);

    if (ioctl(fd_video, VIDIOC_QBUF, &buf) < 0) {
        perror("QBUF");
        return -1;
    }
    return 0;
}

static void clean_up(void)
{
    if (fd_video >= 0) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_video, VIDIOC_STREAMOFF, &type);
    }
    if (buffer_start != NULL) {
        munmap(buffer_start, buffer_length);
        buffer_start = NULL;
    }
    if (fd_video >= 0) {
        close(fd_video);
        fd_video = -1;
    }
    if (udp_sock >= 0) {
        close(udp_sock);
        udp_sock = -1;
    }
    printf("Cleanup done, camera device released\n");
}

int main(void)
{
    signal(SIGINT, sigint_handler);

    if (udp_init(TARGET_IP, TARGET_PORT) < 0)
        return -1;
    if (v4l2_init(VIDEO_DEV) < 0) {
        clean_up();
        return -1;
    }

    printf("Streaming to %s:%d\n", TARGET_IP, TARGET_PORT);
    while (!g_exit) {
        capture_and_send();
        usleep(50000);
    }

    clean_up();
    return 0;
}