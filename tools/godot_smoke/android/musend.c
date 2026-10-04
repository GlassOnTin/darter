// On-device loopback-UDP send variant matrix (the C3 EACCES hunt).
// Mirrors the SITL's send legs: which sendto variant is kernel-denied?
// Measured 2026-10-04 (OPPO CPH2655): every leg delivers (n=1, recv got
// the packet); the kernel was never the culprit — the real cause was the
// SITL's exported shim dyn-syms interposing inet_aton (physics.md §16).
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

static void try_send(int s, const char *tag, const char *sa, unsigned short port)
{
    struct sockaddr_in d;
    memset(&d, 0, sizeof d);
    d.sin_family = AF_INET;
    d.sin_port = htons(port);
    d.sin_addr.s_addr = inet_addr(sa);
    int n = (int)sendto(s, "X", 1, 0, (struct sockaddr *)&d, sizeof d);
    fprintf(stderr, "[MUSEND] %s s=%d n=%d errno=%d (%s)\n", tag, s, n, errno, strerror(errno));
}

int main(void)
{
    // A: unbound sender -> receiver bound 127.0.0.1:9002 (exact SITL shape)
    int r = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons(9002);
    a.sin_addr.s_addr = inet_addr("127.0.0.1");
    int ok = bind(r, (struct sockaddr *)&a, sizeof a);
    fprintf(stderr, "[MUSEND] bind 127.0.0.1:9002 r=%d errno=%d\n", ok, errno);

    int s = socket(AF_INET, SOCK_DGRAM, 0);
    try_send(s, "A unbound->127.0.0.1:9002", "127.0.0.1", 9002);

    // B: sender bound to 127.0.0.1:0 (kernel picks port)
    int s2 = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in b;
    memset(&b, 0, sizeof b);
    b.sin_family = AF_INET;
    b.sin_port = 0;
    b.sin_addr.s_addr = inet_addr("127.0.0.1");
    ok = bind(s2, (struct sockaddr *)&b, sizeof b);
    fprintf(stderr, "[MUSEND] bind 127.0.0.1:0 r=%d errno=%d\n", ok, errno);
    try_send(s2, "B bound->127.0.0.1:9002", "127.0.0.1", 9002);

    // C/D: 0.0.0.0-bound receiver on 9102, both sender kinds
    int r2 = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in c;
    memset(&c, 0, sizeof c);
    c.sin_family = AF_INET;
    c.sin_port = htons(9102);
    c.sin_addr.s_addr = htonl(INADDR_ANY);
    ok = bind(r2, (struct sockaddr *)&c, sizeof c);
    fprintf(stderr, "[MUSEND] bind 0.0.0.0:9102 r=%d errno=%d\n", ok, errno);
    try_send(s, "C unbound->0.0.0.0:9102", "127.0.0.1", 9102);
    try_send(s2, "D bound->0.0.0.0:9102", "127.0.0.1", 9102);

    // E: unbound sender -> unbound port 9103 (no receiver)
    try_send(s, "E unbound->127.0.0.1:9103", "127.0.0.1", 9103);

    // F: connected UDP socket
    int s3 = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in d;
    memset(&d, 0, sizeof d);
    d.sin_family = AF_INET;
    d.sin_port = htons(9002);
    d.sin_addr.s_addr = inet_addr("127.0.0.1");
    ok = connect(s3, (struct sockaddr *)&d, sizeof d);
    fprintf(stderr, "[MUSEND] connect r=%d errno=%d\n", ok, errno);
    int n = (int)send(s3, "Y", 1, 0);
    fprintf(stderr, "[MUSEND] F connected send n=%d errno=%d (%s)\n", n, errno, strerror(errno));

    // Deliverability: did anything actually arrive on 9002?
    struct timeval tv = {0, 300000};
    setsockopt(r, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    char buf[8];
    int rn = (int)recv(r, buf, 8, 0);
    fprintf(stderr, "[MUSEND] recv 9002 r=%d errno=%d\n", rn, errno);

    // G: plain socket() creation sanity
    int s4 = socket(AF_INET, SOCK_DGRAM, 0);
    fprintf(stderr, "[MUSEND] G socket() s4=%d errno=%d\n", s4, errno);

    return rn > 0 ? 0 : 1;
}