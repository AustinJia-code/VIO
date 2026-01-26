/**
 * @file streamer.hpp
 * @brief Small tool for sending data over UDP
 */
#include <arpa/inet.h>
#include <sys/socket.h>
 
/**
 * Telemetry packet for VIO estimation
 */
struct TelemetryPacket
{
    double ekf_x, ekf_y, ekf_z;
    double gt_x, gt_y, gt_z;
};

class Streamer
{
private:
    int sock;
    struct sockaddr_in servaddr;

public:
    Streamer ()
    {
        sock = socket (AF_INET, SOCK_DGRAM, 0);
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons (5555);
        servaddr.sin_addr.s_addr = inet_addr ("127.0.0.1");
    }

    /**
     * Send a packet
     */
    void send (const TelemetryPacket& p)
    {
        sendto (sock, &p, sizeof (p), 0, (struct sockaddr*) &servaddr,
                                          sizeof (servaddr));
    }
};