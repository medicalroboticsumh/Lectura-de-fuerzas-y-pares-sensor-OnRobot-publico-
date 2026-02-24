#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <iostream>
#include <ctime>
#include <chrono>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#pragma comment(lib, "Ws2_32.lib")
typedef SOCKET SOCKET_HANDLE;
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
typedef int SOCKET_HANDLE;
#endif

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define PORT 49152
#define SPEED 50 // 50 ms por muestra = 20 Hz
#define FILTER 0 // 0 = sin filtro
#define BIASING_ON 0xFF
#define COMMAND_START 0x0002
#define COMMAND_STOP 0x0000
#define COMMAND_BIAS 0x0042
#define COMMAND_FILTER 0x0081
#define COMMAND_SPEED 0x0082

#define UNIT 1 // 1 = Newton y Newton-metro

#if UNIT == 1
#define FORCE_DIV 10000.0
#define TORQUE_DIV 100000.0
#else
#define FORCE_DIV 1.0
#define TORQUE_DIV 1.0
#endif

typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;

typedef struct ResponseStruct
{
    unsigned int sequenceNumber;
    unsigned int sampleCounter;
    unsigned int status;
    int32 fx;
    int32 fy;
    int32 fz;
    int32 tx;
    int32 ty;
    int32 tz;
} Response;

static void MySleep(unsigned long ms)
{
#ifdef _WIN32
    Sleep(ms);
#else
    usleep(ms * 1000);
#endif
}

static int Connect(SOCKET_HANDLE *handle, const char *ipAddress, uint16 port)
{
    struct sockaddr_in addr;
    struct hostent *he;
#ifdef _WIN32
    WSADATA wsaData;
    WORD wVersionRequested = MAKEWORD(2, 2);
    WSAStartup(wVersionRequested, &wsaData);
#endif

    *handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (*handle < 0)
    {
        fprintf(stderr, "No se pudo abrir el socket\n");
        return -1;
    }

    he = gethostbyname(ipAddress);
    memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    int err = connect(*handle, (struct sockaddr *)&addr, sizeof(addr));
    if (err < 0)
    {
        fprintf(stderr, "No se pudo conectar con el sensor\n");
        return -2;
    }

    return 0;
}

static void Close(SOCKET_HANDLE *handle)
{
#ifdef _WIN32
    closesocket(*handle);
    WSACleanup();
#else
    close(*handle);
#endif
}

static void SendCommand(SOCKET_HANDLE *socket, uint16 command, uint32 data)
{
    byte request[8];
    *(uint16 *)&request[0] = htons(0x1234);
    *(uint16 *)&request[2] = htons(command);
    *(uint32 *)&request[4] = htonl(data);
    send(*socket, (const char *)request, 8, 0);
    MySleep(5);
    std::cout << "[DEBUG] Comando enviado: 0x" << std::hex << command << std::dec << std::endl;
}

static bool Receive(SOCKET_HANDLE *socket, Response &response)
{
    byte inBuffer[36];
    int bytes = recv(*socket, (char *)inBuffer, 36, 0);
    if (bytes <= 0)
    {
        // No llegaron datos
        return false;
    }

    unsigned int i = 0;
    response.sequenceNumber = ntohl(*(uint32 *)&inBuffer[0]);
    response.sampleCounter = ntohl(*(uint32 *)&inBuffer[4]);
    response.status = ntohl(*(uint32 *)&inBuffer[8]);
    response.fx = ntohl(*(int32 *)&inBuffer[12 + (i++) * 4]);
    response.fy = ntohl(*(int32 *)&inBuffer[12 + (i++) * 4]);
    response.fz = ntohl(*(int32 *)&inBuffer[12 + (i++) * 4]);
    response.tx = ntohl(*(int32 *)&inBuffer[12 + (i++) * 4]);
    response.ty = ntohl(*(int32 *)&inBuffer[12 + (i++) * 4]);
    response.tz = ntohl(*(int32 *)&inBuffer[12 + (i++) * 4]);

    return true;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Uso: rosrun <package> <nodo> <IP_SENSOR>\n";
        return 1;
    }

    const char *ip = argv[1];
    SOCKET_HANDLE socketHandle;

    std::cout << "[INFO] Intentando conectar con el sensor en " << ip << ":" << PORT << " ..." << std::endl;
    if (Connect(&socketHandle, ip, PORT) != 0)
    {
        std::cerr << "[ERROR] Fallo al conectar con el sensor" << std::endl;
        return 1;
    }
    std::cout << "[INFO] Conexión al sensor establecida." << std::endl;

    // Inicia ROS
    ros::init(argc, argv, "force_torque_publisher");
    ros::NodeHandle nh;
    ros::Publisher wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("force_torque_sensor", 10);
    ros::Rate loop_rate(1000.0 / SPEED);

    // Configura el sensor
    SendCommand(&socketHandle, COMMAND_SPEED, SPEED);
    SendCommand(&socketHandle, COMMAND_FILTER, FILTER);
    SendCommand(&socketHandle, COMMAND_START, 0);
    //SendCommand(&socketHandle, COMMAND_BIAS, BIASING_ON);

    std::cout << "[INFO] Iniciando bucle de recepción de paquetes..." << std::endl;

    // Bucle continuo
    while (ros::ok())
    {
        Response r;
        bool received = Receive(&socketHandle, r);

        if (received)
        {
            geometry_msgs::WrenchStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.wrench.force.x = r.fx / FORCE_DIV;
            msg.wrench.force.y = r.fy / FORCE_DIV;
            msg.wrench.force.z = r.fz / FORCE_DIV;
            msg.wrench.torque.x = r.tx / TORQUE_DIV;
            msg.wrench.torque.y = r.ty / TORQUE_DIV;
            msg.wrench.torque.z = r.tz / TORQUE_DIV;

            ROS_INFO_STREAM("Publicando mensaje: "
                            << "fx=" << msg.wrench.force.x << ", "
                            << "fy=" << msg.wrench.force.y << ", "
                            << "fz=" << msg.wrench.force.z << ", "
                            << "tx=" << msg.wrench.torque.x << ", "
                            << "ty=" << msg.wrench.torque.y << ", "
                            << "tz=" << msg.wrench.torque.z);

            wrench_pub.publish(msg);
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "No se reciben datos del sensor...");
            continue; // <-- SOLUCIÓN
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    SendCommand(&socketHandle, COMMAND_STOP, 0);
    Close(&socketHandle);
    std::cout << "[INFO] Conexión cerrada, nodo terminado." << std::endl;
    return 0;
}
