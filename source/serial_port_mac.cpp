/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
 *  Corporation. All rights reserved. This software, including source code, documentation and  related 
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
 * (United States and foreign), United States copyright laws and international treaty provisions. 
 * Therefore, you may use this Software only as provided in the license agreement accompanying the 
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit 
 * products. Any reproduction, modification, translation, compilation,  or representation of this 
 * Software except as specified above is prohibited without the express written permission of 
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
 * the Software without notice. Cypress does not assume any liability arising out of the application 
 * or use of the Software or any product or circuit  described in the Software. Cypress does 
 * not authorize its products for use in any products where a malfunction or failure of the 
 * Cypress product may reasonably be expected to result  in significant property damage, injury 
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
 * to indemnify Cypress against all liability.
 */

/*
 * Sample MCU application for implemeting serial port read/write on Mac OS.
 */


// The buildt in QT serial port class cannot be used on Mac OS various bugs
// and buad rate limitations.
#include <QObject>
#include <QDebug>
#include "mainwindow.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <ctype.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>
#include "mainwindow.h"
#include "hci_control_api.h"
#include <IOKit/serial/ioss.h>
#include <sys/ttycom.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <unistd.h>
#include <poll.h>
#include <unistd.h>
#include "wiced_types.h"

extern void HandleWicedEvent(unsigned int opcode, unsigned char*p_data, unsigned int len);
static int Read(unsigned char * pReadBuf, int nSz);
static void * read_thread(void *arg);
static int ReadNewHciPacket(unsigned char * pu8Buffer, int bufLen, int * pOffset);

#ifndef USERIAL_READ_THREAD_POLICY
#define USERIAL_READ_THREAD_POLICY SCHED_FIFO
#endif

static int fd = -1;
static bool m_bStop = false;
pthread_t      worker_thread1;

void HandleWicedEvent(unsigned int opcode, unsigned char*p_data, unsigned int len)
{
    emit g_pMainWindow->HandleWicedEvent(opcode, len, p_data);
}

static unsigned int min(unsigned int d1, unsigned int d2)
{
    if (d1 < d2)
        return d1;
    return (d2);
}

WicedSerialPort::WicedSerialPort(bool host_mode)
{
    UNUSED(host_mode);
    fd = -1;
    m_bStop = false;
}

WicedSerialPort::~WicedSerialPort()
{

}

qint64 WicedSerialPort::write(const char *data, qint64 len)
{
    if (fd == -1)
        return 0;
    g_pMainWindow->Log("WicedSerialPort::write(),len=%d",(int)len);
    int retry = 0;
    int total = 0;
    int ret=0;

    while(len != 0)
    {
        ret = ::write(fd, data + total, len);
        if (ret == -1)
        {
            g_pMainWindow->Log("write error: %d",errno);
            if (errno == 35/*EDEADLK*/)
            {
                QThread::msleep(100);
                if (++retry == 3)
                    break;
                continue;

            }
            break;
        }
        total += ret;
        len -= ret;
    }
    tcflush(fd,TCIOFLUSH);
    g_pMainWindow->Log("Write returns %d",total);
    return (total);
}


bool WicedSerialPort::open(const char *str_port_name, qint32 baudRate, bool bFlowControl)
{    
    UNUSED(bFlowControl);
    pthread_attr_t thread_attr;

    g_pMainWindow->Log("WicedSerialPort::open() dev: %s baud: %d", str_port_name, baudRate );

    if ((fd = ::open(str_port_name, O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1)
    {
        g_pMainWindow->Log("%s unable to open %s",  __FUNCTION__, str_port_name);
        return false;
    }

    struct termios      options;
    m_bStop = false;

    // clear the O_NONBLOCK flag, so that read() will
    //   block and wait for data.
    if (-1 == fcntl(fd, F_SETFL, O_APPEND | O_NONBLOCK))
    {
        g_pMainWindow->Log("fcntl() failed: %d",errno);
    }

    // grab the options for the serial port
    tcgetattr(fd, &options);

    options.c_cflag |= CREAD | CLOCAL | CS8;
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VMIN]=0;
    options.c_cc[VTIME]=0;
    options.c_cflag &= ~CRTSCTS;    

    tcsetattr(fd, TCSAFLUSH, &options);

    // the Mac iOS does not allow baud rates higher then 115200 so to
    // set the baud rate send a command directly to the USB driver
    speed_t speed = baudRate;
    if (ioctl(fd, IOSSIOSPEED, &speed) == -1)
    {
        g_pMainWindow->Log("WicedSerialPort::open() : Error %d calling ioctl(..., IOSSIOSPEED, ...).", errno);
    }

    pthread_attr_init(&thread_attr);
    if (pthread_create( &worker_thread1,
                          &thread_attr,
                          (void *(*) (void *)) read_thread,
                          (void *) this) != 0)
    {
        g_pMainWindow->Log( "pthread_create failed!");
        return false;
    }

    return true;
}

void WicedSerialPort::close()
{
    if (fd == -1)
        return;
    g_pMainWindow->Log( "Close(): closing transport (%d)\n", fd);
    m_bStop = true;
    int result = pthread_join( worker_thread1, NULL );
    if ( result < 0 )
    {
        g_pMainWindow->Log("pthread_join() FAILED: result: %d", result );
    }
    ::close(fd);
    fd = -1;

    g_pMainWindow->Log( "Close(): Exit\n");
}

bool WicedSerialPort::isOpen() const
{
    return (fd != -1);
}

int Read(unsigned char * pReadBuf, int nSz)
{
    pollfd pfds;
    pfds.fd = fd;
    pfds.events = POLLIN;

    int res = poll(&pfds, 1, 1000);

     if (res == 0)
     {
         return 0;
     }
     else if (m_bStop || (res < 0 && (errno == EINTR || errno == ENOMEM)))
     {
         return -1;
     }
     else if (!( pfds.revents & POLLIN ))
     {
         return 0;
     }

    int r = ::read(fd, (void *)pReadBuf, (size_t) nSz);
    usleep(100);
    return r;

}

qint64 WicedSerialPort::read(char *data, qint64 maxlen)
{
    UNUSED(data);
    UNUSED(maxlen);
    return 0;
}

int ReadNewHciPacket(unsigned char * pu8Buffer, int bufLen, int * pOffset)
{
    int dwLen, len = 0, offset = 0;

    dwLen = Read(&pu8Buffer[offset], 1);

    if (dwLen == 0 || m_bStop)
        return (0);

    if (dwLen < 0)
        return -1;

    offset++;

    switch (pu8Buffer[0])
    {
    case HCI_EVENT_PKT:
    {
        dwLen = Read(&pu8Buffer[offset], 2);
        if(dwLen == 2)
        {
            len = pu8Buffer[2];
            offset += 2;
            g_pMainWindow->Log("HCI_EVENT_PKT len %d", len);
        }
        else
            g_pMainWindow->Log("error HCI_EVENT_PKT, needed 2 got %d", dwLen);
    }
        break;

    case HCI_ACL_DATA_PKT:
    {
        dwLen = Read(&pu8Buffer[offset], 4);
        if(dwLen == 4)
        {
            len = pu8Buffer[3] | (pu8Buffer[4] << 8);
            offset += 4;
            g_pMainWindow->Log("HCI_ACL_DATA_PKT, len %d", len);
        }
        else
            g_pMainWindow->Log("error HCI_ACL_DATA_PKT needed 4 got %d", dwLen);
    }
        break;

    case HCI_WICED_PKT:
    {
        dwLen = Read(&pu8Buffer[offset], 4);
        if(dwLen == 4)
        {
            len = pu8Buffer[3] | (pu8Buffer[4] << 8);
            offset += 4;
        }
        else
            g_pMainWindow->Log("error HCI_WICED_PKT,  needed 4 got %d", dwLen);
    }
        break;
    default:
    {
            g_pMainWindow->Log("error unknown packet, type %d", pu8Buffer[0]);
    }
    }

    if(len > 1024)
    {
        g_pMainWindow->Log("bad packet %d", len);
        return -1; // bad packet
    }

    if (len)
    {
        DWORD lenRd = ::min(len, (DWORD)(bufLen-offset));
        dwLen = Read(&pu8Buffer[offset], lenRd);
        if(dwLen != lenRd)
            g_pMainWindow->Log("read error to read %d, read %d", lenRd, dwLen);
    }

    *pOffset = offset;

    return len;
}
void WicedSerialPort::indicate_close()
{
}

void WicedSerialPort::flush()
{

}

bool WicedSerialPort::waitForBytesWritten(int iMilisec)
{
    UNUSED(iMilisec);
    return true;
}


void * read_thread(void *arg)
{    
    UNUSED(arg);
    unsigned char au8Hdr[1024 + 6];
    int           offset = 0, pktLen;
    int           packetType;

    g_pMainWindow->Log("userial_read_thread starting");
    while (!m_bStop)
    {
        memset(au8Hdr, 0, 1030);
        offset = 0;
        // Read HCI packet
        pktLen = ReadNewHciPacket(au8Hdr, sizeof(au8Hdr), &offset);
        //g_pMainWindow->Log("userial_read_thread read new pkt, len=%d",pktLen);
        if (m_bStop)
            break;

        if(pktLen < 0) // skip this
            continue;

        if (pktLen + offset == 0)
            continue;

        packetType = au8Hdr[0];
        if (HCI_WICED_PKT == packetType)
        {
            DWORD channel_id = au8Hdr[1] | (au8Hdr[2] << 8);
            DWORD len = au8Hdr[3] | (au8Hdr[4] << 8);

            if(len < 0)
                continue;

            if (len > 1024)
            {
                qDebug("Skip bad packet %d", (int) len);
                continue;
            }
            BYTE * pBuf = NULL;

            if(len)
            {
                // malloc and create a copy of the data.
                //  MainWindow::onHandleWicedEvent deletes the data
                pBuf = (BYTE*)malloc(len);
                memcpy(pBuf, &au8Hdr[5], len);
            }

            // send it to main thread
            //emit m_pParent->HandleWicedEvent(channel_id, len, pBuf);
            HandleWicedEvent(channel_id, pBuf, len);
        }
    }

    g_pMainWindow->Log("userial_read_thread exiting");
}

int WicedSerialPort::errorNum()
{
    return errno;
}

void WicedSerialPort::handleReadyRead()
{

}

int openSerialPort(QSerialPort & serial)
{
    int baud = serial.baudRate();
    bool rval = false;

    if (baud > 115200)
    {
        serial.setBaudRate(115200);
    }

    if (rval = serial.open(QIODevice::ReadWrite))
    {
        // port open, need to set baud
        if (baud > 115200)
        {
            speed_t speed = baud;
            if (ioctl(serial.handle(), IOSSIOSPEED, &speed) == -1)
            {
                g_pMainWindow->Log("openSerialPort() : Error %d calling ioctl(..., IOSSIOSPEED, ...).", errno);
                rval = false;
                serial.close();
            }
        }
    }
    return rval;
}


WicedSerialPortHostmode::WicedSerialPortHostmode (){}
qint64 WicedSerialPortHostmode::read(char *data, qint64 maxlen) { return 0; }
qint64 WicedSerialPortHostmode::write(const char *data, qint64 len) { return 0; }
bool WicedSerialPortHostmode::open(const char *str_port_name, qint32 baudRate, bool bFlowControl) { return 0; }
void WicedSerialPortHostmode::close() {}
bool WicedSerialPortHostmode::isOpen() const { return 0; }
void WicedSerialPortHostmode::flush() {}
bool WicedSerialPortHostmode::waitForBytesWritten(int iMilisec) { return 0; }
int WicedSerialPortHostmode::errorNum() { return 0; }
void WicedSerialPortHostmode::handleReadyRead() {}
bool WicedSerialPortHostmode::OpenSocket() { return 0; }
