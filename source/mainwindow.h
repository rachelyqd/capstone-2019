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
 * Sample MCU application for using WICED HCI protocol. Main app header file.
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <stdio.h>
#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QComboBox>
#include <QSettings>
#include <QWaitCondition>
#include <QSemaphore>
#include <QMessageBox>
#include <QThread>
#include <QMutex>
#include "serial_port.h"
#include <QListWidget>
#include <QAudioRecorder>

#ifdef PCM_ALSA
#include <alsa/asoundlib.h>
#endif

#ifdef Q_OS_WIN
// some earlier versions of the Microsoft compiler do not support snprintf()
// so we supply a custom funtion which is functionaly equivalent
int ms_snprintf ( char * s, size_t n, const char * fmt, ... );
#define snprintf(A,B,C,...) ms_snprintf (A,B,C,__VA_ARGS__)
#endif

#ifndef uint16_t
typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef unsigned int   uint32_t;
#endif

typedef unsigned long    DWORD;
typedef unsigned char   BYTE;
typedef unsigned char   UINT8;
typedef unsigned short  UINT16;
typedef unsigned int    UINT32;
typedef unsigned char * LPBYTE;
typedef unsigned short  USHORT;
typedef int            BOOL;
typedef unsigned long  ULONG;
typedef wchar_t         WCHAR;
typedef char            CHAR;
typedef BYTE  BOOLEAN;
#define FALSE   false
#define TRUE    true
typedef DWORD ULONG;
typedef unsigned int UINT;
typedef unsigned long DWORD_PTR;

//#define UNUSED(x) (void)(x)

#define CONNECTION_TYPE_NONE    0x0000
#define CONNECTION_TYPE_AG      0x0001
#define CONNECTION_TYPE_SPP     0x0002
#define CONNECTION_TYPE_AUDIO   0x0004
#define CONNECTION_TYPE_HF      0x0008
#define CONNECTION_TYPE_HIDH    0x0010
#define CONNECTION_TYPE_IAP2    0x0020
#define CONNECTION_TYPE_LE      0x0040
#define CONNECTION_TYPE_AVRC    0x0080
#define CONNECTION_TYPE_AVK     0x0100
#define CONNECTION_TYPE_PBC     0x0200
#define CONNECTION_TYPE_BATTC   0x0400
#define CONNECTION_TYPE_FINDMEL  0x0800
#define CONNECTION_TYPE_OPS     0x1000

#define NULL_HANDLE             0xFF
#define LE_DWORD(p) (((DWORD)(p)[0]) + (((DWORD)(p)[1])<<8) + (((DWORD)(p)[2])<<16) + (((DWORD)(p)[3])<<24))

// ADDED: Constant for number of BT devices
#define NUM_BT_DEVICES      1

typedef struct
{
    BYTE     *m_pWavData;
    BYTE     *m_pData;
    DWORD     m_dwWavDataLen;
    DWORD     m_dwChunkLen;
    DWORD     m_dwWavSent;
    DWORD     m_PacketsToSend; // incremented on receiving the message to send new buffers
    DWORD     m_PacketsSent;   // incremented in the write thread
    DWORD     m_BytesPerPacket;// received       
}hci_audio_sample_t;

class Worker;
// remote device information
class CBtDevice
{
public:
    CBtDevice (bool paired=false);
    ~CBtDevice ();

    UINT8 m_address[6];
    UINT8  address_type;
    UINT16 m_conn_type;
    UINT16 m_audio_handle;
    UINT16 m_spp_handle;
    UINT16 m_avrc_handle;
    UINT16 con_handle;
    UINT16 m_avk_handle;

    UINT8  role;

    char m_name[100];
    bool m_bIsLEDevice;

    QByteArray m_nvram;
    int m_nvram_id;
    bool m_paired;
};


// Main app
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT    

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QString m_SettingsFile;

    void closeEvent (QCloseEvent *event);
    void HandleDeviceEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    void Log(const char * tr,...);
    void DumpData(char *description, void* p, unsigned int length, unsigned int max_lines);
    BYTE ProcNibble (char n);
    USHORT GetHandle(QString &str);
    DWORD GetHexValue(LPBYTE buf, DWORD buf_size, QString &str);

    char * GetCurrentWorkingDirectory();
    DWORD qtmin(DWORD len, DWORD bufLen);

    // Device manager
    QMessageBox dl_msgbox;
    void WriteNVRAMToDevice(bool bBLEDevice);
    int FindBaudRateIndex(int baud);    
    void setDevName(char * dev_name);
    void setDevBda(BYTE* bda);
    void setPairingMode();
    void setVis();
    WicedSerialPort *m_CommPort;
    int errorNumber();
    bool m_bPortOpen ;
    bool SetupCommPort();
    bool SendWicedCommand(unsigned short command, unsigned char * payload, unsigned int len);
    int PortWrite(unsigned char * data, DWORD Length);
    void InitDm();
    void closeEventDm (QCloseEvent *event);
    void HandleDeviceEventsDm(DWORD opcode, LPBYTE p_data, DWORD len);
    void DecodeEIR(LPBYTE p_data, DWORD len, char * szName, int name_len);
    void EnableUI(bool bEnable);
    void EnableTabs(UINT8 feature, bool bEnable);
    bool m_bUIEnabled;
    void CloseCommPort();
    void ClearPort();    
    QWaitCondition serial_read_wait;
    bool m_scan_active;
    bool m_inquiry_active;
    void GetVersion();
    void HandleDeviceEventsMisc(DWORD opcode, LPBYTE p_data, DWORD len);
    UINT8 m_major;
    UINT8 m_minor;
    UINT8 m_rev;
    UINT8 m_build;
    uint32_t m_chip;
    UINT8 m_power;
    UINT16 m_features;
    QStringList m_strComPortsIDs;
    QIcon m_paired_icon;
    QSettings m_settings;
    FILE * m_fp_logfile;
    void VirtualUnplug(CBtDevice *pDev);


    void HandleA2DPEvents(DWORD opcode, DWORD len, BYTE *p_data);
    CBtDevice *AddDeviceToList(BYTE *addr, QComboBox *cb, char * bd_name=NULL,bool bPaired=false);
    CBtDevice * FindInList(BYTE * addr, QComboBox * pCb);
    CBtDevice * FindInList(UINT16 conn_type, UINT16 handle, QComboBox * pCb);
    void SelectDevice(QComboBox* cb, BYTE * bda);
    CBtDevice * GetSelectedDevice();
    CBtDevice * GetSelectedLEDevice();
    void ResetDeviceList(QComboBox *cb);
    void onHandleWicedEventDm(unsigned int opcode, unsigned char*p_data, unsigned int len);
    void SetDevicePaired(BYTE * bda);

    BOOL SendLaunchRam();
    BOOL SendDownloadMinidriver();
    void downlWoad(FILE * fHCD);    
    void SendRecvCmd(BYTE *arHciCommandTx, int tx_sz, BYTE *arBytesExpectedRx, int rx_sz);
    DWORD ReadCommPort(BYTE *lpBytes, DWORD dwLen, QSerialPort * m_CommPort);
    void ReadDevicesFromSettings(const char *group, QComboBox *cbDevices[], QPushButton *btnUnbond);

    // Serial port read
    QThread* m_port_read_thread;
    Worker* m_port_read_worker;
    void CreateReadPortThread();
    QMutex m_write;

    // audio source
    bool m_audio_connected;
    bool m_audio_started;

    bool m_volMute;
    FILE * m_fpAudioFile;
    hci_audio_sample_t m_uAudio;
    void InitAudioSrc();    
    void closeEventAudioSrc(QCloseEvent *event);
    void onHandleWicedEventAudioSrc(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleDeviceEventsAudioSrc(DWORD opcode, LPBYTE p_data, DWORD len);
    void HandleA2DPEventsAudioSrc(DWORD opcode, LPBYTE p_data, DWORD len);    
    void setAudioSrcUI();
    BYTE * ExecuteSetWavFile();
    void HandleA2DPAudioRequestEvent(BYTE * pu8Data, DWORD len);
    CBtDevice* GetConnectedAudioSrcDevice();
    BYTE* GetWavDataDataChunk(BYTE *pWavData, DWORD dwWavDataLen, DWORD *pdwDataLen);
    BYTE * ExecuteSetWavFile(char *pcFileName);
    BYTE* ReadFile(const char* FilePathName, DWORD *pdwWavDataLen);
    bool InitializeAudioFile();
    QMutex m_audio_packets;    
    QWaitCondition audio_tx_wait;

    // SPP
    void InitSPP();
    void onHandleWicedEventSPP(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleSPPEvents(DWORD opcode, LPBYTE p_data, DWORD len);
    DWORD SendFileThreadSPP();
    CBtDevice* GetConnectedSPPDevice();
    DWORD   m_spp_bytes_sent;
    DWORD   m_spp_total_to_send;
    BYTE    m_spp_tx_complete_result;
    FILE   *m_spp_receive_file;
    DWORD m_hSppTxCompleteEvent;
    QWaitCondition spp_tx_wait;

    QThread* m_thread_spp;
    Worker* m_worker_spp;

    // AVRC CT
    void InitAVRCCT();
    void onHandleWicedEventAVRCCT(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleDeviceEventsAVRCCT(DWORD opcode, LPBYTE p_data, DWORD len);
    void HandleAVRCControllerEvents(DWORD opcode, BYTE *p_data, DWORD len);
    void setAVRCCTUI();
    CBtDevice* GetConnectedAVRCDevice();


    // AVRC TG
    void InitAVRCTG();
    void onHandleWicedEventAVRCTG(unsigned int opcode, unsigned char *p_data, unsigned int len);
    void HandleDeviceEventsAVRCTG(DWORD opcode, LPBYTE p_data, DWORD len);
    void HandleAVRCTargetEvents(DWORD opcode, BYTE *p_data, DWORD len);
    void setAVRCTGUI();

    void SetTrack();
    void TrackInfo();
    void PlayerStatus();

    int m_current_volume_pct;

    // Recorder
    QAudioRecorder *m_audioRecorder;
    void InitRecorder();
    bool hasSpoken;

signals:
   void HandleWicedEvent(unsigned int opcode, unsigned int len, unsigned char *p_data);
   void HandleTrace(QString *pTrace);
   void ScrollToTop();

public slots:
    void startUpTimer();

   // utility methods
   void processTrace(QString * trace);
   void processScrollToTop();
   void onClear();
   void btnFindLogfileClicked();
   void btnLogToFileClicked(bool);
   void btnAddTraceClicked();
   void EnableAppTraces();
   void DisableAppTraces();
   void handleReadyRead();
   void serialPortError(QSerialPort::SerialPortError error);

    // Device manager
    void btnClearClicked();
    void onHandleWicedEvent(unsigned int opcode, unsigned int len, unsigned char *p_data);
    void onStartDisc();
    void onStopDisc();
    void onReset();
    void OnBnClickedBREDRUnbond();
    void onUnbond(QComboBox *cb);
    void onDevChange(QString);
    void onDiscoverable(bool);
    void onConnectable(bool);
    void onPairable(bool);
    void on_btnOpenPort_clicked();

    // AV source
    void onDisconnectAudioSrc();
    void onConnectAudioSrc();
    void onFindAudioFile();
    void onStartAudio();
    void onStopAudio();
    void onAudioSrcSine(bool);
    void onAudioSrcFile(bool);
    void on_btnHelpAVSRC_clicked();

    // Needed from HID
    void DumpMemory(BYTE * p_buf, int length);

    // Recorder
    void updateStatus(QMediaRecorder::Status);
    void onStateChanged(QMediaRecorder::State);
    void displayErrorMessage();

public:
    Ui::MainWindow *ui;

private slots:
    void on_btnHelpTab_clicked();
    void on_recordButton_clicked();
};

// Thread for SPP, iAP2 and serial port read
class Worker : public QObject
 {
     Q_OBJECT

public:
    explicit Worker() {}
    ~Worker(){}

    // Read serial port
    DWORD Read(BYTE * lpBytes, DWORD dwLen);
    DWORD ReadNewHciPacket(BYTE * pu8Buffer, int bufLen, int * pOffset);
    MainWindow * m_pParent;    

 public slots:
     void read_serial_port_thread();

 signals:
     void finished();
     void HandleWicedEvent(DWORD opcode, DWORD len, BYTE *p_data);

 };

class WaveFileWriter : public QThread
{
    Q_OBJECT

public:
    explicit WaveFileWriter(MainWindow * pParent);
    ~WaveFileWriter() {}
    MainWindow * m_pParent;
    void SendNextWav(hci_audio_sample_t * puHci, int bytesPerPacket);
    BYTE* GetWavDataDataChunk(BYTE *pWavData, DWORD dwWavDataLen, DWORD *pdwDataLen);
    BYTE * ExecuteSetWavFile(char *pcFileName);


protected:
    void run() Q_DECL_OVERRIDE;
};
extern bool m_bClosing ;
extern MainWindow *g_pMainWindow;

// ADDED: arrays for combo boxes
extern QComboBox * deviceLists[NUM_BT_DEVICES];

#endif // MAINWINDOW_H


