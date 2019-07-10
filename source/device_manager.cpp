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
 * Sample MCU application for using WICED HCI protocol.
 * This module implements device manager (DM) funtonality that
 * includes  -
 * - Local device management
 * - Peer device management
 * - WICED HCI handling
 * - Firmware download
 * - Serial port management
 * - Serial port thread
 */

#include "app_include.h"
#include <time.h>
#include <QTimer>

extern "C"
{
#include "app_host.h"
}


#define NVRAM_GROUP         "NVRAM"

extern void TraceHciPkt(BYTE type, BYTE *buffer, USHORT length, USHORT serial_port_index);
Q_DECLARE_METATYPE( CBtDevice* )

// CBtDevice data structure for caching peer device info
CBtDevice::CBtDevice (bool paired) :
    m_nvram(NULL), m_nvram_id(-1), m_paired(paired)
{
    memset(m_address,0,sizeof(m_address));
    memset(m_name,0,sizeof(m_name));

    m_audio_handle = NULL_HANDLE;
    m_spp_handle = NULL_HANDLE;
    m_avrc_handle = NULL_HANDLE;
    m_avk_handle = NULL_HANDLE;
    m_conn_type = 0;
    m_bIsLEDevice = false;
    role = 0;
}

CBtDevice::~CBtDevice ()
{
}

// valid baud rates
int as32BaudRate[] =
{
    115200,
    3000000,
#ifndef __MACH__
    4000000
#endif
};


// Initialize app
void MainWindow::InitDm()
{
    m_scan_active = m_inquiry_active = false;
    m_CommPort = NULL;

    m_port_read_worker = NULL;
    m_port_read_thread = NULL;

    EnableUI(false);

    // read settings for baudrate, serial port and flow-ctrl
    //QSettings settings(m_SettingsFile, QSettings::IniFormat);
    int baud_rate = m_settings.value("Baud",3000000).toInt();
    bool flow_control = m_settings.value("FlowControl",true).toBool();
    QString comm_port = m_settings.value("port").toString();
    ui->btnPairable->setChecked(m_settings.value("pairing_mode",true).toBool());
    ui->btnDiscoverable->setChecked(m_settings.value("discoverable",true).toBool());
    ui->btnConnectable->setChecked(m_settings.value("Connectable",true).toBool());

    // setup icon in device list for paried devices
    ui->cbDeviceList->setIconSize(QSize(20,20));

    //ReadDevicesFromSettings("devices", deviceLists, ui->btnUnbond);

    // get list of all available serial ports
    int port_inx = -1;
    m_strComPortsIDs.clear();
    m_strComPortsIDs.append("<select port>"); // dummy string to match combo box
    QList<QSerialPortInfo> port_list = QSerialPortInfo::availablePorts();
    for (int i =0; i < port_list.size(); i++)
    {        
        QString strName = port_list.at(i).portName();
        QString strDesc =  port_list.at(i).description();
        strName += " (" + strDesc + ")";
        QString strPortID = port_list.at(i).systemLocation();

        // m_strComPortsIDs contains serial port ID used to open the port
        m_strComPortsIDs.append(strPortID);

        // cbCommport contains friendly names
        ui->cbCommport->addItem(strName, strPortID);

        for (int i = 0; i < 6; i++) {
            connectedDeviceAddress[i] = 0;
        }
    }

#ifndef Q_OS_WINDOWS
#ifndef Q_OS_MACOS
    // add entry in list for emulator
    ui->cbCommport->addItem(QString("host-mode"), "0");
    m_strComPortsIDs.append("0");
#endif
#endif

    if ( -1 != (port_inx = ui->cbCommport->findText(comm_port)))
    {
        ui->cbCommport->setCurrentIndex(port_inx);
    }

    // populate dropdown list of baud rates
    QString strBaud;
    int baud_inx = (sizeof(as32BaudRate) / sizeof(as32BaudRate[0])) - 1; // select default baud rate as highest allowed
    for (int i = 0; i < (int) (sizeof(as32BaudRate) / sizeof(as32BaudRate[0])); i++)
    {
        strBaud.sprintf( "%d", as32BaudRate[i]);
        ui->cbBaudRate->addItem(strBaud,i);
        if (as32BaudRate[i] == baud_rate)
            baud_inx = i;
    }
    ui->cbBaudRate->setCurrentIndex(baud_inx);
    ui->btnFlowCntrl->setChecked(flow_control );


    // setup signals/slots
    connect(ui->btnStartDisc, SIGNAL(clicked()), this, SLOT(onStartDisc()));
    connect(ui->btnStopDisc, SIGNAL(clicked()), this, SLOT(onStopDisc()));    
    connect(ui->btnReset, SIGNAL(clicked()), this, SLOT(onReset()));
    connect(ui->btnUnbond, SIGNAL(clicked()), this, SLOT(OnBnClickedBREDRUnbond()));
    connect(ui->cbDeviceList, SIGNAL(currentTextChanged(QString)), this, SLOT(onDevChange(QString)));
    connect(ui->btnDiscoverable, SIGNAL(clicked(bool)), this, SLOT(onDiscoverable(bool)));
    connect(ui->btnConnectable, SIGNAL(clicked(bool)), this, SLOT(onConnectable(bool)));
    connect(ui->btnPairable, SIGNAL(clicked(bool)), this, SLOT(onPairable(bool)));
    connect(&dl_msgbox,SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(onMsgBoxButton(QAbstractButton*)));

    m_bPortOpen = false;

    m_major = 0;
    m_minor = 0;
    m_rev = 0;
    m_build = 0;
    m_chip = 0;
    m_power = 0;
    m_features = 0xFF;

    char strBda[100];
    srand (time(NULL));
    sprintf(strBda, "%02X:%02X:%02X:%02X:%02X:%02X", 0x2, 0x0, 0x7, rand() % 255, rand() % 255, rand() % 255);
}

// Enable or disable UI
void MainWindow::EnableUI(bool bEnable)
{
    EnableTabs(0xFF, bEnable);

    ui->btnStartDisc->setEnabled(bEnable);
    ui->btnReset->setEnabled(bEnable);
    ui->btnStopDisc->setEnabled(bEnable);
    ui->btnPairable->setEnabled(bEnable);
    ui->btnDiscoverable->setEnabled(bEnable);
    ui->btnConnectable->setEnabled(bEnable);

    m_bUIEnabled = bEnable;

}

// Enable UI tabs depending on supported features of embedded device
void MainWindow::EnableTabs(UINT8 feature, bool bEnable)
{
    if((feature != 0xFF) && bEnable)
    {
        switch(feature)
        {
        case HCI_CONTROL_GROUP_AUDIO:
            ui->tabAVSRC->setEnabled(bEnable);
            Log("AV Source");
            break;
        }
    }
    else
    {
        ui->tabAVSRC->setEnabled(bEnable);
    }          
}

/************** Local device management *************/

// read info of paired devices saved on disk
void MainWindow::ReadDevicesFromSettings(const char *group, QComboBox *cbDevices[], QPushButton *btnUnbond)
{
    // get paired devices
    m_settings.beginGroup(group);
    QStringList  device_keys = m_settings.allKeys();

    int nvram_id=-1;
    QString dev_name_id, dev_name="";
    int bda0,bda1,bda2,bda3,bda4,bda5;
    for (int i = 0; i < device_keys.size(); i++)
    {
        dev_name_id = m_settings.value(device_keys[i],"").toString();
        if (dev_name_id.size() >= 3)
        {
            sscanf(dev_name_id.toStdString().c_str(),"%02X:", &nvram_id);
            if (dev_name_id.size() > 4)
                dev_name = dev_name_id.right(dev_name_id.size()-3);
        }

        sscanf(device_keys[i].toStdString().c_str(), "%02X:%02X:%02X:%02X:%02X:%02X",
               &bda0, &bda1, &bda2, &bda3, &bda4, &bda5);
        BYTE bda[6];
        bda[0] = bda0;
        bda[1] = bda1;
        bda[2] = bda2;
        bda[3] = bda3;
        bda[4] = bda4;
        bda[5] = bda5;

        CBtDevice * pDev = AddDeviceToList(bda, ui->cbDeviceList, (char *)dev_name.toStdString().c_str(),true);
        if (pDev && nvram_id != -1)
        {
            pDev->m_nvram_id = nvram_id;
        }
        if(pDev && (strcmp(group, "devicesLE") == 0))
        {
            pDev->m_bIsLEDevice = true;
        }
    }
    m_settings.endGroup();

    // get link key for paired devices
    m_settings.beginGroup("NVRAM");
    for (int i = 0; i < cbDevices[0]->count(); i++) // TODO: Segfault when counting for array of one
    {
        QVariant var;
        var = cbDevices[0]->itemData(i);
        CBtDevice * pDev = var.value<CBtDevice *>();
        if (NULL == pDev)
            continue;
        if (pDev->m_nvram_id == -1)
            continue;
        QString nvram_id_key;
        nvram_id_key.sprintf("%02X",pDev->m_nvram_id);
        pDev->m_nvram = m_settings.value(nvram_id_key).toByteArray();
    }
    m_settings.endGroup();

    btnUnbond->setEnabled(false);
    if (cbDevices[0]->count())
    {
        cbDevices[0]->setCurrentIndex(0);
        btnUnbond->setEnabled(true);
    }
}

// Set local device to be discoverable
void MainWindow::onDiscoverable(bool)
{
    m_settings.setValue("discoverable",ui->btnDiscoverable->isChecked());
    setVis();
}

// Set local device to be connectable
void MainWindow::onConnectable(bool)
{
    m_settings.setValue("Connectable",ui->btnConnectable->isChecked());
    setVis();
}

// Set local device to be pair-able
void MainWindow::onPairable(bool)
{
    m_settings.setValue("pairing_mode",ui->btnPairable->isChecked());
    setPairingMode();
}

// set local device name
void MainWindow::setDevName(char *device_name)
{
    Log("setDevName");

    if(!m_CommPort)
        return;

    if (!m_CommPort->isOpen())
        return;

    BYTE   cmd[60];
    int    commandBytes = 0;

    commandBytes = strlen(device_name);
    memcpy(cmd, device_name, commandBytes);

    Log("setDevName: %s", device_name);

    SendWicedCommand(HCI_CONTROL_COMMAND_SET_LOCAL_NAME, cmd, commandBytes);
}

// set local device BD address
void MainWindow::setDevBda(BYTE* bda)
{
    Log("setDevBda");

    if (!m_CommPort)
        return;

    if (!m_CommPort->isOpen())
        return;

    BYTE    cmd[16];
    uint8_t *p = cmd;
    int     commandBytes = BD_ADDR_LEN;

    BDADDR_TO_STREAM(p, bda);

    Log("setDevBda: addr = %02X:%02X:%02X:%02X:%02X:%02X", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    SendWicedCommand(HCI_CONTROL_COMMAND_SET_LOCAL_BDA, cmd, commandBytes);
}

// Set local device pairable or non pairable
void MainWindow::setPairingMode()
{
    if(!m_CommPort)
        return;

    if (!m_CommPort->isOpen())
        return;

    BYTE pairing_mode=ui->btnPairable->isChecked() ? 1 : 0;
    SendWicedCommand(HCI_CONTROL_COMMAND_SET_PAIRING_MODE, &pairing_mode, 1);
}

// set local device visibility
void MainWindow::setVis()
{
    if(!m_CommPort)
        return;

    if (!m_CommPort->isOpen())
        return;

    BYTE   cmd[60];
    int    commandBytes = 0;

    cmd[commandBytes++] = ui->btnDiscoverable->isChecked() ? 1:0; //discoverable
    cmd[commandBytes++] = ui->btnConnectable->isChecked() ? 1:0; ; //CONNECTABLE

    SendWicedCommand(HCI_CONTROL_COMMAND_SET_VISIBILITY, cmd, commandBytes);
}

// Reset local device
void MainWindow::onReset()
{
    if (m_CommPort == NULL)
        return;

    if (!m_bPortOpen)
    {
         return;
    }

    if (QMessageBox::Yes != QMessageBox(QMessageBox::Information, "Reset", "Are you sure you want to reset? After resetting the device, you may need to download the embedded application again.", QMessageBox::Yes|QMessageBox::No).exec())
    {
        return;
    }

    // send command to reset
    SendWicedCommand(HCI_CONTROL_COMMAND_RESET, 0, 0);

    // close comm port reader thread and disable UI
    QThread::msleep(10);

    ClearPort();

}

/************** Peer device management *************/

// get selected BR/EDR device (ORIGINAL)
CBtDevice * MainWindow::GetSelectedDevice()
{
    int i = ui->cbDeviceList->currentIndex();
    if ( i == -1)
        return NULL;

    QVariant var;
    var = ui->cbDeviceList->itemData(i);
    return var.value<CBtDevice *>();
}

// unpair a BR-EDR device
void MainWindow::OnBnClickedBREDRUnbond()
{
    onUnbond(ui->cbDeviceList);
}

// unpair the device
void MainWindow::onUnbond(QComboBox *cb)
{
    if (m_CommPort == NULL)
    {
        Log("Serial port is not open");
        return;
    }

    if (!m_bPortOpen)
    {
        Log("Serial port is not open");
        return;
    }

    int i = cb->currentIndex();
    if ( i == -1)
        return;

    QVariant var;
    var = cb->itemData(i);

    CBtDevice * pDev = var.value<CBtDevice *>();
    if (pDev == NULL)
        return;

    m_settings.beginGroup("devices");
    QString strBda;
    strBda.sprintf("%02X:%02X:%02X:%02X:%02X:%02X",
        pDev->m_address[0], pDev->m_address[1], pDev->m_address[2], pDev->m_address[3], pDev->m_address[4], pDev->m_address[5]);
    m_settings.remove(strBda);
    m_settings.endGroup();

    m_settings.beginGroup("NVRAM");
    QString nvram_id_key;
    nvram_id_key.sprintf("%02X",pDev->m_nvram_id);
    m_settings.remove(nvram_id_key);
    m_settings.endGroup();

    m_settings.sync();

    BYTE   cmd[10];
    cmd[0] = (BYTE)pDev->m_nvram_id;
    cmd[1] = (BYTE)(pDev->m_nvram_id >> 8);
    SendWicedCommand(HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA, cmd, 2);

    pDev->m_paired = false;
    pDev->m_nvram_id = -1;
    pDev->m_nvram.clear();

    ui->btnUnbond->setEnabled(false);

    cb->setItemIcon(cb->currentIndex(), *new QIcon);
}


// User changed selected BR/EDR device
void MainWindow::onDevChange(QString)
{
    CBtDevice * pDev = GetSelectedDevice();
    if (pDev)
    {
         ui->btnUnbond->setEnabled(pDev->m_paired);
    }
}

// BR/EDR discovery start
void MainWindow::onStartDisc()
{
    if (!m_inquiry_active)
    {
        m_inquiry_active = TRUE;

        // Clear all items from combo box except paired devices
        ResetDeviceList(ui->cbDeviceList);

        BYTE command[] = { 1 };
        int r = SendWicedCommand(HCI_CONTROL_COMMAND_INQUIRY, command, 1);
        if (r <= 0)
        {
            Log("Error starting inquiry");
            m_inquiry_active = FALSE;
        }
    }

    ui->btnStartDisc->setEnabled(!m_inquiry_active);
    ui->btnStopDisc->setEnabled(m_inquiry_active);
}

// BR/EDR discovery stop
void MainWindow::onStopDisc()
{
    m_inquiry_active = FALSE;

    BYTE command[] = { 0 };
    if (SendWicedCommand(HCI_CONTROL_COMMAND_INQUIRY, command, 1) <= 0)
    {
        Log("Error stopping inquiry");
    }

    ui->btnStartDisc->setEnabled(!m_inquiry_active);
    ui->btnStopDisc->setEnabled(m_inquiry_active);
}

// Find a paired device in list of devices using BDA
CBtDevice *MainWindow::FindInList(BYTE * addr, QComboBox * pCb)
{
    int j;
    CBtDevice *device = NULL;
    for (int i = 0; i < pCb->count(); i++)
    {
        device = (CBtDevice *)pCb->itemData(i).value<CBtDevice *>();
        if (device == NULL)
            continue;
        for (j = 0; j < 6; j++)
        {
            if (device->m_address[j] != addr[j])
                break;
        }
        if (j == 6)
        {
            return device;
        }
    }
    return NULL;
}

// Find a paired device in list of devices using connection type
CBtDevice *MainWindow::FindInList(UINT16 conn_type, UINT16 handle, QComboBox * pCb)
{
    UNUSED(handle);

    CBtDevice *device = NULL;
    for (int i = 0; i < pCb->count(); i++)
    {
        device = (CBtDevice *)pCb->itemData(i).value<CBtDevice *>();
        if (device == NULL)
            continue;

        if(device->m_conn_type & conn_type)
            return device;
    }
    return NULL;
}

// Set a device as currently selected in the combo box
void MainWindow::SelectDevice(QComboBox* cb, BYTE * bda)
{
    int j;
    CBtDevice *device = NULL;
    for (int i = 0; i < cb->count(); i++)
    {
        device = (CBtDevice *)cb->itemData(i).value<CBtDevice *>();
        if (device == NULL)
            continue;
        for (j = 0; j < 6; j++)
        {
            if (device->m_address[j] != bda[j])
                break;
        }
        if (j == 6)
        {
            cb->setCurrentIndex(i);
            break;
        }
    }
}

// Set device status to paired
void MainWindow::SetDevicePaired(BYTE * bda)
{
    QString strBda;
    strBda.sprintf("%02X:%02X:%02X:%02X:%02X:%02X",
                   bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    bool bLeDevice = false;
    int i = ui->cbDeviceList->findText(strBda,Qt::MatchStartsWith);
    if (i == -1)
        return;

    CBtDevice * device = NULL;
    device = (CBtDevice *)ui->cbDeviceList->itemData(i).value<CBtDevice *>();

    if (device == NULL)
        return;

    if (device->m_paired)
        return;

    device->m_paired = true;

    ui->cbDeviceList->setItemIcon(i,m_paired_icon);

    m_settings.beginGroup(bLeDevice? "devicesLE" : "devices");
    m_settings.setValue(strBda,device->m_name);
    m_settings.endGroup();
    m_settings.sync();

    ui->btnUnbond->setEnabled( (i == ui->cbDeviceList->currentIndex()));
}

// Add new device to combo box
CBtDevice *MainWindow::AddDeviceToList(BYTE *addr, QComboBox *cb, char * bd_name, bool paired)
{
    CBtDevice *device = NULL;
    QString abuffer;

    Log("In AddDeviceToList"); // REMOVE
    if (bd_name)
        abuffer.sprintf("%02x:%02x:%02x:%02x:%02x:%02x (%s)",
            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5],bd_name);
    else
        abuffer.sprintf("%02x:%02x:%02x:%02x:%02x:%02x",
            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Check if device is already present
    int i = cb->findText( abuffer,Qt::MatchStartsWith);
    if (i == -1)
    {
        QVariant qv;

        device = new CBtDevice(paired);
        if (bd_name && strlen(bd_name))
            strncpy(device->m_name, bd_name,sizeof(device->m_name)-1);

        qv.setValue<CBtDevice *>(device);

        if (paired)
        {
            cb->addItem( m_paired_icon, abuffer, qv);
        }
        else
        {
            cb->addItem( abuffer, qv );
        }

        memcpy(device->m_address, addr, 6);
    }
    else
    {
        // device already in list

        // If paired, set icon
        if(paired)
            for (int i = 0; i < NUM_BT_DEVICES; i++) {
                cb->setItemIcon(i,m_paired_icon);
            }

        device = (CBtDevice *)cb->itemData(i).value<CBtDevice*>();
    }

    return device;
}

// TODO: add ability to push second device to NVRAM
// Push NVRAM data to embedded device
void MainWindow::WriteNVRAMToDevice(bool bBLEDevice)
{
    QVariant var;
    CBtDevice * pDev = NULL;
    BYTE cmd[1000];
    int len=0;

    QComboBox *cb = ui->cbDeviceList;

    for (int i = 0; i < cb->count(); i++)
    {
        var = cb->itemData(i);
        if (NULL == (pDev = var.value<CBtDevice *>()))
            continue;
        if (pDev->m_nvram_id == -1 || pDev->m_nvram.size() == 0)
            continue;

        len = pDev->m_nvram.size();
        if (len <= int (sizeof(cmd) - 2))
        {
            cmd[0] = (BYTE)pDev->m_nvram_id;
            cmd[1] = (BYTE)(pDev->m_nvram_id >> 8);
            memcpy(&cmd[2], pDev->m_nvram.constData(), len);
            SendWicedCommand(HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA, cmd, 2 + len);
        }
        else
        {
            Log("Err: NVRAM chunk too big");
        }
    }
}

// Clear all items from combo box except paired devices
void MainWindow::ResetDeviceList(QComboBox *cb)
{
    CBtDevice *device = NULL;
    QComboBox temp;
    int i = 0;
    // Save the paired devices to a temp combo box
    for(i = 0; i < cb->count(); i++)
    {
        device = (CBtDevice *)cb->itemData(i).value<CBtDevice *>();
        if(device && device->m_paired)
        {
            QVariant qv;
            qv.setValue<CBtDevice *>(device);

            temp.addItem(cb->itemText(i), qv);
        }
    }

    // clear the device combo box
    cb->clear();

    // add the paired devices back to device combo box
    for(i = 0; i < temp.count(); i++)
    {
        device = (CBtDevice *)temp.itemData(i).value<CBtDevice *>();
        if(device)
        {
            QVariant qv;
            qv.setValue<CBtDevice *>(device);

            cb->addItem(m_paired_icon, temp.itemText(i), qv);
        }
    }
}


/*************************WICED HCI handling*********************************/

// Send WICED HCI commmand to embedded device
bool MainWindow::SendWicedCommand(unsigned short command, unsigned char * payload, unsigned int len)
{
    return wiced_hci_send_command(command, payload, len);
}

// Send WICED HCI commmand to embedded device
int MainWindow::PortWrite(unsigned char * data, DWORD Length)
{
    int64_t dwWritten = 0;
    DWORD dwTotalWritten = 0;
    unsigned char * p = &data[0];

    if (NULL == m_CommPort)
        return -1;
    if (!m_CommPort->isOpen())
        return -1;

    // Since the write to serial port can happen from main thread as well as
    // audio wav thread, serialize the write of WICED HCI packets
    m_write.lock();

    // Loop and write data to serial port till all is written
    while (Length && m_bPortOpen)
    {
        dwWritten = 0;

        dwWritten = m_CommPort->write((const char*)p, Length);
        m_CommPort->flush();

        if(dwWritten == -1)
        {
            Log("error - port write error");
            break;
        }

        if(Length != (DWORD)dwWritten)
        {
            if(!m_bPortOpen)
                break;

            Log("port write mismatch, to write %d, written %d, wait and try", Length, dwWritten);

            if (!m_CommPort->waitForBytesWritten(100))
            {
                Log("error - waitForBytesWritten");
                break;
            }

            if(!m_bPortOpen)
                break;
        }
        if ( (DWORD)dwWritten > Length)
            break;

        Length -=  (DWORD)dwWritten;

        if(Length)
            p += dwWritten;

        dwTotalWritten += dwWritten;
    }

    m_write.unlock();


    return dwTotalWritten;
}

// Command to get version and supported features of embedded app
void MainWindow::GetVersion()
{
    // send command to get version and feature info
    SendWicedCommand(HCI_CONTROL_MISC_COMMAND_GET_VERSION, NULL, 0);
}

// Command to enable WICED HCI traces
void MainWindow::EnableAppTraces()
{
    BYTE cmd[2];
    cmd[0] = TRUE; // Enable Bluetooth HCI trace
    cmd[1] = 1;    //  WICED_ROUTE_DEBUG_TO_WICED_UART;

    // send command to configure traces
    SendWicedCommand(HCI_CONTROL_COMMAND_TRACE_ENABLE, cmd, 2);
}

// Command to disable WICED HCI traces
void MainWindow::DisableAppTraces()
{
    BYTE cmd[2];
    cmd[0] = FALSE; // Disable Bluetooth HCI trace
    cmd[1] = 1;    //  WICED_ROUTE_DEBUG_TO_WICED_UART;

    // send command to configure traces
    SendWicedCommand(HCI_CONTROL_COMMAND_TRACE_ENABLE, cmd, 2);
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEventDm(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    if(!m_bUIEnabled)
    {
        EnableUI(true);
        EnableAppTraces();
        Log("Client Control app established communication with Bluetooth device.");
    }

    switch (HCI_CONTROL_GROUP(opcode))
    {
        case HCI_CONTROL_GROUP_DEVICE:
            HandleDeviceEventsDm(opcode, p_data, len);
            break;
        case HCI_CONTROL_GROUP_MISC:
            HandleDeviceEventsMisc(opcode, p_data, len);
            break;

    }
}

// Handle WICED HCI events for device management
void MainWindow::HandleDeviceEventsDm(DWORD opcode, LPBYTE p_data, DWORD len)
{
    BYTE    bda[6];
    char trace[1024];
    char num[5];
    QVariant var;
    CBtDevice * pDev;
    int dev_idx;

    switch (opcode)
    {
    // received embedded app traces
    case HCI_CONTROL_EVENT_WICED_TRACE:
        if (len >= 2)
        {            
            if ((len > 2) && (p_data[len - 2] == '\n'))
            {
                p_data[len - 2] = 0;
                len--;
            }
            // If there are two or more devices connected to the computer, we
            // need a way to distingusih trace sent to BTSpy from each device.
            // When printing embedded app traces to BTSpy, prepend the index of the
            // serial port in the drop down combo box to the trace.
            sprintf(num, "%d", ui->cbCommport->currentIndex());
            trace[0] =  num[0];
            trace[1] = ' ';
            memcpy(&trace[2], p_data, len);
            TraceHciPkt(0, (BYTE *)trace, (USHORT)len+2, ui->cbCommport->currentIndex());
        }

        break;

        // received HCI traces
    case HCI_CONTROL_EVENT_HCI_TRACE:
        // If there are two or more devices connected to the computer, we
        // need a way to distingusih trace sent to BTSpy from each device.
        // When printing HCI protocol traces to BTSpy, the index of the
        // serial port in the drop down combo box will be displayed in the
        // HCI trace.
        if (p_data)
            TraceHciPkt(p_data[0] + 1, &p_data[1], (USHORT)(len - 1), ui->cbCommport->currentIndex());
        break;

        // received paired device data for all to save in NVRAM
    /*
    case HCI_CONTROL_EVENT_NVRAM_DATA:
    {
        QString nvram_id_key;
        nvram_id_key.sprintf("%02X", p_data[0] + (p_data[1] << 8));
        m_settings.beginGroup("NVRAM");
        // Use QByteArray constructor with length parameter otherwise a 0x00 is considered as '\0'
        QByteArray val ((const char *)&p_data[2], len - 2);
        m_settings.setValue(nvram_id_key,val);
        m_settings.endGroup();
        m_settings.sync();

        Log("HCI_CONTROL_EVENT_NVRAM_DATA %s", nvram_id_key.toLocal8Bit().data());

        // if it is a link key add this device to the appropriate combobox
        if (len - 2)
        {
            // application should probably send type of the device, if host does not remember.
            // for now we will use the fact that BR/EDR keys are just 16 bytes.
            BOOL bLeDevice = (p_data[25] | p_data[26] | p_data[27]) != 0;

            // Adds previously paired devices to list
            CBtDevice * pDev = AddDeviceToList(&p_data[2], ui->cbDeviceList, NULL, false);
            // TODO: add to second list as well
            pDev->m_nvram_id =  p_data[0] + (p_data[1] << 8);
            pDev->m_nvram = QByteArray ((const char *)&p_data[2], len - 2);
            if(bLeDevice)
                pDev->m_bIsLEDevice = true;
            pDev->m_paired = true;

            // save the paired device data to disk
            if(!bLeDevice)
                m_settings.beginGroup("devices");
            else
                m_settings.beginGroup("devicesLE");

            QString dev_key;
            dev_key.sprintf("%02X:%02X:%02X:%02X:%02X:%02X",
                pDev->m_address[0], pDev->m_address[1], pDev->m_address[2],
                pDev->m_address[3], pDev->m_address[4], pDev->m_address[5] );
            QString name_id;
            name_id.sprintf("%02X:%s", pDev->m_nvram_id, pDev->m_name);
            m_settings.setValue(dev_key,name_id);
            m_settings.endGroup();
            m_settings.sync();

            // Set Unbond button state for selected device
            onDevChange(dev_key);

            Log("device (LE? %d) %s, ID %s", bLeDevice, dev_key.toLocal8Bit().data(), name_id.toLocal8Bit().data());

        }
    }
    break;
    */

        // received event for user to confirm pairing
    case HCI_CONTROL_EVENT_USER_CONFIRMATION:
    {
        Log("Numeric Comparison %02x:%02x:%02x:%02x:%02x:%02x Value:%d", p_data[0], p_data[1], p_data[2], p_data[3], p_data[4], p_data[5], p_data[6] + (p_data[7] << 8) + (p_data[8] << 16) + (p_data[9] << 24));
        QString str;
        str.sprintf("Confirm pairing code: %d", p_data[6] + (p_data[7] << 8) + (p_data[8] << 16) + (p_data[9] << 24));
        if (QMessageBox::Yes != QMessageBox(QMessageBox::Information, "User Confirmation", str, QMessageBox::Yes|QMessageBox::No).exec())
        {
            break;
        }

        // send command to confirm user confirmation value
        {
            BYTE command[20];

            for (int i = 0; i < 6; i++)
                command[5 - i] = p_data[i];

            command[6] = 1; // 1 - accept, 0 - do not accept

            //SendWicedCommand(HCI_CONTROL_COMMAND_USER_CONFIRMATION, command, 7);
        }
    }
        break;

        // received event that local device started
    case HCI_CONTROL_EVENT_DEVICE_STARTED:
        Log("Device Started");
        m_scan_active = false;
        m_inquiry_active = false;
        ui->btnStartDisc->setEnabled(!m_inquiry_active);
        ui->btnStopDisc->setEnabled(m_inquiry_active);

        m_settings.sync();

        if (!m_audio_connected) {
            // Mark every BE/EDR device as disconnected
            Log("NO DEVICE PAIRED YET, DISCONNETING EVERY DEVICE \n");
            for (dev_idx = 0 ; dev_idx < ui->cbDeviceList->count() ; dev_idx++)
            {
                var = ui->cbDeviceList->itemData(dev_idx);
                pDev = var.value<CBtDevice *>();
                if(pDev != NULL)
                {
                    pDev->m_conn_type = CONNECTION_TYPE_NONE;
                    pDev->m_audio_handle = NULL_HANDLE;
                    pDev->m_avk_handle = NULL_HANDLE;
                    pDev->m_avrc_handle = NULL_HANDLE;
                }
            }
        } else {
            UINT8* address = connected_device->m_address;
            Log("ALREADY HAS DEVICE PAIRED, device address is");
            Log("%02X:%02X:%02X:%02X:%02X:%02X \n", address[0], address[1], address[2], address[3], address[4], address[5]) ;
            SetDevicePaired(connectedDeviceAddress);
            pDev = AddDeviceToList(connectedDeviceAddress, ui->cbDeviceList, NULL, true);
            pDev->m_audio_handle = 1;
            pDev->m_conn_type |= CONNECTION_TYPE_AUDIO;
            pDev->m_avk_handle = 1;
            pDev->m_avrc_handle = 1;
            connected_device = pDev;
        }

        setVis();           // Set Discoverable and/or Connectable (depending on CheckBoxes)
        setPairingMode();   // Set Pairable (depending on CheckBox)

        //WriteNVRAMToDevice(false);
        //WriteNVRAMToDevice(true);
        break;

        // received event that pairing completed
    case HCI_CONTROL_EVENT_PAIRING_COMPLETE:
    {        
        Log("Pairing status: %s, code: %d", (p_data[0] == 0) ? "Success" : "Fail", (int)p_data[0]);
        if(len >= 7)
        {
            BYTE command[20];
            for (int i = 0; i < 6; i++)
                command[5 - i] = p_data[i+1];
            Log("BDA %02x:%02x:%02x:%02x:%02x:%02x",
                command[0], command[1], command[2], command[3], command[4], command[5]);

            SetDevicePaired(&p_data[1]);
        }
        break;
    }

        // received BR/EDR inquiry result
    case HCI_CONTROL_EVENT_INQUIRY_RESULT:
    {        
        Log("Device found: %02x:%02x:%02x:%02x:%02x:%02x device_class %02x:%02x:%02x rssi:%d",
            (int)p_data[5], (int)p_data[4], (int)p_data[3], (int)p_data[2], (int)p_data[1], (int)p_data[0],
                (int)p_data[8], (int)p_data[7], (int)p_data[6],(int) p_data[9] - 256);

        char szName[50] = {0};

#if 0
        if (ui->cbCommport->itemData(ui->cbCommport->currentIndex()).toString().compare("0") == 0)
        {
            if (len > 11)
            {
                DecodeEIR(&p_data[11], len - 11, szName,sizeof(szName));
            }
        }
        else
#endif
        {
            if (len > 10)
            {
                DecodeEIR(&p_data[10], len - 10, szName,sizeof(szName));
            }
        }

        // dump advertisement data to the trace
        /*
        if (len > 10)
        {
            trace[0] = 0;
            for (int i = 0; i < (int)len - 10; i++)
                sprintf(&trace[strlen(trace)], "%02x", p_data[10 + i]);
            Log(trace);

        }
        */
        for (int i = 0; i < 6; i++)
            bda[5 - i] = p_data[i];
        // check if this device is not present yet.
        if (FindInList(bda,ui->cbDeviceList) != NULL)
            break;
        AddDeviceToList(bda, ui->cbDeviceList, szName);
    }
        break;

        // received event that BR/EDR inquiry is complete
    case HCI_CONTROL_EVENT_INQUIRY_COMPLETE:
        if (m_inquiry_active)
        {
            m_inquiry_active = FALSE;
            Log("Inquiry Complete");
        }
        ui->btnStartDisc->setEnabled(!m_inquiry_active);
        ui->btnStopDisc->setEnabled(m_inquiry_active);

        break;

    case HCI_CONTROL_EVENT_CONNECTED_DEVICE:
        Log("CHECKED CONNECTED DEVICE");
        bool empty_addr = TRUE;
        for (int i = 0; i < 6; i++) {
            if (p_data[i] != 0) {
                empty_addr = FALSE;
                break;
            }
        }
        if (!empty_addr) {
            memcpy(connectedDeviceAddress, p_data, 6);
            m_audio_connected = TRUE;
            Log("THERE IS DEVICE CONNECTED");
            connected_device = AddDeviceToList(connectedDeviceAddress, ui->cbDeviceList, NULL, true);
            app_host_add_device(connectedDeviceAddress);
            UINT8* address = connected_device->m_address;
            Log("%02X:%02X:%02X:%02X:%02X:%02X \n", address[0], address[1], address[2], address[3], address[4], address[5]) ;
            SetDevicePaired(connectedDeviceAddress);
            connected_device->m_audio_handle = 1;
            connected_device->m_conn_type |= CONNECTION_TYPE_AUDIO;
            connected_device->m_avk_handle = 1;
            connected_device->m_avrc_handle = 1;
        }
        break;
    }

}

// handle misc event
void MainWindow::HandleDeviceEventsMisc(DWORD opcode, LPBYTE tx_buf, DWORD len)
{
    int index = 0;

    switch (opcode)
    {
    // recieved version info event
    case HCI_CONTROL_MISC_EVENT_VERSION:
        m_major = tx_buf[index++];
        m_minor = tx_buf[index++];
        m_rev =  tx_buf[index++];
        uint8_t data1 = tx_buf[index++];
        uint8_t data2 = tx_buf[index++];
        m_build = data1 | (data2 << 8);
        data1 = tx_buf[index++];
        data2 = tx_buf[index++];
        uint8_t data3 = tx_buf[index++];
        m_chip = data1 | (data2 << 8) | (data3 << 24);
        m_power = tx_buf[index++];
        if(len < 10) // old API len is 9
            break;

        Log("Tabs supported by embedded application - ");
        // Embedded device sent us supported features.
        // First disable all tabs, then enable supported
        // tabs one at a time.
        EnableTabs(0xFF, false);
        for(; index < (int) len; index++)
        {
            EnableTabs(tx_buf[index], true);
        }

        break;
    }
}

/************** Serial port management *************/

// User clicked button to open or close serial port
void MainWindow::on_btnOpenPort_clicked()
{
    // If port is not open, open it
    if(!m_bPortOpen)
    {
        EnableUI(false);
        ui->btnOpenPort->setEnabled(false);
        Log("setting up commport");
        bool bopen = SetupCommPort();
        ui->btnOpenPort->setEnabled(true);

        if(!bopen)
        {
            QMessageBox(QMessageBox::Information, "Serial Port", "Error opening serial port", QMessageBox::Ok).exec();
        }
        else
        {
            ui->cbCommport->setEnabled(false);
            ui->btnOpenPort->setText("Close Port");
            ui->cbBaudRate->setEnabled(false);
            ui->btnFlowCntrl->setEnabled(false);
        }

    }
    // Close port if open
    else
    {
#ifdef PCM_ALSA
        if (m_alsa_handle != NULL)
        {
            snd_pcm_close(m_alsa_handle);
            m_alsa_handle = NULL;
        }
#endif
        ClearPort();
    }
}

// Open and setup serial port
bool MainWindow::SetupCommPort()
{
    int baud_rate = ui->cbBaudRate->currentText().toInt();
    m_settings.setValue("Baud",baud_rate);

    QString comm_port = ui->cbCommport->currentText();
    m_settings.setValue("port",comm_port);

    if (m_CommPort)
    {
        ClearPort();
    }

    // Find the serial port ID from the index of serial port combo box
    QString serialPortName;

    int serialPortBaudRate = as32BaudRate[ui->cbBaudRate->currentIndex()];
    bool bFlow = ui->btnFlowCntrl->isChecked();
    if (ui->cbCommport->itemData(ui->cbCommport->currentIndex()).toString().compare("0") == 0)
    {
        m_CommPort = new WicedSerialPortHostmode();
        serialPortName = "host-mode";
    }
    else
    {
    m_CommPort = new WicedSerialPort();
    // Find the serial port ID from the index of serial port combo box
        serialPortName = m_strComPortsIDs.at(ui->cbCommport->currentIndex());
    }

    m_bPortOpen = m_CommPort->open(serialPortName.toLocal8Bit().data(), serialPortBaudRate, bFlow);

    if (m_bPortOpen)
    {
        Log("Opened %s at speed: %u flow %s", serialPortName.toStdString().c_str(), serialPortBaudRate,
                ui->btnFlowCntrl->isChecked() ? "on":"off");

        // on opening the port, set a 2 sec timer
        QTimer::singleShot(2000, this, SLOT(startUpTimer()));
    }
    else
    {
        Log("Error opening serial port %s: Error number %d", serialPortName.toStdString().c_str(),
            (int)m_CommPort->errorNum());//p_qt_serial_port->error());
    }



    return m_bPortOpen;

}

// After timer ticks on opening the port, set visibilit/pairing mode of embedded device
// and query supported features (using GetVersion)
void MainWindow::startUpTimer()
{
    qDebug("startup timer");
    setVis();
    setPairingMode();
    GetVersion();
    getCurrentConnection();
    if (m_audio_connected) {
        //Log("ALREADY HAS DEVICE PAIRED \n");
        UINT8* address = connected_device->m_address;
        Log("ALREADY HAS DEVICE PAIRED, device address is");
        Log("%02X:%02X:%02X:%02X:%02X:%02X \n", address[0], address[1], address[2], address[3], address[4], address[5]);
    } else {
        Log("NO DEVICE PAIRED \n");
    }
}

void MainWindow::getCurrentConnection()
{
    Log("getting current connection\n");
    SendWicedCommand(HCI_CONTROL_COMMAND_GET_CONNECTION_INFO, NULL, 0);
}

int MainWindow::FindBaudRateIndex(int baud)
{
    int index = 0;

    for (; index < (int) (sizeof(as32BaudRate) / sizeof(as32BaudRate[0])); index++)
        if (as32BaudRate[index] == baud)
            return index;

    return 0;
}


// Close port
void MainWindow::CloseCommPort()
{
    if (m_port_read_worker)
    {
        m_bClosing = true;
        if (m_CommPort && m_CommPort->isOpen())
            m_CommPort->indicate_close();

        serial_read_wait.wakeAll();

        if(m_port_read_thread)
        {
            m_port_read_thread->exit();
            if (!m_port_read_thread->wait(2000))
            {
                m_port_read_thread->terminate();
                m_port_read_thread->wait(1000);
            }
        }
    }

    if (m_CommPort && m_CommPort->isOpen())
        m_CommPort->close();

    m_bPortOpen = false;
}


// When the UI is closed, close port and save settings
void MainWindow::closeEventDm (QCloseEvent *)
{
    CloseCommPort();

    // save settings for baudrate, serial port and flow-ctrl
    QSettings settings(m_SettingsFile, QSettings::IniFormat);
    int baud_rate = ui->cbBaudRate->currentText().toInt();
    m_settings.setValue("Baud",baud_rate);

    bool flow_control = ui->btnFlowCntrl->isChecked();
    m_settings.setValue("FlowControl",flow_control);

    QString comm_port = ui->cbCommport->currentText();
    m_settings.setValue("port",comm_port);
}

// Clear port and UI
void MainWindow::ClearPort()
{
    CloseCommPort();
    QThread::sleep(1);
    if(m_CommPort)
        delete m_CommPort;
    m_CommPort = NULL;

    Log("Serial port closed.");
    if(m_bUIEnabled)
        Log("Client Control app disconnected from Bluetooth device.");

    EnableUI(false);
    ui->cbCommport->setEnabled(true);
    ui->cbBaudRate->setEnabled(true);
    ui->btnFlowCntrl->setEnabled(true);
    ui->btnOpenPort->setText("Open Port");
}



void MainWindow::serialPortError(QSerialPort::SerialPortError error)
{
    //Print error etc.
    qDebug("serialPortError %d", error);
}

void MainWindow::handleReadyRead()
{
    m_CommPort->handleReadyRead();
}

/****************** Serial port thread **************/

// Create a thread the read serial port
void MainWindow::CreateReadPortThread()
{
    m_port_read_thread = new QThread;
    m_port_read_worker = new Worker();
    m_port_read_worker->moveToThread(m_port_read_thread);
    m_bClosing = false;
    m_port_read_worker->m_pParent = this;


    connect(m_port_read_thread, SIGNAL(started()), m_port_read_worker, SLOT(read_serial_port_thread()));
    connect(m_port_read_worker, SIGNAL(finished()), m_port_read_thread, SLOT(quit()));
    connect(m_port_read_worker, SIGNAL(finished()), m_port_read_worker, SLOT(deleteLater()));
    connect(m_port_read_thread, SIGNAL(finished()), m_port_read_thread, SLOT(deleteLater()));
}

// Serial port read thread
void Worker::read_serial_port_thread()
{
    unsigned char au8Hdr[1024 + 6];
    memset(au8Hdr, 0, 1030);
    int           offset = 0, pktLen;
    int           packetType;

    // While the port is not closed, keep reading
    while (!m_bClosing)
    {
        memset(au8Hdr, 0, 1030);
        offset = 0;
        // Read HCI packet
        pktLen = ReadNewHciPacket(au8Hdr, sizeof(au8Hdr), &offset);
        if (m_bClosing || pktLen < 0) // skip this
            break;

        if (pktLen + offset == 0)
            continue;

        packetType = au8Hdr[0];
        if (HCI_WICED_PKT == packetType)
        {
            DWORD channel_id = au8Hdr[1] | (au8Hdr[2] << 8);
            DWORD len = au8Hdr[3] | (au8Hdr[4] << 8);

            if (len > 1024)
            {
                m_pParent->Log("Skip bad packet %d", len);
                qDebug("Skip bad packet %d", (int) len);
                continue;
            }

            // If we are streaming audio, call the method to request audio data request
            // directly instead of posting on main thread. Posting on main thread
            // can cause audio glitches because of other UI activity.
            if(m_pParent->m_audio_started && (HCI_CONTROL_AUDIO_EVENT_REQUEST_DATA == channel_id))
            {
                if (m_pParent->m_uAudio.m_pWavData != NULL)
                {
                    m_pParent->HandleA2DPAudioRequestEvent(&au8Hdr[5], len);
                }
            }
            else
            {
                BYTE * pBuf = NULL;

                if(len)
                {
                    // malloc and create a copy of the data.
                    //  MainWindow::onHandleWicedEvent deletes the data
                    pBuf = (BYTE*)malloc(len);
                    memcpy(pBuf, &au8Hdr[5], len);
                }

                // send it to main thread
                emit m_pParent->HandleWicedEvent(channel_id, len, pBuf);
            }
        }
    }

    // When the thread exits, the m_port_read_thread is auto deleted, set the pointer to NULL
    m_pParent->m_port_read_thread = NULL;
    emit finished();
}

// Read HCI packet
DWORD Worker::ReadNewHciPacket(BYTE * pu8Buffer, int bufLen, int * pOffset)
{
    DWORD dwLen, len = 0, offset = 0;

    dwLen = Read(&pu8Buffer[offset], 1);

    if ((int)dwLen <= 0 || m_bClosing)
        return (-1);

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
            m_pParent->Log("HCI_EVENT_PKT len %d", len);
        }
        else
            m_pParent->Log("error HCI_EVENT_PKT, needed 2 got %d", dwLen);        
    }
        break;

    case HCI_ACL_DATA_PKT:
    {
        dwLen = Read(&pu8Buffer[offset], 4);
        if(dwLen == 4)
        {
            len = pu8Buffer[3] | (pu8Buffer[4] << 8);
            offset += 4;
            m_pParent->Log("HCI_ACL_DATA_PKT, len %d", len);
        }
        else
            m_pParent->Log("error HCI_ACL_DATA_PKT needed 4 got %d", dwLen);        
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
            m_pParent->Log("error HCI_WICED_PKT,  needed 4 got %d", dwLen);
    }
        break;
    default:
    {
        if(m_pParent->m_bUIEnabled)
        {
            qDebug("error unknown packet, type %d", pu8Buffer[0]);
            //m_pParent->Log("error unknown packet, type %d", pu8Buffer[0]);
        }
    }
    }

    if(len > 1024)
    {
        qDebug("bad packet %d", (int) len);
        m_pParent->Log("bad packet %d", len);
        return -1; // bad packet
    }

    if (len)
    {
        DWORD lenRd = m_pParent->qtmin(len, (DWORD)(bufLen-offset));
        dwLen = Read(&pu8Buffer[offset], lenRd);
        if(dwLen != lenRd)
            m_pParent->Log("read error to read %d, read %d", lenRd, dwLen);
    }

    *pOffset = offset;

    return len;
}

// Read from serial port
DWORD Worker::Read(BYTE *lpBytes, DWORD dwLen)
{
    BYTE * p = lpBytes;
    DWORD Length = dwLen;
    DWORD dwRead = 0;
    DWORD dwTotalRead = 0;

    if(!m_pParent->m_CommPort)
        return 0;

    QMutex mutex;
    int retry_cnt = 0;
    // Loop here until request is fulfilled or port is closed
    while (Length && !m_bClosing)
    {
        if(m_bClosing)
            return 0;

        dwRead = 0;
        char buff_temp[1030];
        memset(buff_temp, 0, 1030);

        dwRead = m_pParent->m_CommPort->read(buff_temp,Length);
        if((int)dwRead <= 0)
        {
            if(m_bClosing)
                return 0;

            if((int)dwRead < 0)
            {
                m_pParent->Log("Error in port read");
                return -1;
            }

            // retry 3 time with longer timeout for each subsequent
            unsigned long ulTimeout = 20;
            if(retry_cnt < 3)
                {
                    retry_cnt++;
                    ulTimeout *= retry_cnt;
                }
                else
                    ulTimeout = ULONG_MAX;

            //qDebug("serial_read wait Length %d, timeout %d, retry %d", Length, ulTimeout, retry_cnt);
            // If dwRead == 0, then we need to wait for device to send the MCU app data
            mutex.lock();
             //serial_read_wait is set when there is more data or when the serial port is closed
            m_pParent->serial_read_wait.wait(&mutex, ulTimeout);
            mutex.unlock();
            //qDebug("serial_read continue");
        }
        else
        {
            memcpy(p, buff_temp, dwRead);
            retry_cnt = 0;
        }

        if (dwRead > Length)
            break;
        p += dwRead;
        Length -= dwRead;
        dwTotalRead += dwRead;
    }

    return dwTotalRead;
}

