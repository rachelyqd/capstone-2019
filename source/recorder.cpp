#include "app_include.h"
extern "C"
{
#include "app_host.h"
}

#include <QAudioRecorder>
#include <QAudioProbe>
#include <QDir>
#include <QFileDialog>
#include <QMediaRecorder>
#include <QStandardPaths>

const QString fileDirectory = "C:/Users/Philip/Desktop/file.wav";

void MainWindow::InitRecorder()
{
    m_audioRecorder = new QAudioRecorder(this);

    m_audioRecorder->setOutputLocation(QUrl::fromLocalFile(fileDirectory));
    m_settings.setValue("AudioFile",fileDirectory);

    hasSpoken = false;

    //ui->audioDeviceBox->addItem(tr("Default"), QVariant(QString()));
    for (auto &device: m_audioRecorder->audioInputs()) {
        ui->audioDeviceBox->addItem(device, QVariant(device));
    }

    connect(m_audioRecorder, &QAudioRecorder::statusChanged, this, &MainWindow::updateStatus);
    connect(m_audioRecorder, &QAudioRecorder::stateChanged, this, &MainWindow::onStateChanged);
    connect(m_audioRecorder, QOverload<QMediaRecorder::Error>::of(&QAudioRecorder::error), this,
            &MainWindow::displayErrorMessage);
}

void MainWindow::updateStatus(QMediaRecorder::Status status)
{
    QString statusMessage;
    if (m_audioRecorder->error() == QMediaRecorder::NoError)
        ui->statusLine->setText(statusMessage);
}

void MainWindow::onStateChanged(QMediaRecorder::State state)
{
    QFile file;
    QFileInfo check_file(fileDirectory);
    switch (state) {
    case QMediaRecorder::RecordingState:
        ui->recordButton->setText(tr("Stop Speaking"));
        break;
    case QMediaRecorder::StoppedState:
        ui->recordButton->setText(tr("Start Speaking"));
        if (check_file.exists()) {
            file.setFileName(fileDirectory);
            file.remove();
        }
        m_audioRecorder->setOutputLocation(QUrl::fromLocalFile("/Users/rachely/Desktop/input.wav"));
        hasSpoken = true;
        break;
    case QMediaRecorder::PausedState:
        break;
    }
}

static QVariant boxValue(const QComboBox *box)
{
    int idx = box->currentIndex();
    if (idx == -1)
        return QVariant();

    return box->itemData(idx);
}

void MainWindow::displayErrorMessage()
{
    //ui->statusLine->setText(m_audioRecorder->errorString());
}


void MainWindow::on_recordButton_clicked()
{
    if (m_audioRecorder->state() == QMediaRecorder::StoppedState) {
        m_audioRecorder->setAudioInput(boxValue(ui->audioDeviceBox).toString());

        QAudioEncoderSettings settings;
        settings.setCodec("audio/pcm");
        settings.setSampleRate(96000);
        settings.setBitRate(96000);
        settings.setChannelCount(1);
        settings.setQuality(QMultimedia::HighQuality);
        settings.setEncodingMode(QMultimedia::ConstantQualityEncoding);

        QString container = "audio/x-wav";

        m_audioRecorder->setEncodingSettings(settings, QVideoEncoderSettings(), container);
        m_audioRecorder->record();
    }
    else {
        m_audioRecorder->stop();
    }
}
