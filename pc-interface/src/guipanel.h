#ifndef GUIPANEL_H
#define GUIPANEL_H

#include <QWidget>
#include <QtSerialPort/qserialport.h>
#include "qmqtt.h"

namespace Ui {
class GUIPanel;
}

//QT4:QT_USE_NAMESPACE_SERIALPORT

class GUIPanel : public QWidget
{
    Q_OBJECT
    
public:
    //GUIPanel(QWidget *parent = 0);
    explicit GUIPanel(QWidget *parent = 0);
    ~GUIPanel(); // Da problemas
    
private slots:

    void on_runButton_clicked();
    void on_pushButton_clicked();

    void onMQTT_Received(const QMQTT::Message &message);
    void onMQTT_Connected(void);

    void onMQTT_subscribed(const QString &topic);


    void on_pushButton_2_toggled(bool checked);

    void on_pushButton_4_toggled(bool checked);

    void on_pushButton_3_toggled(bool checked);

private: // funciones privadas
//    void pingDevice();
    void startClient();
    void processError(const QString &s);
    void activateRunButton();
    void cambiaLEDs();
    void SendMessage();
private:
    Ui::GUIPanel *ui;
    int transactionCount;
    QMQTT::Client *_client;
    bool connected;
};

#endif // GUIPANEL_H
