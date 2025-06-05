#ifndef GUIPANEL_H
#define GUIPANEL_H

#include <QWidget>
#include <QtSerialPort/qserialport.h>
#include "qmqtt.h"

#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>

#define NMAX 300

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

    void on_pushButton_5_clicked(void);

    void on_pushButton_6_clicked(void);

    void on_checkBox_toggled(bool checked);

    void on_Knob_valueChanged(double value);

    void on_Knob_2_valueChanged(double value);

    void on_Knob_3_valueChanged(double value);

private: // funciones privadas
//    void pingDevice();
    void startClient();
    void processError(const QString &s);
    void activateRunButton();
    void cambiaLEDs();
    void SendMessage_LED();
    void SendMessage_General(QJsonObject objeto_json);
private:
    Ui::GUIPanel *ui;
    int transactionCount;
    QMQTT::Client *_client;
    bool connected;
    bool pingRequest;
    bool updatingPWMControlInternally;
    QString suscribeRootTopic;
    QString publishRootTopic;

    double xVal[NMAX];
    double yVal1[NMAX];

    QwtPlotGrid  *m_Grid;
    QwtPlotCurve *m_curve_1;
};

#endif // GUIPANEL_H
