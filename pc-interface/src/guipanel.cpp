#include "guipanel.h"
#include "ui_guipanel.h"
#include <QSerialPort>      // Comunicacion por el puerto serie
#include <QSerialPortInfo>  // Comunicacion por el puerto serie
#include <QMessageBox>      // Se deben incluir cabeceras a los componentes que se vayan a crear en la clase

#include <QJsonObject>
#include <QJsonDocument>

// y que no estén incluidos en el interfaz gráfico. En este caso, la ventana de PopUp <QMessageBox>
// que se muestra al recibir un PING de respuesta

#include<stdint.h>      // Cabecera para usar tipos de enteros con tamaño
#include<stdbool.h>     // Cabecera para usar booleanos


GUIPanel::GUIPanel(QWidget *parent) :  // Constructor de la clase
    QWidget(parent),
    ui(new Ui::GUIPanel)               // Indica que guipanel.ui es el interfaz grafico de la clase
  , transactionCount(0)
{
    ui->setupUi(this);                // Conecta la clase con su interfaz gráfico.
    setWindowTitle(tr("Interfaz de Control")); // Título de la ventana

    _client=new QMQTT::Client(QHostAddress::LocalHost, 1883); //localhost y lo otro son valores por defecto


    connect(_client, SIGNAL(connected()), this, SLOT(onMQTT_Connected()));    
    connect(_client, SIGNAL(received(const QMQTT::Message &)), this, SLOT(onMQTT_Received(const QMQTT::Message &)));
    connect(_client, SIGNAL(subscribed(const QString &)), this, SLOT(onMQTT_subscribed(const QString &)));

    connected=false;                 // Todavía no hemos establecido la conexión con el servidor MQTT
    pingRequest = false;             // No se ha hecho solicitud de PING.
    updatingPWMControlInternally = false; // flag de actualización de controles de LED internos.

    // Se oculta el control PWM de los LED en el arranque
    ui->Knob->setHidden(true);
    ui->Knob_2->setHidden(true);
    ui->Knob_3->setHidden(true);

    ui->label_5->setHidden(true);
    ui->label_6->setHidden(true);
    ui->label_7->setHidden(true);

    //Configuramos la grafica
    ui->qwtPlot->setTitle("Voltímetro");
        ui->qwtPlot->setAxisTitle(QwtPlot::xBottom,"número de muestra");
        ui->qwtPlot->setAxisTitle(QwtPlot::yLeft, "Volt");
    //ui->qwtPlot->axisAutoScale(true); // Con Autoescala
    ui->qwtPlot->setAxisScale(QwtPlot::yLeft, 0, ADC_MAXVOLTAGE); // Con escala definida
    ui->qwtPlot->setAutoReplot(false);

    //Creamos una curva y la añadimos a la grafica
    m_curve_1 = new QwtPlotCurve();
    m_curve_1->setPen(QPen(Qt::red));
    m_Grid = new QwtPlotGrid();
    m_Grid->attach(ui->qwtPlot);
    m_curve_1->attach(ui->qwtPlot);

    //Inicializadmos los datos que se muestran en la grafica
    for (int i=0; i<NMAX; i++) {
        yVal1[i]=0;
        xVal[i]=i;
    }
    m_curve_1->setRawSamples(xVal,yVal1,NMAX);
    ui->qwtPlot->replot();
}

GUIPanel::~GUIPanel() // Destructor de la clase
{
    delete ui;   // Borra el interfaz gráfico asociado a la clase
}


// Establecimiento de la comunicación USB serie a través del interfaz seleccionado en la comboBox, tras pulsar el
// botón RUN del interfaz gráfico. Se establece una comunicacion a 9600bps 8N1 y sin control de flujo en el objeto
// 'serial' que es el que gestiona la comunicación USB serie en el interfaz QT
void GUIPanel::startClient()
{
    _client->setHostName(ui->leHost->text());
    _client->setPort(1883);
    _client->setKeepAlive(300);
    _client->setCleanSession(true);
    _client->connectToHost();

}

// Funcion auxiliar de procesamiento de errores de comunicación (usada por startSlave)
void GUIPanel::processError(const QString &s)
{
    activateRunButton(); // Activa el botón RUN
    // Muestra en la etiqueta de estado la razón del error (notese la forma de pasar argumentos a la cadena de texto)
    ui->statusLabel->setText(tr("Status: Not running, %1.").arg(s));
}

// Funcion de habilitacion del boton de inicio/conexion
void GUIPanel::activateRunButton()
{
    ui->runButton->setEnabled(true);
}

// SLOT asociada a pulsación del botón RUN
void GUIPanel::on_runButton_clicked()
{
    startClient();
}


void GUIPanel::onMQTT_subscribed(const QString &topic)
{
     ui->statusLabel->setText(tr("subscribed %1").arg(topic));
}


void GUIPanel::on_pushButton_clicked()
{
    ui->statusLabel->setText(tr(""));
}


void GUIPanel::onMQTT_Received(const QMQTT::Message &message)
{
    bool previousblockinstate,checked;
    int  knobValue;
    if (connected)
    {
        QJsonParseError error;
        QJsonDocument mensaje=QJsonDocument::fromJson(message.payload(),&error);
        QString topic = message.topic();

        if ((error.error==QJsonParseError::NoError)&&(mensaje.isObject()))
        { //Tengo que comprobar que el mensaje es del tipo adecuado y no hay errores de parseo...

            QJsonObject objeto_json=mensaje.object();

            if ((topic == (publishRootTopic + "/pong")) && (pingRequest == true))
            {
                pingRequest = false;
                QMessageBox ventanaPopUp(QMessageBox::Information,tr("Evento"),tr("RESPUESTA A PING RECIBIDA"),QMessageBox::Ok,this,Qt::Popup);
                ventanaPopUp.exec();
            }
            else if (topic == (publishRootTopic + "/button_poll"))
            {
                QJsonValue entrada=objeto_json["button1"];
                QJsonValue entrada2=objeto_json["button2"];

                if (entrada.isBool())
                {
                    ui->led->setState(entrada.toBool());
                }

                if(entrada2.isBool())
                {
                    ui->led_2->setState(entrada2.toBool());
                }
            }
            else if (topic == (publishRootTopic + "/adc_read"))
            {
                QJsonValue entrada = objeto_json["adc_read"];

                if (entrada.isDouble())
                {
                    //Actualizamos gráfica correspondiente
                    memmove(yVal1,yVal1+1,sizeof(double)*(NMAX-1)); //Desplazamos las muestras hacia la izquierda
                    yVal1[NMAX-1]=ADC_MAXVOLTAGE*entrada.toDouble()/ADC_RESOLUTION; //Añadimos el último punto
                    m_curve_1->setRawSamples(xVal,yVal1,NMAX);  //Refrescamos..
                    ui->qwtPlot->replot();
                }
            }
            else if (topic == (publishRootTopic + "/last_will"))
            {
                QJsonValue entrada = objeto_json["disconnected"];

                if (entrada.isBool())
                {
                    if (entrada.toBool())
                    {
                        ui->statusLabel_2->setText("Desconectado");
                    }
                    else
                    {
                        ui->statusLabel_2->setText("Conectado");
                    }
                }
            }
            else
            {
                QStringList keys = objeto_json.keys();

                for (int i = 0; i < keys.count(); i++)
                {
                    QJsonValue entrada = objeto_json[keys[i]];

                    if ((keys[i] == "redLed") && (entrada.isBool()))
                    {
                        checked=entrada.toBool();
                        previousblockinstate=ui->pushButton_2->blockSignals(true);

                        ui->pushButton_2->setChecked(checked);
                        if (checked)
                        {
                            ui->pushButton_2->setText("Apaga");

                        }
                        else
                        {
                            ui->pushButton_2->setText("Enciende");
                        }
                        ui->pushButton_2->blockSignals(previousblockinstate);
                    }
                    else if ((keys[i] == "greenLed") && (entrada.isBool()))
                    {
                        checked=entrada.toBool();
                        previousblockinstate=ui->pushButton_3->blockSignals(true);

                        ui->pushButton_3->setChecked(checked);
                        if (checked)
                        {
                            ui->pushButton_3->setText("Apaga");

                        }
                        else
                        {
                            ui->pushButton_3->setText("Enciende");
                        }
                        ui->pushButton_3->blockSignals(previousblockinstate);
                    }
                    else if ((keys[i] == "blueLed") && (entrada.isBool()))
                    {
                        checked=entrada.toBool();
                        previousblockinstate=ui->pushButton_4->blockSignals(true);

                        ui->pushButton_4->setChecked(checked);
                        if (checked)
                        {
                            ui->pushButton_4->setText("Apaga");

                        }
                        else
                        {
                            ui->pushButton_4->setText("Enciende");
                        }
                        ui->pushButton_4->blockSignals(previousblockinstate);
                    }
                    else if ((keys[i] == "PWM_Rojo") && (entrada.isDouble()))
                    {
                        knobValue = entrada.toInt();
                        previousblockinstate=ui->Knob->blockSignals(true);

                        ui->Knob->setValue(int(((double(knobValue) / 255.0) * 100)));

                        ui->Knob->blockSignals(previousblockinstate);
                    }
                    else if ((keys[i] == "PWM_Verde") && (entrada.isDouble()))
                    {
                        knobValue = entrada.toInt();
                        previousblockinstate=ui->Knob_2->blockSignals(true);

                        ui->Knob_2->setValue(int(((double(knobValue) / 255.0) * 100)));

                        ui->Knob_2->blockSignals(previousblockinstate);
                    }
                    else if ((keys[i] == "PWM_Azul") && (entrada.isDouble()))
                    {
                        knobValue = entrada.toInt();
                        previousblockinstate=ui->Knob_3->blockSignals(true);

                        ui->Knob_3->setValue(int(((double(knobValue) / 255.0) * 100)));

                        ui->Knob_3->blockSignals(previousblockinstate);
                    }
                    else if ((keys[i] == "PWM_mode") && (entrada.isBool()))
                    {
                        checked = entrada.toBool();
                        updatingPWMControlInternally = true;

                        ui->checkBox->setChecked(checked);

                        updatingPWMControlInternally = false;
                    }
                    else
                    {
                        // do nothing
                    }

                }
            }
        }


    }
}


/* -----------------------------------------------------------
 MQTT Client Slots
 -----------------------------------------------------------*/
void GUIPanel::onMQTT_Connected()
{
    suscribeRootTopic = ui->suscribe_topic->text();
    publishRootTopic  = ui->publish_topic->text();


    ui->runButton->setEnabled(false);

    // Se indica que se ha realizado la conexión en la etiqueta 'statusLabel'
    ui->statusLabel->setText(tr("Ejecucion, conectado al servidor"));

    connected=true;

    _client->subscribe(suscribeRootTopic,0);       //Se suscribe al mismo topic en el que publica
    _client->subscribe(publishRootTopic + "/#",0); //Se suscribe a los hijos del topic de recepción padre y sus descendientes
}

void GUIPanel::SendMessage_General(QJsonObject objeto_json)
{
    QJsonDocument mensaje(objeto_json); //crea un objeto de tivo QJsonDocument conteniendo el objeto objeto_json (necesario para obtener el mensaje formateado en JSON)

    QMQTT::Message msg(0, suscribeRootTopic, mensaje.toJson()); //Crea el mensaje MQTT contieniendo el mensaje en formato JSON

    //Publica el mensaje
    _client->publish(msg);
}

void GUIPanel::SendMessage_LED()
{

    QByteArray cadena;


    QJsonObject objeto_json;
    //Añade un campo "redLed" al objeto JSON, con el valor (true o false) contenido en checked
    objeto_json["redLed"]=ui->pushButton_2->isChecked(); //Puedo hacer ["redLed"] porque el operador [] está sobrecargado.
    //Añade un campo "orangeLed" al objeto JSON, con el valor (true o false) contenido en checked
    objeto_json["greenLed"]=ui->pushButton_3->isChecked(); //Puedo hacer ["orangeLed"] porque el operador [] está sobrecargado.
    //Añade un campo "greenLed" al objeto JSON, con el valor (true o false) contenido en checked
    objeto_json["blueLed"]=ui->pushButton_4->isChecked();


    QJsonDocument mensaje(objeto_json); //crea un objeto de tivo QJsonDocument conteniendo el objeto objeto_json (necesario para obtener el mensaje formateado en JSON)

    QMQTT::Message msg(0, suscribeRootTopic, mensaje.toJson()); //Crea el mensaje MQTT contieniendo el mensaje en formato JSON

    //Publica el mensaje
    _client->publish(msg);

}

void GUIPanel::on_pushButton_2_toggled(bool checked)
{
    //Rojo
    if (checked)
    {
        ui->pushButton_2->setText("Apaga");

    }
    else
    {
        ui->pushButton_2->setText("Enciende");
    }
    SendMessage_LED();
}

void GUIPanel::on_pushButton_4_toggled(bool checked)
{
    //Verde
    if (checked)
    {
        ui->pushButton_4->setText("Apaga");

    }
    else
    {
        ui->pushButton_4->setText("Enciende");
    }
    SendMessage_LED();
}

void GUIPanel::on_pushButton_3_toggled(bool checked)
{
    //Azul
    if (checked)
    {
        ui->pushButton_3->setText("Apaga");

    }
    else
    {
        ui->pushButton_3->setText("Enciende");
    }
    SendMessage_LED();
}

void GUIPanel::on_pushButton_5_clicked(void)
{
    //Ping
    QJsonObject objeto_json;

    //Añade un campo "ping" al objeto JSON, con el valor true.
    objeto_json["ping"]=true;

    SendMessage_General(objeto_json);

    //Activamos el flag de petición de PING pendiente
    pingRequest = true;
}

void GUIPanel::on_pushButton_6_clicked(void)
{
    // Sondeo de botones

    QJsonObject objeto_json;

    //Añade un campo "button_poll" al objeto JSON, con el valor true.
    objeto_json["button_poll"]=true;

    SendMessage_General(objeto_json);
}

void GUIPanel::on_checkBox_toggled(bool checked)
{
    // Crear mensaje MQTT
    QJsonObject objeto_json;

    if (checked)
    {
        // Activar control PWM
        ui->Knob->setEnabled(true);
        ui->Knob_2->setEnabled(true);
        ui->Knob_3->setEnabled(true);

        ui->Knob->setHidden(false);
        ui->Knob_2->setHidden(false);
        ui->Knob_3->setHidden(false);

        ui->label_5->setHidden(false);
        ui->label_6->setHidden(false);
        ui->label_7->setHidden(false);

        // Desactivar control binario
        ui->pushButton_2->setEnabled(false);
        ui->pushButton_3->setEnabled(false);
        ui->pushButton_4->setEnabled(false);

        ui->pushButton_2->setHidden(true);
        ui->pushButton_3->setHidden(true);
        ui->pushButton_4->setHidden(true);

        // Cambiar texto de check box
        ui->checkBox->setText("Estado actual: Control PWM");

        //Añade un campo "PWM_mode" al objeto JSON, con el valor true.
        objeto_json["PWM_mode"]=true;
    }
    else
    {
        // Desactivar control PWM
        ui->Knob->setEnabled(false);
        ui->Knob_2->setEnabled(false);
        ui->Knob_3->setEnabled(false);

        ui->Knob->setHidden(true);
        ui->Knob_2->setHidden(true);
        ui->Knob_3->setHidden(true);

        ui->label_5->setHidden(true);
        ui->label_6->setHidden(true);
        ui->label_7->setHidden(true);

        // Activar control binario
        ui->pushButton_2->setEnabled(true);
        ui->pushButton_3->setEnabled(true);
        ui->pushButton_4->setEnabled(true);

        ui->pushButton_2->setHidden(false);
        ui->pushButton_3->setHidden(false);
        ui->pushButton_4->setHidden(false);

        // Cambiar texto de check box
        ui->checkBox->setText("Estado actual: Control ON/OFF");

        //Añade un campo "PWM_mode" al objeto JSON, con el valor false.
        objeto_json["PWM_mode"]=false;
    }

    if (updatingPWMControlInternally) // Para evitar que se envíe repetido un nuevo cambio en la checkbox cuando se está recibiendo el cambio por MQTT
        return;
    SendMessage_General(objeto_json);
}

void GUIPanel::on_Knob_valueChanged(double value)
{
    // Crear mensaje MQTT
    QJsonObject objeto_json;

    //Añade un campo "PWM_Rojo" al objeto JSON, con el valor del knob para el led ROJO.
    objeto_json["PWM_Rojo"]=int((value/100)*255);

    SendMessage_General(objeto_json);
}

void GUIPanel::on_Knob_2_valueChanged(double value)
{
    // Crear mensaje MQTT
    QJsonObject objeto_json;

    //Añade un campo "PWM_Verde" al objeto JSON, con el valor del knob para el led Verde.
    objeto_json["PWM_Verde"]=int((value/100)*255);

    SendMessage_General(objeto_json);
}

void GUIPanel::on_Knob_3_valueChanged(double value)
{
    // Crear mensaje MQTT
    QJsonObject objeto_json;

    //Añade un campo "PWM_Azul" al objeto JSON, con el valor del knob para el led Azul.
    objeto_json["PWM_Azul"]=int((value/100)*255);

    SendMessage_General(objeto_json);
}

void GUIPanel::on_checkBox_2_toggled(bool checked)
{
    // Crear mensaje MQTT
    QJsonObject objeto_json;

    if (checked)
    {
        ui->pushButton_6->setDisabled(true);
        ui->pushButton_6->setHidden(true);

        objeto_json["button_interrupt"] = true;
    }
    else
    {
        ui->pushButton_6->setEnabled(true);
        ui->pushButton_6->setVisible(true);

        objeto_json["button_interrupt"] = false;
    }

    SendMessage_General(objeto_json);
}
