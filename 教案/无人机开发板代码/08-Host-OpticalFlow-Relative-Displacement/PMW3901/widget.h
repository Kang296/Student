#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <SerialPortManager.h>
#include "FlowParser.h"

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();

private:
    Ui::Widget *ui;

    SerialPortManager * s;
    FlowParser * parser;
};

#endif // WIDGET_H
