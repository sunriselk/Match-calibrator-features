#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_FeatureMatching.h"

class FeatureMatching : public QMainWindow
{
    Q_OBJECT

public:
    FeatureMatching(QWidget *parent = Q_NULLPTR);

private:
    Ui::FeatureMatchingClass ui;
    QString m_strImageDir;
    QString m_strResultDir;


protected slots:

    void OnImageDirection();
    void OnResultDirection();
    void OnStartMatching();
    
};
