#pragma once

#include <QMainWindow>
#include <QPushButton>
#include <QWebEngineView>
#include <memory>
#include "drone_controller.hpp"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(std::shared_ptr<DroneController> controller);

private slots:
    void onTakeoffClicked();
    void onLandClicked();
    void onArmClicked();
    void onDisarmClicked();
    void updateMap();

private:
    QPushButton *m_takeoff_button;
    QPushButton *m_land_button;
    QPushButton *m_arm_button;
    QPushButton *m_disarm_button;
    QWebEngineView *m_map_view;
    QTimer *m_position_timer;

    std::shared_ptr<DroneController> m_drone_controller;
};
