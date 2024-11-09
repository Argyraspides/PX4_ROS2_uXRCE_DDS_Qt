#include "main_window.hpp"
#include <QVBoxLayout>
#include <QTimer>
#include <iostream>

MainWindow::MainWindow(std::shared_ptr<DroneController> controller) : m_drone_controller(controller)
{
    setWindowTitle("Drone Control");
    setGeometry(100, 100, 800, 600);

    QWidget *central_widget = new QWidget();
    setCentralWidget(central_widget);
    QVBoxLayout *layout = new QVBoxLayout(central_widget);

    // Map view
    m_map_view = new QWebEngineView();
    m_map_view->setUrl(QUrl("https://www.openstreetmap.org"));
    layout->addWidget(m_map_view);

    // Buttons
    m_takeoff_button = new QPushButton("Take Off");
    m_land_button = new QPushButton("Land");
    m_arm_button = new QPushButton("Arm");
    m_disarm_button = new QPushButton("Disarm");

    layout->addWidget(m_takeoff_button);
    layout->addWidget(m_land_button);
    layout->addWidget(m_arm_button);
    layout->addWidget(m_disarm_button);

    // Connect signals
    connect(m_takeoff_button, &QPushButton::clicked, this, &MainWindow::onTakeoffClicked);
    connect(m_land_button, &QPushButton::clicked, this, &MainWindow::onLandClicked);
    connect(m_arm_button, &QPushButton::clicked, this, &MainWindow::onArmClicked);
    connect(m_disarm_button, &QPushButton::clicked, this, &MainWindow::onDisarmClicked);

    // Timer for position updates
    m_position_timer = new QTimer(this);
    connect(m_position_timer, &QTimer::timeout, this, &MainWindow::updateMap);
    m_position_timer->start(3000);
}

void MainWindow::onTakeoffClicked()
{
    m_drone_controller->takeoff();
}
void MainWindow::onLandClicked()
{
    m_drone_controller->land();
}

void MainWindow::onArmClicked()
{
    m_drone_controller->arm_disarm(1.0f);
}

void MainWindow::onDisarmClicked()
{
    m_drone_controller->arm_disarm(0.0f);
}

void MainWindow::updateMap()
{
    QString lonStr = QString::fromStdString(std::to_string(m_drone_controller->get_current_lon()));
    QString latStr = QString::fromStdString(std::to_string(m_drone_controller->get_current_lat()));
    QString finalURL = QString("https://www.openstreetmap.org/#map=8/") + latStr + "/" + lonStr;

    m_map_view->setUrl(QUrl(finalURL));
}
