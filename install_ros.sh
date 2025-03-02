#!/bin/bash

# Habilitar modo estricto para errores
set -e

echo "🔹 Verificando e instalando lsb-release si es necesario..."
if ! command -v lsb_release &> /dev/null; then
    echo "🔹 Instalando lsb-release..."
    sudo apt update
    sudo apt install -y lsb-release
fi

echo "🔹 Configurando el repositorio de ROS..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

echo "🔹 Agregando la clave del servidor de ROS..."
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

echo "🔹 Actualizando índices de paquetes..."
sudo apt update

echo "🔹 Instalando ROS Noetic Desktop Full..."
sudo apt install -y ros-noetic-desktop-full

echo "🔹 Inicializando rosdep..."
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update

echo "🔹 Configurando variables de entorno..."
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

echo "🔹 Verificando configuración de ROS..."
export | grep ROS

echo "🔹 Instalando herramientas adicionales..."
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

echo "🔹 Creando y compilando workspace de ROS..."
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash

echo "🔹 Instalando paquetes adicionales..."
sudo apt install -y python-is-python3 ros-noetic-tf2-tools ros-noetic-robot-controllers ros-noetic-joy ros-noetic-ros-control \
    ros-noetic-ros-controllers ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control python3-wstool \
    ros-noetic-scan-tools ros-noetic-geographic-msgs ros-noetic-turtlebot3-simulations ros-noetic-turtlebot3-navigation \
    ros-noetic-csm ros-noetic-rviz-imu-plugin ros-noetic-slam-gmapping ros-noetic-map-server ros-noetic-navigation \
    ros-noetic-rtabmap-ros ros-noetic-dwa-local-planner libignition-math4-dev

echo "🔹 Moviendo el archivo MobileRoboticsCourse-ROSPackages.zip a ~/catkin_ws/src/"
mv ~/MobileRoboticsCourse-ROSPackages.zip ~/catkin_ws/src/

echo "🔹 Descomprimiendo MobileRoboticsCourse-ROSPackages.zip..."
cd ~/catkin_ws/src/
unzip MobileRoboticsCourse-ROSPackages.zip
rm MobileRoboticsCourse-ROSPackages.zip

echo "🔹 Eliminando carpeta MobileRoboticsCourse/worlds_tc..."
rm -rf ~/catkin_ws/src/MobileRoboticsCourse/worlds_tc

echo "🔹 Haciendo todos los scripts .py en src ejecutables..."
find ~/catkin_ws/src/ -type f -name "*.py" -exec chmod +x {} \;

echo "🔹 Ejecutando catkin_make en ~/catkin_ws..."
cd ~/catkin_ws
catkin_make

echo "✅ Instalación y configuración completadas con éxito."
