# Базовый образ ROS 2 Humble
FROM osrf/ros:humble-desktop

# **************************************************************************** #

# Установка необходимых пакетов
RUN apt-get update && apt-get install -y \
    ros-humble-rtabmap-ros \
    ros-humble-rviz2 \
    ros-humble-image-transport \
    v4l-utils \
    nano vim \
    && rm -rf /var/lib/apt/lists/*

# **************************************************************************** #

ENV PROJECT_NAME rtab_map_ros2_docker

# set work directory
ENV WORKSPACE /workspace/src
RUN mkdir -p $WORKSPACE/$PROJECT_NAME
WORKDIR $WORKSPACE

# copy project
COPY . $WORKSPACE/$PROJECT_NAME

# **************************************************************************** #

# Install Python 3 and pip
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# install python dependencies
COPY ../requirements.txt /tmp/requirements.txt

RUN pip install --upgrade pip && \
    pip install -r /tmp/requirements.txt  && \
    rm -rf /root/.cache/pip

# **************************************************************************** #

# Устанавливаем переменные окружения для x11
ENV QT_X11_NO_MITSHM=1

# Команда запуска по умолчанию
CMD ["bash"]
