cmake_minimum_required(VERSION 3.5)
project(rtab_map_ros2_docker)

# Найдем зависимости
find_package(ament_cmake REQUIRED)

# Установка файлов запуска
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Файлы конфигурации
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY scripts/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

# Упаковка проекта
ament_package()
