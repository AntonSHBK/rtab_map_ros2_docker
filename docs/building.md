### Инструкция по запуску и сборке проекта RTAB-Map в Docker

#### 1. **Подготовка системы**
1. **Убедитесь, что Docker установлен:**
   - На **Windows**:
     Скачайте [Docker Desktop](https://www.docker.com/products/docker-desktop/) и установите его.
   - На **Ubuntu**:
     Установите Docker и Docker Compose:
     ```bash
     sudo apt update
     sudo apt install docker.io docker-compose
     ```

2. **Проверьте установку Docker:**
   ```bash
   docker --version
   docker-compose --version
   ```

3. **Дополнительно для Ubuntu:**
   Разрешите доступ контейнеров к X-серверу:
   ```bash
   xhost +local:docker
   ```

---

#### 2. **Создание проекта**
1. **Создайте рабочую директорию:**
   ```bash
   mkdir rtabmap_project
   cd rtabmap_project
   ```

2. **Создайте необходимые файлы:**
   - **Dockerfile:**
     Сохраните содержимое из секции [Dockerfile](#dockerfile) в файл `Dockerfile`.
   - **docker-compose.yml:**
     Сохраните содержимое из секции [docker-compose.yml](#docker-composeyml) в файл `docker-compose.yml`.
   - **.env:**
     Сохраните содержимое из секции [.env](#env) в файл `.env`.

---

#### 3. **Редактирование файла `.env`**
В файле `.env` настройте параметр `DISPLAY` в зависимости от вашей операционной системы:
- Для Windows:
  ```env
  DISPLAY=host.docker.internal:0.0
  ```
- Для Ubuntu:
  ```env
  DISPLAY=:0
  ```

---

#### 4. **Сборка Docker-контейнера**
1. Выполните сборку контейнера:
   ```bash
   docker-compose up --build
   ```
   Это команда:
   - Соберёт Docker-образ из `Dockerfile`.
   - Запустит контейнер с RTAB-Map.

2. Убедитесь, что контейнер запущен без ошибок.

---

#### 5. **Проверка подключения камеры**
Внутри контейнера проверьте подключение камеры:
```bash
v4l2-ctl --list-devices
```
Вы должны увидеть список доступных устройств. Обычно камера подключается как `/dev/video0`.

---

#### 6. **Запуск RTAB-Map**
1. Внутри контейнера выполните команду для запуска RTAB-Map:
   ```bash
   ros2 launch rtabmap_ros rtabmap.launch.py \
     rgb_topic:=/camera/image_raw \
     frame_id:=camera_link
   ```

2. Если используется USB-камера без глубины, добавьте параметры:
   ```bash
   subscribe_depth:=false subscribe_rgbd:=false
   ```

---

#### 7. **Визуализация через RViz**
1. Запустите RViz:
   ```bash
   rviz2
   ```
2. Настройте отображение следующих топиков:
   - `/rtabmap/mapData`: Карта.
   - `/rtabmap/odom`: Одометрия.
   - `/rtabmap/cloud`: Облако точек.

---

#### 8. **Остановка контейнера**
Чтобы остановить контейнер:
```bash
docker-compose down
```

---

### Примечания:
1. **Windows:**
   - Убедитесь, что **xLaunch** запущен с параметром "Disable access control".

2. **Ubuntu:**
   - Перед запуском разрешите доступ контейнеров к X-серверу:
     ```bash
     xhost +local:docker
     ```

3. **Отладка:**
   Если RTAB-Map не запускается, проверьте:
   - Значение `DISPLAY`:
     ```bash
     echo $DISPLAY
     ```
   - Доступность камеры:
     ```bash
     v4l2-ctl --list-devices
     ```

Теперь вы готовы использовать RTAB-Map в Docker на любой платформе!