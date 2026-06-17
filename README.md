# RMPC-project-KUKA-youBot

**Управление мобильным манипулятором KUKA youBot на ROS Noetic**

Этот проект представляет собой среду для разработки и тестирования алгоритмов управления для мобильного манипулятора KUKA youBot. Проект построен на ROS Noetic и использует готовые форки драйверов.

---

## Оглавление

- [О KUKA youBot](#о-kuka-youbot)
- [Архитектура проекта](#архитектура-проекта)
- [Предварительные требования](#предварительные-требования)
- [Инструкция по установке и настройке](#инструкция-по-установке-и-настройке)
  - [1. Установка ROS Noetic и WSL2 (для Windows)](#1-установка-ros-noetic-и-wsl2-для-windows)
  - [2. Клонирование репозитория](#2-клонирование-репозитория)
  - [3. Настройка конфигурационных файлов](#3-настройка-конфигурационных-файлов)
  - [4. Сборка проекта](#4-сборка-проекта)
- [Запуск и тестирование](#запуск-и-тестирование)
- [Устранение неполадок](#устранение-неполадок)
- [Полезные ссылки](#полезные-ссылки)

---

## О KUKA youBot

**KUKA youBot** — это мобильный манипулятор, разработанный для образования и научных исследований. Он состоит из двух основных частей:

1.  **Омни-направленная мобильная платформа**:
    - 4 колеса Mecanum для движения в любом направлении.
    - Размеры: 580 x 380 x 140 мм.
    - Вес: ~20 кг.
    - Максимальная скорость: 0.8 м/с.
    - Питание: 24 В.

2.  **5-степенной манипулятор**:
    - 5 вращательных сочленений.
    - Двухпальцевый схват.
    - Высота: 655 мм.
    - Вес: ~5.8 кг.
    - Полезная нагрузка: 0.5 кг.

**Связь и управление**:
- Все мотор-контроллеры (и платформы, и манипулятора) доступны через **EtherCAT**.
- ПЛК: Mini-ITX с Intel Atom, 2 ГБ ОЗУ, 32 ГБ SSD.

**Особенности ПО**:
- Робот поставляется без предустановленного ПО. Вся настройка выполняется пользователем с Live-USB.
- Исходный код драйверов и утилит — открытый и доступен на GitHub.

> 📖 **Подробная документация**:
> - [KUKA youBot User Manual (PDF)](./docs/KUKA_youBot_User_Manual.pdf)
> - [Спецификация KUKA youBot](./docs/Specification_KUKA_youBot.pdf)

---

## Архитектура проекта

Проект построен на основе многоуровневой архитектуры, описанной в документации KUKA, и адаптирован под ROS Noetic.

### Схема архитектуры

```text
┌─────────────────────────────────────────────────────────────┐
│  Robot Application Layer: youbot_controller.py             │
│  (наш Python-узел с логикой управления)                    │
└───────────────────────────┬─────────────────────────────────┘
                            │ ROS-топики:
                            │ /cmd_vel, /arm_1/arm_controller/command,
                            │ /arm_1/gripper_controller/command
┌───────────────────────────▼─────────────────────────────────┐
│  Component Layer: youbot_driver_ros_interface              │
│  (ROS-обёртка: преобразует ROS-сообщения в вызовы API)    │
└───────────────────────────┬─────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────┐
│  OODL: youbot_driver (Object-Oriented Device Layer)        │
│  (C++ API: классы YouBotBase, YouBotManipulator,           │
│   YouBotJoint)                                             │
└───────────────────────────┬─────────────────────────────────┘
                            │ EtherCAT
┌───────────────────────────▼─────────────────────────────────┐
│  Hardware Device Interfaces: SOEM EtherCAT драйвер         │
└───────────────────────────┬─────────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────────┐
│  Hardware Elements: KUKA youBot (моторы, энкодеры,         │
│  EtherCAT-шина)                                            │
└─────────────────────────────────────────────────────────────┘
```
## Описание слоёв

| **Уровень** | **Что делает** | **Компонент в проекте** |
| :--- | :--- | :--- |
| **Hardware Elements** | Физическое оборудование робота (моторы, колёса, рука). | KUKA youBot |
| **Hardware Device Interfaces** | Низкоуровневый обмен по EtherCAT. | `youbot_driver` (SOEM) |
| **Object-Oriented Device Layer (OODL)** | C++ API для управления суставами и базой. | `youbot_driver` (классы `YouBotBase`, `YouBotManipulator`) |
| **Component Layer (ROS-обёртка)** | Связывает OODL с ROS: публикует `/joint_states`, принимает команды. | `youbot_driver_ros_interface` |
| **Robot Application Layer** | Наша логика управления (чтение состояния, расчёт команд, отправка). | `youbot_controller.py` |

### Выбор репозиториев

- **`youbot_driver`** — от [a2s-institute](https://github.com/a2s-institute/youbot_driver) (стабильный форк с OODL).
- **`youbot_driver_ros_interface`** — от [huynhtancuong](https://github.com/huynhtancuong/youbot_driver_ros_interface) (ветка `noetic-devel`).
- **`youbot_description`** — от [a2s-institute](https://github.com/a2s-institute/youbot_description) (URDF-модель).
- **`brics_actuator`** — от [wnowak](https://github.com/wnowak/brics_actuator) (сообщения для управления приводами).

### Взаимодействие через ROS-топики

Наш узел `youbot_controller.py` взаимодействует с драйвером через стандартные ROS-топики:

| Топик | Тип сообщения | Назначение |
| :--- | :--- | :--- |
| `/joint_states` | `sensor_msgs/JointState` | **Вход**: текущее состояние всех суставов (положение, скорость, усилие). |
| `/cmd_vel` | `geometry_msgs/Twist` | **Выход**: команда скорости для мобильной платформы (линейная и угловая). |
| `/arm_1/arm_controller/command` | `brics_actuator/JointPositions` | **Выход**: целевые углы для 5 суставов руки. |
| `/arm_1/gripper_controller/command` | `brics_actuator/JointPositions` | **Выход**: желаемое раскрытие схвата. |

### Структура репозитория

```text
RMPC-project-KUKA-youBot/
├── config/                                    # Конфигурационные файлы драйвера
│   ├── youbot-ethercat.cfg
│   ├── youbot-base.cfg
│   └── youbot-manipulator.cfg
├── docs/                                      # Документация и прочие файлы
│   ├── KUKA_youBot_User_Manual.pdf
│   └── Specification_KUKA_youBot.pdf
├── src/                                       # Исходный код (catkin workspace)
│   ├── youbot_driver/                         # [подмодуль] Базовый EtherCAT API
│   │   └── config -> ../../../config          # симлинк на корневую папку config
│   ├── youbot_driver_ros_interface/           # [подмодуль] ROS-обёртка для Noetic
│   ├── youbot_description/                    # [подмодуль] URDF-модель
│   ├── brics_actuator/                        # [вручную] Сообщения для управления
│   └── youbot_controller/                     # [наш пакет] Управляющий узел
│       ├── scripts/
│       │   └── youbot_controller.py
│       ├── launch/
│       │   └── youbot_controller.launch
│       ├── package.xml
│       └── CMakeLists.txt
├── .gitignore
├── .gitmodules
└── README.md
```
---

## Предварительные требования

Для работы с проектом вам понадобится:

- **Операционная система**: Ubuntu 20.04 LTS (или Windows 10/11 с WSL2 + Ubuntu 20.04).
- **ROS Noetic**: полная версия (`ros-noetic-desktop-full`).
- **Git**: для клонирования репозитория.
- **Catkin**: система сборки ROS.
- **Базовые инструменты**: `build-essential`, `cmake`, `python3`.

## Инструкция по установке и настройке

### 1. Установка ROS Noetic и WSL2 (для Windows)

Если вы работаете в Windows, настоятельно рекомендуется использовать **WSL2** с Ubuntu 20.04.

1.  **Установите WSL2** (в PowerShell от имени администратора):
    ```powershell
    wsl --install -d Ubuntu-20.04
    ```
2.  **Запустите Ubuntu из меню «Пуск» и выполните первоначальную настройку** (создание пользователя).
3.  **Установите ROS Noetic внутри WSL:**
    ```powershell
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-desktop-full
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo rosdep init
    rosdep update
    ```
### 2. Клонирование репозитория

**Рекомендация:** клонируйте репозиторий **внутри WSL** (или в файловой системе Linux), чтобы избежать проблем с производительностью и правами доступа.

```bash
# В терминале Ubuntu (WSL)
cd ~
git clone --recursive https://github.com/ваш_username/RMPC-project-KUKA-youBot.git
cd RMPC-project-KUKA-youBot
```
Если вы уже клонировали без --recursive, подтяните подмодули отдельно:
```bash
git submodule update --init --recursive
```

### 3. Настройка конфигурационных файлов

#### 3.1. Конфигурационные файлы драйвера

В папке `config/` находятся три файла. Проверьте их содержимое:

- **`youbot-ethercat.cfg`** — настройка EtherCAT (имя сетевого интерфейса).
  ```cfg
  EthernetDevice = eth0
  CycleTime = 1000
  ```
  > **Важно:** если Ethernet-интерфейс называется `eth1`, измените `eth0` на `eth1`. Узнать имя можно командой `ifconfig`.

- **`youbot-base.cfg`** — топология колёс (для платформы).
  ```cfg
  [JointTopology]
  BaseLeftFront = 1
  BaseRightFront = 2
  BaseLeftBack = 3
  BaseRightBack = 4
  ```
- **`youbot-manipulator.cfg`** — топология манипулятора.
  ```cfg
  [JointTopology]
  ManipulatorJoint1 = 5
  ManipulatorJoint2 = 6
  ManipulatorJoint3 = 7
  ManipulatorJoint4 = 8
  ManipulatorJoint5 = 9
  ```

#### 3.2. Создание симлинка для конфигов

Чтобы драйвер находил конфиги, создайте символическую ссылку внутри папки драйвера:

```bash
cd ~/RMPC-project-KUKA-youBot/src/youbot_driver
rm -rf config   # если существует
ln -s ../../../config config
cd ../..
```

#### 3.3. Установка зависимостей
Установите пакеты, необходимые для сборки youbot_driver_ros_interface:

```bash
sudo apt update
sudo apt install ros-noetic-pr2-msgs ros-noetic-control-msgs
```

### 4. Сборка проекта

```bash
cd ~/RMPC-project-KUKA-youBot
rm -rf build devel   # если они уже есть
catkin_make
```
Если сборка завершилась успешно, активируйте рабочее пространство:
```bash
source devel/setup.bash
```

Проверьте, что все пакеты видны:
```bash
rospack list | grep youbot
```
Вы должны увидеть:
youbot_controller
youbot_description
youbot_driver
youbot_driver_ros_interface

---

## Запуск и тестирование

### Запуск драйвера и управляющего узла

1.  **В терминале WSL** перейдите в проект и активируйте окружение:
    ```bash
    cd ~/RMPC-project-KUKA-youBot
    source devel/setup.bash
    ```
2.  Запустите launch-файл (он запустит и драйвер, и наш контроллер):
    ```bash
    roslaunch youbot_controller youbot_controller.launch
    ```
### Проверка работы

- **Если у вас есть реальный робот**: драйвер установит соединение по EtherCAT, и вы увидите сообщения о состоянии суставов.
- **Если робота нет**: вы можете использовать симуляцию в Gazebo (требуется дополнительная настройка пакета `youbot_gazebo`).

### Что делает `youbot_controller.py`?

По умолчанию скрипт выполняет простые тестовые действия:
- Движение платформы вперёд.
- Публикация нулевых углов для манипулятора.
- Публикация команды раскрытия схвата.

Вы можете изменить логику в файле `src/youbot_controller/scripts/youbot_controller.py`.

## Устранение неполадок

| **Проблема** | **Решение** |
| :--- | :--- |
| `catkin_make` не найден | Убедитесь, что Вы выполнили `source /opt/ros/noetic/setup.bash`. |
| Ошибка `brics_actuator` не найден | Убедитесь, что папка `src/brics_actuator` существует (Вы её склонировали). |
| Ошибка `pr2_msgs` или `control_msgs` | Установите: `sudo apt install ros-noetic-pr2-msgs ros-noetic-control-msgs`. |
| `youbot_driver_ros_interface` требует прав root | Выполните: `sudo setcap cap_net_raw+ep ~/RMPC-project-KUKA-youBot/devel/lib/youbot_driver_ros_interface/youbot_driver_ros_interface` |
| Ошибка "No socket connection on eth0" | Проверьте имя интерфейса в `config/youbot-ethercat.cfg` (может быть `eth1`). |
| Медленная сборка в WSL из-за `/mnt/c/` | Скопируйте проект в домашнюю папку WSL (`~/`) и работайте оттуда. |

## Полезные ссылки

- **Официальная документация KUKA youBot**: [User Manual](./docs/KUKA_youBot_User_Manual.pdf)
- **ROS Noetic Installation**: [wiki.ros.org/noetic/Installation/Ubuntu](https://wiki.ros.org/noetic/Installation/Ubuntu)
- **Репозиторий драйвера**: [a2s-institute/youbot_driver](https://github.com/a2s-institute/youbot_driver)
- **ROS-обёртка**: [huynhtancuong/youbot_driver_ros_interface](https://github.com/huynhtancuong/youbot_driver_ros_interface)
- **URDF-модель**: [a2s-institute/youbot_description](https://github.com/a2s-institute/youbot_description)
