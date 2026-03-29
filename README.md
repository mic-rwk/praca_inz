# praca_inz

## Cel

Celem projektu jest stworzenie symulacji w ROS2 oraz Gazebo. Następnie zbierane są dane z symulacji z czujników oraz odometrii, w celu estymacji prędkości robota przez model sztucznej inteligencji.

## Etap 1 - ROS2 i Gazebo

Pierwszym etapem było stworzenie symulacji w środowisku ROS2 Foxy. Podczas symulacji miały być zbierane dane z odometri robota, skany laserowe z lidaru. Należało zmapować teren, aby móc zadawać robotowi pozycję, gdzie ma się przemieścić.

### Budowa robota

<div class="grid" markdown>

![Image title](res/robot_in_the_world.png)

![Image title](res/robot_view.png)

</div>

### Zbieranie danych

Zbieranie danych odbywa się z wykorzystywaniem topic'ów:  */joint_states* ,  */scan*  oraz */odom*. Następnie klasy związane z: prędkościami enkoderów, skanami z lidara oraz odometrią przetwarzają w odpowiedni sposób wiadomości z tych topiców. Potem publikują nowe, odpowiednie wiadomości (zdefiniowane w folderze msg) do nowych topików. Później dane są synchronizowane w czasie, dzięki paczce [MessageFilters](https://docs.ros.org/en/rolling/p/message_filters/), zaimplementowanej w klasie *RobotMonitor*. Następnie dane są "nagrywane" przez *rosbaga*, który subskrybuje topic */robot_monitor.*

```python
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', './src/robot/bag_files/data1', '/robot_monitor'],
        output='screen'
    )
```

### Mapowanie

Kolejnym krokiem było mapowanie terenu. W tym celu wykorzystano gotowy świat z Gazebo - [WillowGarage](https://github.com/arpg/Gazebo/blob/master/worlds/willowgarage.world) oraz - [mapy labiryntów](https://github.com/HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving). Włączono symulację w Gazebo. Następnie wykorzystano launchfile online_async_launch z paczki slamtoolbox, aby zmapować teren. W celu sprawdzenie poprawności mapy uruchomiono program rviz2. Aby poruszać się robotem wykorzystano teleop.

```bash
ros2 launch robot launch_sim_launch.py world:=./src/robot/worlds/willowgarage.world

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

ros2 run rviz2 rviz2 -d /opt/ros/foxy/share/nav2_bringup/rviz/nav2_default_view.rviz 

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_controller/cmd_vel_unstamped
```
![Alt text](res/map.png)

### Nawigacja

Po uruchomieniu świata Gazebo, włączono nav2_bringup, aby umożliwić nawigację robota. Włączono Rviza, aby mieć podgląc, jak przebiega nawigacja w świecie. Dodatkowo użyto *twist_mux*, aby móc korzystać z 2 źródeł dla prędkości - z nav2 oraz teleop.

```bash
ros2 launch robot launch_sim_launch.py
```

![Alt text](res/navigation.png)

## Instalacja

```bash
git clone https://github.com/mic-rwk/praca_inz.git
```

### Otwarcie konteru Dockera

Przez basha lub w VSCode CTRL+SHIFT+P -> Rebuild Container. W celu włączenia Gazebo konieczne może być ustawienie tej komendy:

```bash
export DISPLAY=":0"
```

```bash
xhost +
```

### Uruchomienie symulacji

```bash
cd ros_ws
```

```bash
source install/setup.bash
```

W ciągu 10-20 sekund uruchamia się: Gazebo, RViz, PlotJuggler (wyświetlanie prędkości robota mobilnego) oraz Nav2.

```bash
ros2 launch robot launch_sim_launch.py
```

Następnie można ręcznie sterować robotem przez RViz lub z wykorzystanime node auto_goal_setter. Node będzie zlecał automatycznie nowe punkty, do których robot ma jechać. 

```bash
ros2 run robot auto_goal_setter.py
```

Przekonwertuj rosbaga do csv.

```bash
python3 RosbagParser.py -b $(find src/robot/bag_files/willowgarage -name "*2026*") -t /robot_monitor
```

Zapisz z CSV do przefiltrowanego CSV - same liczby, bez wiadomości z ROSa.

```bash
python3 dataset/DatasetCreate.py -f "csv_from_rosbag/willowgarage/*" -o "csv_output/willow2"
```

Uruchom proces uczenia. Zweryfikuj wyniki na zbiorze testowym.

```bash
python3 dataset/create_models.py
```