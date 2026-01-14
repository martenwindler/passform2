# Bauen
colcon build --packages-select passform2

# Umgebung laden
source install/setup.bash

# Node starten
ros2 run passform2 manager
