# ros2_diagnostic_weather
```sh
git clone --recurse-submodules git@github.com:alchrist42/ros2_diagnostic_weather.git
cd ros2_diagnostic_weather
rosdep install -y --from-paths src --ignore-src --rosdistro iron
colcon build
source source ./install/setup.bash 
```

#### for run
`ros2 launch weather_diagnostic diagnostic.launch.py`  
or  
`ros2 run weather_diagnostic weather_diagnostics`
