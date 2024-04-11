# corgi_sim

1. Comment ```import std.proto``` in all the proto files at *~/corgi_ws/corgi_system/protos*

2. Build the controller with the following command

>>```
>>cmake .. -DCMAKE_PREFIX_PATH=$HOME/corgi_ws/build
>>```

+ When running the RT-control, follow these steps:
>>1. Run ```bash simulation.sh``` in *~/corgi_ws/corgi_system*
>>2. Press "Run" in the webots
>>3. Press "Enter" in the terminal which is running simulation.sh
>>4. Open *~/corgi_ws/corgi_system/js/control_panel/index.html* (UI)
>>5. Input 127.0.0.1:8080 in the UI, and send commands to the robot

+ If a new robot is added by *corgi.proto*, remeber to convert the robot node into base node in the scene tree penal.
