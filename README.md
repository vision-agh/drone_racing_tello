# drone_racing_tello



## Running on PC
1. Instal dependencies using the follwoing command: 
    ```
    pip install -r requirements.txt
    ```
2. Turn on the Tello EDU drone and connect to it's Wi-Fi (like when using the mobile appilcation). 
3. Run DronConfig.py.

## Running on Jetson TX2 (should be similar for other devices)

1. Board configuration and JetPack instalation is described the Jetson TX2 Developer Kit User Guide (avaliavle on https://developer.nvidia.com/ after registering and logging in).
2. Connection to Wi-Fi is described here: https://www.linuxbabe.com/ubuntu/connect-to-wi-fi-from-terminal-on-ubuntu-18-04-19-04-with-wpa-supplicant
3. Install dependencies in linux console if You want to run a project from a console using command: 
    ```
    xargs sudo apt-get install requirements.txt 
    ```
    or install requirements in pycharm using: 
    ```
    pip install -r requirements.txt
    ``` 
4. Turn on Tello drone and connect to its Wi-Fi 
5. Run program in console via command 
    ```
    python Jetson.py 
    ```
    or in Pycharm.
