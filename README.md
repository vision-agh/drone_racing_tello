# A simple vision-based navigation and control strategy for autonomous drone racing


Please take a look at the video abstract: 

[![Video abstract](https://img.youtube.com/vi/3o-n0vZu0ps/0.jpg)](https://youtu.be/3o-n0vZu0ps "A simple vision-based navigation and control strategy for autonomous drone racing")


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
   In the end you should establish a connection with the Tello EDU drone (like using your smartphone).
   
4. Install python3.7 and pip:
   ```
   sudo apt-get install python3
   sudo apt-get install python3-pip
   ```
6. Install dependencies in linux console if You want to run a project from a console using command: 
    ```
    pip3 install -r requirements_linux.txt
    ``` 
4. Turn on Tello drone and connect to its Wi-Fi 
5. Run program in console via command 
    ```
    python Jetson.py 
    ```
    or in PyCharm.
