## ROS master/client configuration

For ROS nodes to find each other, some environment variables (```ROS_IP```, ```ROS_MASTER_URI```) should be properly configured to match the used network.

### Master

The master node could be configured by:
```bash
source ~/cascar_ws/src/cascar_pkgs/conf_scripts/setrosmaster.sh
```

In the script, IP address which is assigned to WiFi interface (```wlan0```) is parsed and inserted into two export commands defining the needed environment variables.
The master node has enough information to setup all variable by its own.

One could manually check which IP address is assigned to the master node and configure environment variables on the clients. Or use the provided scripts which work under the assumption that all nodes are located inside one subnetwork, meaning that the nodes could comminuted via broadcast messages.

The master runs ```rosmasterip.service```, which gets broadcast messages from clients and replies to them, so they could find out the master IP address. The service is based on a small Python program, ```rosmasterip.py```.

To setup the service in the system, copy the file:
```bash
sudo cp ~/cascar_ws/src/cascar_pkgs/conf_scripts/rosmasterip.service  /etc/systemd/system
```
username ```cascar``` is assumed, update it if needed:
```bash
sudo nano /etc/systemd/system/rosmasterip.service
```

To start the service run:
```bash
systemctl daemon-reload
systemctl enable rosmasterip.service
systemctl start rosmasterip.service
```

The service should start immediately and also start automatically after each system boot.

(The service configuration is an adaptation of https://www.digitalocean.com/community/questions/convert-run-at-startup-script-from-upstart-to-systemd-for-ubuntu-16)

### Client

When rosmasterip.service is running and the master and the clients are on the same network, each client could set up its environment variables with:
```bash
  source  ~/cascar_ws/src/cascar_pkgs/conf_scripts/setrosclient.sh
```

The script runs ```rosclientip.py```  to obtain the master node IP and creates ```envrosip.sh``` with the suitable environment variables, which are later sourced by the script.

## Prevent headless mode
To prevent the Rapsberry Pi to start in headless mode (no HDMI output, WiFi hotspot problems),
update the file ```/boot/firmware/usercfg.txt``` with the provided file ```usercfg.txt```.
The configuration is taken from https://www.enricozini.org/blog/2020/himblick/raspberry-pi-4-force-video-mode-at-boot/.
A detailed explanation of the parameters is available at https://www.raspberrypi.org/documentation/configuration/config-txt/video.md.
