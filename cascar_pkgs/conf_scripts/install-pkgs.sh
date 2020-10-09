# shell script to install all necessary stuff for cascar 2020

sudo apt-get update
sudo apt install net-tools
sudo apt install openssh-server
sudo ufw allow ssh
sudo apt install git
sudo apt install python3-pip
# Then use pip3 install -r requirements
sudo apt-get install python3-rosdep
sudo rosdep init
# Then use rosdep init
#echo set completion-ignore-case on | sudo tee -a /etc/inputrc


