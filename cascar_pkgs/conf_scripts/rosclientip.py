import socket
import time
import getpass

client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
# Set a timeout so the socket does not block
# indefinitely when trying to receive data.
client.settimeout(5)
client.bind(("", 44444))
message = b"get_master_ip"
# while True:
client.sendto(message, ('<broadcast>', 37020))
print("get_master_ip sent!")
# time.sleep(1)
data, addr = client.recvfrom(1024)
print("received message: %s"%data)
print(["from", addr])
ownip = data.decode().split(':')[1]

username = getpass.getuser()
file = open('/home/'+username+'/cascar_ws/src/opcascar2020/cascar_pkgs/conf_scripts/envrosip.sh','wb')
lines = [b'export ROS_IP='+ownip.encode()+b'\n',\
b'export ROS_MASTER_URI=http://'+addr[0].encode()+b':11311/\n',\
b'export ROS_MASTER_IP='+addr[0].encode()+b'\n']
print("Writing to envrosip.sh:")
for line in lines:
	file.write(line)
	print(line)
file.close()
