import socket

master = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
master.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
master.bind(("", 37020))
# master.settimeout(0.2)
# i = 0
# while i<3:
while True:
    data, addr = master.recvfrom(1024)
    print("received message: %s"%data)
    print(["from", addr])
    if data == b'get_master_ip':
        #import pdb; pdb.set_trace()
        master.sendto(b'i_am_master-my_ip:'+addr[0].encode(), addr)
        #print(['i_am_master-my_ip:'+addr[0], addr])
    # i = i + 1
