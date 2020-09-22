import socket
import time
from random import randint

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
port = 12345
s.bind(('', port))
s.listen(5)

while True:
    c, addr = s.accept()
    print('Connection form', addr)
    while True:
        try:
            data = str(randint(0, 9)) + ',' + str(randint(0, 9)) + ',' + str(randint(0, 9))
            print(data)
            c.send(bytes(data, 'utf-8'))
        except socket.error:
            print ('Client connection closed', addr)
            break

c.close()
