import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

port = 12345
s.connect((socket.gethostname(), port))

while True:
    try:
        data = s.recv(1024)
        data = data.decode('utf-8')
        data = data.split(",")
        if data[0]==" ":
            print(data)
            data = data[1:]
        try:
            cur_pos = [float(data[0]), float(data[1]), float(data[2])]
            print (cur_pos)
        except:
            print (data)
            break
    except KeyboardInterrupt:
        s.close
s.close
