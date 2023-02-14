import socket
import cv2
import base64
import numpy as np



def recvall(sock, count):
    # http://stupidpythonideas.blogspot.com/2013/05/sockets-are-byte-streams-not-message.html
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf



def mouse_callback(event, x, y, flags, params):
    # If the left mouse button was clicked, send the coordinates to the server
    if event == cv2.EVENT_LBUTTONDOWN:
        message = f'Mouse clicked at ({x}, {y})'
        # Encode the message in base64
        encoded_message = base64.b64encode(bytes(message, 'utf-8'))
        sock.sendall(encoded_message)

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


# Connect the socket to the port where the server is listening
server_address = ('192.168.0.158', 8000)
print('connecting to {} port {}'.format(*server_address))
sock.connect(server_address)

try:
    # Set the mouse event handler
    cv2.namedWindow('Video')
    cv2.setMouseCallback('Video', mouse_callback)

    # Receive the video data in small chunks and display it using OpenCV
    while True:
        length = recvall(sock, 64)
        length_dec = length.decode('utf-8')
        stringData = recvall(sock, int(length_dec))
        data = np.frombuffer(base64.b64decode(stringData), np.uint8)
        decimg = cv2.imdecode(data, 1)
        cv2.imshow('Video', decimg)
        cv2.waitKey(10)
    # Clean up
    cv2.destroyAllWindows()
    sock.close()












