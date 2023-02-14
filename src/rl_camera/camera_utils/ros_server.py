import socket
import cv2
import base64
import numpy as np

class ServerNode:
    def __init__(self):
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Bind the socket to the port
        server_address = ('192.168.0.158', 8000)
        print('starting up on {} port {}'.format(*server_address))
        self.sock.bind(server_address)

        # Listen for incoming connections
        self.sock.listen(1)
        self.encoding = [int(cv2.IMWRITE_JPEG_QUALITY),90]
        print('waiting for a connection')
        self.connection, self.client_address = self.sock.accept()
        try:
            print('connection from', self.client_address)
        except:
            print("There was an error")


    def send_image(self, image):
        res, encoded_img = cv2.imencode('.jpg', image, self.encoding)
        data = np.array(encoded_img)
        data_string = base64.b64encode(data)
        length = str(len(data_string))
        self.connection.sendall(length.encode('utf-8').ljust(64))
        self.connection.send(data_string)

    def run(self):
        # Wait for a connection
        print('waiting for a connection')
        connection, client_address = self.sock.accept()
        try:
            print('connection from', client_address)

            # TODO: change video cap to zed camera image
            # Read the video file and send it to the client in small chunks
            video = cv2.VideoCapture('video.mp4')
            while video.isOpened():
                ret, frame = video.read()
                if ret:
                    # Encode the frame in base64 and send it to the client
                    res, encoded_img = cv2.imencode('.jpg', frame, self.encoding)
                    data = np.array(encoded_img)
                    data_string = base64.b64encode(data)
                    length = str(len(data_string))
                    connection.sendall(length.encode('utf-8').ljust(64))
                    connection.send(data_string)
                else:
                    break

            # Receive a message from the client
            data = connection.recv(1024)
            # Decode the message from base64
            decoded_data = base64.b64decode(data)
            print('received {!r}'.format(decoded_data))
        finally:
            # Clean up the connection
            connection.close()

if __name__ == '__main__':
    # Create the server node
    server_node = ServerNode()

    # Run the node
    server_node.run()