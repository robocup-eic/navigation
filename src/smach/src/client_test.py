from custom_socket import CustomSocket
import socket
import numpy as np
import cv2

if __name__ == "__main__":
    # connect to server
    host = socket.gethostname()
    port = 11000
    c = CustomSocket(host,port)
    c.clientConnect()
    cap = cv2.VideoCapture(0)
    while True:
        _, img = cap.read()
        # print(img.shape)
        img = cv2.resize(img, (720,1080))
        result = c.req(img)
        print(result)