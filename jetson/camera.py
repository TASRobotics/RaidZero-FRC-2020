import numpy as np
import cv2

class cameraSet:

    camArray = []
    frameArray = []
    loss_factor = 16

    def capFrame(self,id):
        ret, frame = self.camArray[id].read()
        frame = cv2.resize(frame, (256, 144))
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR555)
        frame = np.reshape(frame, (12, -1))
        self.frameArray[id] = frame

    def getFrame(self, id):
        return self.frameArray[id]

    def decCam(self, id):
        return cv2.VideoCapture(id)

    def getLen(self):
        return len(self.camArray)

    def closeCap(self):
        for cam in self.camArray:
            self.cam.release()

    def startCap(self):
        global camArray
        global frameArray

        for i in range(4):
            cap = self.decCam(int(i))
            if cap is None or not cap.isOpened():
                print('camera ', str(i), ' does not exist')
            else:
                print('declared')
                self.camArray.append(cap)
        print('Cameras added:',len(self.camArray))
        self.frameArray = [None] * len(self.camArray)

if __name__ == '__main__':
    cams = cameraSet()
    cams.startCap()
    print('started cap')
    while True:
        for id in range(len(cams.camArray)):
            cams.capFrame(id)
            cv2.imshow('id:'+ str(id),cams.getFrame(id))
        if cv2.waitKey(10) & 0xFF == ord("q"): 
            break
    for cam in cams.camArray:
        cam.release()
    cv2.destroyAllWindows()