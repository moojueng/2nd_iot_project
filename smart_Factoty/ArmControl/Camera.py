import cv2
import numpy as np

class check:
    def __init__(self, count, check, model):
        self.count = count
        self.check = check
        self.model = model
        
    def cam_check(self):
        capture = cv2.VideoCapture(0) 

        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        while(capture.isOpened()):
            ret, frame = capture.read()

            if ret == True: 
                print("read success")

            frame_fliped = cv2.flip(frame, 1)
            
            if self.count > 8:
                capture.release()
                cv2.destroyAllWindows()
                print(self.check)
                return self.check
            size = (224, 224)
            
            frame_resized = cv2.resize(frame_fliped, size, interpolation=cv2.INTER_AREA)
        
            frame_normalized = (frame_resized.astype(np.float32) / 127.0) - 1
        
            frame_reshaped = frame_normalized.reshape((1, 224, 224, 3))
            
            preprocessed = frame_reshaped

            prediction = self.model.predict(preprocessed)

            print(prediction)
            
            if prediction[0,0] > prediction[0,1]:
                self.check=1
            else:
                self.check=2
            
            self.count += 1
