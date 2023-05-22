from controller import Robot, Node
import base64
import os
import sys
import tempfile

try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found. Please check the Python modules installation instructions " +
             "at 'https://www.cyberbotics.com/doc/guide/using-python'.")
try:
    import cv2
except ImportError:
    sys.exit("Warning: 'cv2' module not found. Please check the Python modules installation instructions " +
             "at 'https://www.cyberbotics.com/doc/guide/using-python'.")


def cleanup():
   
    try:
        os.remove(deviceImagePath + '/display.jpg')
    except OSError:
        pass
    try:
        os.remove(deviceImagePath + '/camera.jpg')
    except OSError:
        pass


def sendDeviceImage(robot, device):
   
    if device.getNodeType() == Node.DISPLAY:
        deviceName = 'display'
        fileName = deviceName + '.jpg'
        device.imageSave(None, deviceImagePath + '/' + fileName)
    elif device.getNodeType() == Node.CAMERA:
        deviceName = 'camera'
        fileName = deviceName + '.jpg'
        device.saveImage(deviceImagePath + '/' + fileName, 80)
    else:
        return
    with open(deviceImagePath + '/' + fileName, 'rb') as f:
        fileString = f.read()
        fileString64 = base64.b64encode(fileString).decode()
        robot.wwiSendText("image[" + deviceName + "]:data:image/jpeg;base64," + fileString64)



deviceImagePath = os.getcwd()
try:
    imageFile = open(deviceImagePath + "/image.jpg", 'w')
    imageFile.close()
except IOError:
    deviceImagePath = tempfile.gettempdir()


robot = Robot()


timestep = int(robot.getBasicTimeStep() * 4)


panHeadMotor = robot.getDevice('PRM:/r1/c1/c2-Joint2:12')
tiltHeadMotor = robot.getDevice('PRM:/r1/c1/c2/c3-Joint2:13')



panHeadMotor.setPosition(float('+inf'))
tiltHeadMotor.setPosition(float('+inf'))

panHeadMotor.setVelocity(0.0)
tiltHeadMotor.setVelocity(0.0)


camera = robot.getDevice('PRM:/r1/c1/c2/c3/i1-FbkImageSensor:F1')
camera.enable(timestep)
width = camera.getWidth()
height = camera.getHeight()


display = robot.getDevice('display')

display.attachCamera(camera)
display.setColor(0xFF0000)


targetPoint = []
targetRadius = 0


while robot.step(timestep) != -1:

    if targetPoint:
         
        display.setAlpha(0.0)
        radius = targetRadius
        if radius < 5:
            
            radius = 5
        size = 2 * radius + 1
        display.fillRectangle(targetPoint[0] - radius, targetPoint[1] - radius, size, size)

  

    rawString = camera.getImage()

   
    index = 0
    maskRGB = np.zeros([height, width], np.uint8)
    for j in range(0, height):
        for i in range(0, width):
            
            if sys.version_info.major > 2:  
                b = rawString[index]
                g = rawString[index + 1]
                r = rawString[index + 2]
            else:  
                b = ord(rawString[index])
                g = ord(rawString[index + 1])
                r = ord(rawString[index + 2])
            index += 4
            
            if r > 200 and g > 200 and b > 200:
                maskRGB[j][i] = True

    
    contours = cv2.findContours(maskRGB.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

   
    if not contours:
        continue

  
    blob = max(contours, key=cv2.contourArea)

    
    ((x, y), radius) = cv2.minEnclosingCircle(blob)
    targetPoint = [int(x), int(y)]
    targetRadius = int(radius)

  
    display.setAlpha(1.0)
    if targetRadius > 0:
        display.setColor(0x00FFFF)
        display.drawOval(targetPoint[0], targetPoint[1], targetRadius, targetRadius)
    display.setColor(0xFF0000)
    display.fillOval(int(targetPoint[0]), int(targetPoint[1]), 5, 5)
   
    sendDeviceImage(robot, display)

   
    dy = targetPoint[0] - width / 2
    dz = targetPoint[1] - height / 2
   
    panHeadMotor.setVelocity(-1.5 * dy / width)
    tiltHeadMotor.setVelocity(-1.5 * dz / height)

cleanup()