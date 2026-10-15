def sysCall_init():
    sim = require('sim')
    simVision = require('simVision')

def sysCall_vision(inData):
    simVision.sensorImgToWorkImg(inData['handle']) 
    simVision.edgeDetectionOnWorkImg(inData['handle'], 0.2) 
    simVision.workImgToSensorImg(inData['handle']) 

def sysCall_actuation():
    pass

def sysCall_sensing():
    pass

def sysCall_cleanup():
    pass
