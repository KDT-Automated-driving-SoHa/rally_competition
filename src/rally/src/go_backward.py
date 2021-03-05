#!/usr/bin/env python

class BACKWARD:
    angle = 0
    speed = 0
    
    def __init__(self):
        pass

    def set_motor(self, angle, speed):
        self.angle = angle
        self.speed = speed
	

    def set_data(self, data):
        #pass
	
	for j in range(10):
	    self.set_motor(0,0)
	    time.sleep(0.1)
		
	if data==False:
	    for j in range(3):
	    	self.set_motor(30, -15)
		time.sleep(0.1)
 	elif data:
            for j in range(3):
		self.set_motor(-30,15)	
		time.sleep(0.1)
		
    def get_motor(self):
        return self.angle, self.speed
