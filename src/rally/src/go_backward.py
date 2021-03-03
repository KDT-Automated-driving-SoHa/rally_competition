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
	for i in range(5):
	    for j in range(10):
		set_motor(0,0)
		time.sleep()

	    for j in range(data):
	    	set_motor(20, -25)
		time.sleep(0.1)
 	    
            for j in range(data):
		set_motor(20,25)	
		time.sleep(0.1)
    def get_motor(self):
        return self.angle, self.speed
