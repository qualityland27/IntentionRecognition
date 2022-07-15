class detectedObjects:
    '''
    object class holds info about detected object
    '''
    def __init__(self):
        self.x = 0
        self.y = 0
        self.width = 0
        self.height = 0
        self.angle = 0
        self.time = 0
    
    def update(self, object):
        self.x = object.x
        self.y = object.y
        self.width = object.width
        self.height = object.height
        self.angle = object.angle
        self.time = object.time



