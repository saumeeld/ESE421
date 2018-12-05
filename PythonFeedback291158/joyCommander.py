import pygame
import time

class JoyCommander():
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        
        self.J = pygame.joystick.Joystick(0)
        self.J.init()
        
        self.VD = 0
        self.TopSpeed = 90
        self.SpeedScale = .05;

    def setSpeed(self, speed):
        self.VD = speed
        if self.VD > self.TopSpeed:
            self.VD = self.TopSpeed

    def getCommands(self):
        for k in range(self.J.get_numaxes()):
            pygame.event.pump()

        for k in range(self.J.get_numbuttons()):
            pygame.event.pump()

        psiD = (self.J.get_axis(0)*30);
        A = self.J.get_button(6)
        B = self.J.get_button(8)
        Start = self.J.get_button(9)
        Z = self.J.get_button(7)
        R = self.J.get_button(5)

        self.VD = self.VD + (A*self.TopSpeed)*self.SpeedScale

        self.VD = self.VD - (B*self.TopSpeed)*self.SpeedScale
        if self.VD < 0:
            self.VD = 0
        if self.VD > self.TopSpeed:
            self.VD = self.TopSpeed

        takePic = Z
        eStop = Start
        switch = R

        return [psiD, self.VD, takePic, eStop, switch]
