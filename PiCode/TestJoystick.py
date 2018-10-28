import pygame
import time

pygame.init()
pygame.joystick.init()
J = pygame.joystick.Joystick(0)
J.init()
print(J.get_name())

while True:
    time.sleep(0.5)
    axes = []
    for k in range(J.get_numaxes()):
        pygame.event.pump()
        axes.append(J.get_axis(k))
##    print("{}".format(axes))

    buttons = []
    for k in range(J.get_numbuttons()):
        pygame.event.pump()
        buttons.append(J.get_button(k))
    print("{}".format(buttons))
    
    hats = []
    print("Number of hats is {}".format(J.get_numhats()))
    for k in range(J.get_numhats()):
        pygame.event.pump()
        hats.append(J.get_hat(k))
    print("{}".format(hats))
        


