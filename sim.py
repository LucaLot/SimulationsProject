#Luca Lotito
#Main class for the final project
import pygame, sys
import pygame_widgets
from pygame_widgets.slider import Slider
from pygame_widgets.textbox import TextBox

import flightSim

WHITE = (255, 255, 255)

def main():
    #Simulation setup
    title = 'Paper Airplane Simulator'
    pygame.init()
    font = pygame.font.Font("arial.TTF", 18)
    hasStarted =False
    clock = pygame.time.Clock()
    
    win_width = 960
    win_height = 960
    pos = [10, win_height/2, 0]
    screen = pygame.display.set_mode((win_width, win_height))
    pygame.display.set_caption(title)
    background = pygame.image.load('win_XP.jpg')

    plane = flightSim.Plane(win_height, 'side-view.png')
    plane.setup(pos, 10, 10)
    #Bars and buttons to add functionality
    s = Button(30, 900, 160, 50, 'Start/Stop Simulation', plane.pause, font)
    e = Button(200, 900, 160, 50, 'Quit Simulation', sys.exit, font)

    slideR = Slider(screen, 400, 900, 200, 30, min=-15, max=15, initial = 0, step=1)
    textR = TextBox(screen, 400, 840, 230, 40, fontSize=18)
    slideS = Slider(screen, 650, 900, 200, 30, min=1, max=20, initial = 10, step=1)
    textS = TextBox(screen, 650, 840, 200, 40, fontSize=18)

    textR.disable()
    textS.disable()
    objects = [s, e]
    #Main loop
    while True:
        clock.tick(30)
        events = pygame.event.get()
        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit(0)
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_q:
            pygame.quit()
            sys.exit(0)
        elif plane.get_pos()[0] > 210 or plane.get_pos()[1] < 380: 
            pygame.quit()
            sys.exit(0)
        #Values only update if the simulation is running
        if not hasStarted:
            textR.setText(f"Initial Angle Value (°) = {slideR.getValue()}")
            textS.setText(f"Initial Speed (m/s) = {slideS.getValue()}")
            #Runs the setup again. Probably a better way to update speed and angle, but this worked the best
            plane.setup(pos, slideS.getValue(), slideR.getValue())
        #When simulation is running, update with current values instead
        if not plane.paused:
            hasStarted=True
            plane.step()
            ang, _l = plane.get_angle_2d()
            textR.setText(f"Angle Value (°) = { str(round(ang,2))}")
            textS.setText(f"Speed (m/s) = { str(round(plane.getSpeed(),2))}")
        #Updates UI elements
        screen.blit(background, (0,0))
        for o in objects:
            o.process(screen)
        pygame_widgets.update(events)
        plane.rotate()
        plane.move()
        plane.draw(screen)
        pygame.display.update()



#Mostly bolier plate button code to get things working 
class Button():
    def __init__(self, x, y, width, height, buttonText, onclickFunction, font) -> None:
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.buttonText = buttonText
        self.onclickFunction = onclickFunction
        self.alreadyPressed=False

        self.buttonSurface = pygame.Surface((self.width, self.height))
        self.buttonRect = pygame.Rect(self.x, self.y, self.width, self.height)
        self.buttonSurf = font.render(buttonText, True, (255, 255, 255))
        self.fillColors = {
            'normal': '#333333',
            'hover': '#666666',
            'pressed': '#111111'
        }

    def process(self, screen):
        mousePos = pygame.mouse.get_pos()
        self.buttonSurface.fill(self.fillColors['normal'])
        if self.buttonRect.collidepoint(mousePos):
            self.buttonSurface.fill(self.fillColors['pressed'])
            if pygame.mouse.get_pressed(num_buttons=3)[0]:
                self.buttonSurface.fill(self.fillColors['pressed'])
                if not self.alreadyPressed:
                    self.onclickFunction()
                    self.alreadyPressed = True
            else:
                self.alreadyPressed=False
        self.buttonSurface.blit(self.buttonSurf, [self.buttonRect.width/2-self.buttonSurf.get_rect().width/2,
                                                  self.buttonRect.height/2 - self.buttonSurf.get_rect().height/2])
        screen.blit(self.buttonSurface, self.buttonRect)


if __name__ == '__main__':
    main()
