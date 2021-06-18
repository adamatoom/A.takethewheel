import pygame
import serial
import time
arduino = serial.Serial(port='COM8', baudrate=115200, timeout=.1)
time.sleep(10)

BLACK = pygame.Color('black')
WHITE = pygame.Color('white')

class TextPrint(object):
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def tprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10
count = 0
pygame.init()
screen = pygame.display.set_mode((500, 700))
clock = pygame.time.Clock()
ang =0
inc = 0.2     
textPrint = TextPrint()
pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
print(joysticks)
joystick1 = pygame.joystick.Joystick(0)
joystick1.init()
print(joystick1.get_init())
print(joystick1.get_id())
print(joystick1.get_name())
print(joystick1.get_numaxes())
print(joystick1.get_numballs())
print(joystick1.get_numbuttons())
print(joystick1.get_numhats())
axis1=0
clock.tick(5)
while 1:
    pygame.event.get()
    screen.fill(WHITE)
    textPrint.reset()

    axes = joystick1.get_numaxes()
    textPrint.tprint(screen, "Number of axes: {}".format(axes))
    textPrint.indent()
    for i in range(axes):
        axis = joystick1.get_axis(i)
        textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(i, axis))
    textPrint.unindent()
    buttons = joystick1.get_numbuttons()
    textPrint.tprint(screen, "Number of buttons: {}".format(buttons))
    textPrint.indent()

    for i in range(buttons):
        button = joystick1.get_button(i)
        textPrint.tprint(screen,
                         "Button {:>2} value: {}".format(i, button))
    textPrint.unindent()

    hats = joystick1.get_numhats()
    textPrint.tprint(screen, "Number of hats: {}".format(hats))
    textPrint.indent()

    # Hat position. All or nothing for direction, not a float like
    # get_axis(). Position is a tuple of int values (x, y).
    for i in range(hats):
        hat = joystick1.get_hat(i)
        textPrint.tprint(screen, "Hat {} value: {}".format(i, str(hat)))
    textPrint.unindent()

    textPrint.unindent()

    axis1 = joystick1.get_axis(0)*127+127
    arduino.write(bytes([int(max(min(axis1,255),0))]))
    pygame.display.flip()
    
    print(max(min(axis1*127+127,255),0))
    print(' ')
    print(bytes([int(max(min(axis1*127+127,255),0)/6.5)]))
    

    clock.tick(5)
    

pygame.quit()
