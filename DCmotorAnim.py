import sys, pygame, time, random
pygame.init()

size = width, height = 1500, 1000
velocity = [0, 1] #speed vector

black = 0, 0, 0
WHITE=(255,255,255)
blue=(0,0,255)

global myfont

pygame.init()

class Car(object):  # represents the bird, not the game
    def __init__(self):
        """ The constructor of the class """
        self.image = pygame.image.load("cart.jpg")
        # the bird's position
        self.x = (size[0]/2)-(self.image.get_width()/2)
        self.y = (size[1]-100)-self.image.get_height()
        self.myfont = pygame.font.Font(None, 72)

    def handle_keys(self):
        """ Handles Keys """
        key = pygame.key.get_pressed()
        self.dist = 5 # distance moved in 1 frame, try changing it to 5
        # if key[pygame.K_DOWN]: # down key
        #     self.y += self.dist # move down
        # elif key[pygame.K_UP]: # up key
        #     self.y -= self.dist # move up
        if key[pygame.K_RIGHT]: # right key
            self.x += self.dist # move right
        elif key[pygame.K_LEFT]: # left key
            self.x -= self.dist # move left

        self.left=self.x
        self.right=self.x+self.image.get_width()

    def draw(self, surface):
        """ Draw on surface """
        # blit yourself at your current position
        surface.blit(self.image, (self.x, self.y))

    def test_contact(self,size):
        if self.left <= 0 :
            self.x=1
        if self.right >= size[0]:
            self.x=size[0]-(1+self.image.get_width())
        if self.y <= 0:
            self.y=1
        if self.y+self.image.get_height() >= size[1]:
            label = self.myfont.render("Game Over", 1, (0,0,0))
            screen.blit(label, ((size[0]/2)-(label.get_width()/2), (size[1]/2)-(label.get_height()/2)))


class Map(object):
    def __init__(self):
        self.p1=(500,-500)
        self.p2=(500,500)
        self.p3=(1000,-500)
        self.p4=(1000,500)
        self.p5=(600,-1500)
        self.p6=(1100,-1500)
        self.i=0
    def update(self,screen,mapSurf,size,speed):
        mapSurf.fill(WHITE)


        self.current1=self.p1[0]
        self.current2=self.p2[0]
        pygame.draw.aaline(mapSurf,(0,0,0),self.p1,self.p2,8)
        pygame.draw.line(mapSurf,(0,0,0),self.p3,self.p4,8)
        pygame.draw.line(mapSurf,(0,0,0),self.p5,self.p1,8)
        pygame.draw.line(mapSurf,(0,0,0),self.p6,self.p3,8)
        self.p1=(self.p1[0],self.p1[1]+speed)
        self.p2=(self.p2[0],self.p2[1]+speed)
        self.p3=(self.p3[0],self.p3[1]+speed)
        self.p4=(self.p4[0],self.p4[1]+speed)
        self.p5=(self.p5[0],self.p5[1]+speed)
        self.p6=(self.p6[0],self.p6[1]+speed)
        if self.p5[1] >= 0:
            
            self.p2=self.p1
            self.p1=self.p5
            
            self.p4=self.p3
            self.p3=self.p6
            
            self.p5=(500+random.randint(0,500),-1000+speed)
            self.p6=(self.p5[0]+500,-1000+speed)

        mapSurf.convert()
        screen.blit(mapSurf,(0,0))
        


screen = pygame.display.set_mode(size)
mapSurf=pygame.Surface((size[0],2*size[1]))
mapSurf.set_colorkey(WHITE) # The default background color is black


clock = pygame.time.Clock()
car = Car() # create an instance
map1=Map()


while 1:
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()

    car.handle_keys() # handle the keys


    screen.fill(WHITE)
    car.test_contact(size)
    map1.update(screen,mapSurf,size,speed=10)
    car.draw(screen)
    
    
    pygame.display.update()
    clock.tick(30)
   
