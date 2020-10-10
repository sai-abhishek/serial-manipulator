import numpy as np
import sys
import time
import pygame


G = [[100.0], [200.0]]  # 2d array to define goal

L = 150  # length of arm
Kp = 0.01  # proportionality const. play around with this
lamb = 100.0  # proportionality constant
tolerance = 1


# dome definitions for calculations
de = [[0.0], [0.0]]  # 2D array
dtheta = [[0.0], [0.0]]
E = [[0.0], [0.0]]


angle = [0, 0]                        # to calculate sum of angles in Fkine
Jangle = [0, 0]                       # ^^for jacobian
angle_comp = [0.0, 0.0]               # first element= sum of cos components
# second element = sum of sine components

Jangle_comp = [0.0, 0.0]                # same as ^^ but for jacobian


e = []
J = [[], []]

error_mag = 2

# pygame window definitons
background_colour = (0, 174, 239)
(width, height) = (720, 640)

screen = pygame.display.set_mode((width, height))
linecolor = 255, 255, 255
endcolor = 255, 0, 0
pygame.display.set_caption("Robotics Arm - Inverse Kinematics")
screen.fill(background_colour)

links = 2
theta = []
for i in range(links):
    theta.append(np.deg2rad(0.0))

# theta = [np.deg2rad(0.0),np.deg2rad(0.0)] #array of joint initial joint parameters
    # add more or remove some random elements
    # no of links depends on the length if this array

# FK definition


def fkine(theta):
    for j in range(len(theta)):
        for i in range(len(theta)-j):
            angle[0] += theta[i]
            angle_comp[0] += np.cos(angle[0])
            angle_comp[1] += np.sin(angle[0])

        e.append([L*angle_comp[0], L*angle_comp[1]])
        angle_comp[0] = 0.0
        angle_comp[1] = 0.0
        angle[0] = 0.0
        angle[1] = 0.0
    e.append([0.0, 0.0])

# compute jacobian


def jacobian(theta):
    for k in range(len(theta)):
        for j in range(len(theta)-k):
            for i in range(len(theta)-j):
                Jangle[0] += theta[i]
            Jangle_comp[0] += np.sin(Jangle[0])
            Jangle_comp[1] += np.cos(Jangle[0])
            Jangle[0] = 0.0

        J[0].append((-L)*Jangle_comp[0])
        J[1].append(L*Jangle_comp[1])
        Jangle_comp[0] = 0.0
        Jangle_comp[1] = 0.0


running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            pygame.quit()             # event handeler to get mouse position
            sys.exit()
        if event.type == pygame.MOUSEBUTTONDOWN:
            (mouseX, mouseY) = pygame.mouse.get_pos()

            # redefining goal to mouse clicked position
            G[0][0] = mouseX - width/2
            G[1][0] = mouseY - height/2

            k = 0
            while error_mag > tolerance and k < 1000:
                k += 1
                fkine(theta)
                jacobian(theta)
                # Jinv=np.linalg.pinv(np.asmatrix(J))

                Jinv = np.matrix.transpose(np.mat(J)) * np.linalg.inv(
                    (np.mat(J) * np.matrix.transpose(np.mat(J)) + lamb*np.identity(2)))  # DLS

                de[0][0] = Kp*(G[0][0] - e[0][0])
                de[1][0] = Kp*(G[1][0] - e[0][1])

                dtheta = np.asarray(np.transpose(
                    np.dot(np.asarray(Jinv), np.asarray(de))))

                for i in range(len(theta)):
                    theta[i] += dtheta[0][i]

                E[0][0] = G[0][0] - e[0][0]  # error in x
                E[1][0] = G[1][0] - e[0][1]  # error in y

                error_mag = np.sqrt(np.square(E[0][0]) + np.square(E[1][0]))

                for i in range(len(theta)):
                    pygame.draw.line(screen, linecolor, (e[i][0] + width/2, e[i]
                                                         [1] + height/2), (e[i+1][0] + width/2, e[i+1][1] + height/2), 5)
                    pygame.draw.circle(screen, endcolor, (int(
                        e[0][0] + width/2), int(e[0][1] + height/2)), 6, 0)
                pygame.draw.circle(screen, endcolor, (int(width/2),
                                                      int(height/2)), L*len(theta), 1)
                pygame.display.flip()
                time.sleep(0.001)
                screen.fill(background_colour)

                e = []
                J = [[], []]

            error_mag = 2
