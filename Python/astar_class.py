# header files
import cv2
import numpy as np 
import math
from heapq import heappush, heappop



# class for Astar
class Astar(object):
    # init function
    def __init__(self, start, goal, clearance, radius):
        self.start = start
        self.goal = goal
        self.numRows = 200
        self.numCols = 300
        self.i = 0
        self.j = 0
        self.clearance = clearance
        self.radius = radius
       
        
    # move is valid 
    def IsValid(self, currRow, currCol):
        return (currRow >= (1 + self.radius + self.clearance) and currRow <= (self.numRows - self.radius - self.clearance) and currCol >= (1 + self.radius + self.clearance) and currCol <= (self.numCols - self.radius - self.clearance))

    # checks for an obstacle
    def IsObstacle(self, row, col):
        # constants
        sum_of_c_and_r = self.clearance + self.radius
        sqrt_of_c_and_r = 1.4142 * sum_of_c_and_r
        
        # check circle
        dist1 = ((row - 150) * (row - 150) + (col - 225) * (col - 225)) - ((25 + sum_of_c_and_r) * (25 + sum_of_c_and_r))
        
        # check eclipse
        dist2 = ((((row - 100) * (row - 100)) / ((20 + sum_of_c_and_r) * (20 + sum_of_c_and_r))) + (((col - 150) * (col - 150)) / ((40 + sum_of_c_and_r) * (40 + sum_of_c_and_r)))) - 1
        
        # check triangles
        (x1, y1) = (120 - (2.62 * sum_of_c_and_r), 20 - (1.205 * sum_of_c_and_r))
        (x2, y2) = (150 - sqrt_of_c_and_r, 50)
        (x3, y3) = (185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x1 - x3)) - ((y1 - y3) * (row - x3))
        dist3 = 1
        if(first <= 0 and second <= 0 and third <= 0):
            dist3 = 0
            
        (x1, y1) = (150 - sqrt_of_c_and_r, 50)
        (x2, y2) = (185 + sum_of_c_and_r, 25 - (sum_of_c_and_r * 0.9247))
        (x3, y3) = (185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x1 - x3)) - ((y1 - y3) * (row - x3))
        dist4 = 1
        if(first >= 0 and second >= 0 and third >= 0):
            dist4 = 0
        
        # check rhombus
        (x1, y1) = (10 - sqrt_of_c_and_r, 225)
        (x2, y2) = (25, 200 - sqrt_of_c_and_r)
        (x3, y3) = (40 + sqrt_of_c_and_r, 225)
        (x4, y4) = (25, 250 + sqrt_of_c_and_r)
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3))
        fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4))
        dist5 = 1
        dist6 = 1
        if(first >= 0 and second >= 0 and third >= 0 and fourth >= 0):
            dist5 = 0
            dist6 = 0
        
        # check square
        (x1, y1) = (150 - sqrt_of_c_and_r, 50)
        (x2, y2) = (120 - sqrt_of_c_and_r, 75)
        (x3, y3) = (150, 100 + sqrt_of_c_and_r)
        (x4, y4) = (185 + sum_of_c_and_r, 75 + (sum_of_c_and_r * 0.5148))
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3))
        fourth = ((col - y4) * (x1 - x4)) - ((y1 - y4) * (row - x4))
        dist7 = 1
        dist8 = 1
        if(first <= 0 and second <= 0 and third <= 0 and fourth <= 0):
            dist7 = 0
            dist8 = 0
        
        # check rod
        first = ((col - 95) * (8.66 + sqrt_of_c_and_r)) - ((5 + sqrt_of_c_and_r) * (row - 30 + sqrt_of_c_and_r))
        second = ((col - 95) * (37.5 + sqrt_of_c_and_r)) - ((-64.95 - sqrt_of_c_and_r) * (row - 30 + sqrt_of_c_and_r))
        third = ((col - 30.05 + sqrt_of_c_and_r) * (8.65 + sqrt_of_c_and_r)) - ((5.45 + sqrt_of_c_and_r) * (row - 67.5))
        fourth = ((col - 35.5) * (-37.49 - sqrt_of_c_and_r)) - ((64.5 + sqrt_of_c_and_r) * (row - 76.15 - sqrt_of_c_and_r))
        dist9 = 1
        dist10 = 1
        if(first <= 0 and second >= 0 and third >= 0 and fourth >= 0):
            dist9 = 0
            dist10 = 0
        
        if(dist1 <= 0 or dist2 <= 0 or dist3 == 0 or dist4 == 0 or dist5 == 0 or dist6 == 0 or dist7 == 0 or dist8 == 0 or dist9 == 0 or dist10 == 0):
            return True
        return False

    # action move left
    def ActionMove30Up(self, currRow, currCol, currangle):
        theta = 30 + currangle
        if(theta >= 360):
            theta = theta - 360
        rad = theta * (math.pi/180)
        self.i = math.sin(rad)
        self.j = math.cos(rad)

        if(self.IsValid(currRow + self.i, currCol + self.j) and self.IsObstacle(currRow + self.i, currCol + self.j) == False):
            return True
        return False

    def ActionMove60Up(self, currRow,currCol, currangle):
        theta = 60 + currangle
        if(theta >= 360):
            theta = theta - 360
        rad = theta * (math.pi/180)
        self.i = math.sin(rad)
        self.j = math.cos(rad)

        if(self.IsValid(currRow + self.i, currCol + self.j) and self.IsObstacle(currRow + self.i, currCol + self.j) == False):
            return True
        return False

    def ActionMove30Down(self, currRow,currCol, currangle):
        theta = currangle - 30
        if(theta < 0):
            theta = 360 + theta
        rad = theta * (math.pi/180)
        self.i = math.sin(rad)
        self.j = math.cos(rad)

        if(self.IsValid(currRow + self.i, currCol + self.j) and self.IsObstacle(currRow + self.i, currCol + self.j) == False):
            return True
        return False

    def ActionMove60Down(self, currRow,currCol, currangle):
        theta = currangle - 60
        if(theta < 0):
            theta = 360 + theta
        rad = theta * (math.pi/180)
        self.i = math.sin(rad)
        self.j = math.cos(rad)

        if(self.IsValid(currRow + self.i, currCol + self.j) and self.IsObstacle(currRow + self.i, currCol + self.j) == False):
            return True
        return False

    def ActionMoveStraight(self, currRow,currCol, currangle):
        theta = currangle
        rad = theta * (math.pi/180)
        self.i = math.sin(rad)
        self.j = math.cos(rad)
        if(self.IsValid(currRow + self.i, currCol + self.j) and self.IsObstacle(currRow + self.i , currCol + self.j) == False):
            return True
        return False


    def CheckIfGoal(self, currRow, currCol):
        check = (((currRow - self.goal[0]) * (currRow - self.goal[0])) + ((currCol - self.goal[1]) * (currCol - self.goal[1])) - ( 1.5 * 1.5))
        if(check <= 0):
            global goalRow
            global goalCol
            goalRow = currRow
            goalCol = currCol
            print("goal reached")
            return True
        else:
            return False


    
    # astar algorithm
    def Astar(self):
        # create hashmap to store distances
        distMap = {}
        visited = {}
        path = {}
        for row in np.arange(1, self.numRows + 1, 0.5):
            for col in np.arange(1, self.numCols + 1, 0.5):
                for angle in range(0,360,30):
                    distMap[(row, col, angle)] = float('inf')
                    path[(row, col, angle)] = -1
                    visited[(row, col, angle)] = False
            
        # create queue, push the source and mark distance from source to source as zero
        explored_states = []
        queue = []
        heappush(queue, (0, self.start))
        distMap[self.start] = 0

        while(len(queue) > 0):
            _, currNode = heappop(queue)
            if visited[currNode]:
                continue
            visited[currNode] = True
            explored_states.append(currNode)

            # if goal node then exit
            if(self.CheckIfGoal(currNode[0],currNode[1]) == True):
                break

            # go through each edge of current node
            a1 = (currNode[2] + 30) % 360
            a2 = (currNode[2] + 60) % 360
            a3 = (currNode[2] - 30) % 360
            a4 = (currNode[2] - 60) % 360

            moves = [
                (self.ActionMove30Up,    a1, 1.12),
                (self.ActionMove60Up,    a2, 1.12),
                (self.ActionMove30Down,  a3, 1.12),
                (self.ActionMove60Down,  a4, 1.12),
                (self.ActionMoveStraight, currNode[2], 1.0),
            ]

            for action, next_angle, step_cost in moves:
                if action(currNode[0], currNode[1], currNode[2]):
                    nextRow = currNode[0] + round(self.i)
                    nextCol = currNode[1] + round(self.j)
                    nextNode = (nextRow, nextCol, next_angle)
                    g_next = distMap[currNode] + step_cost
                    if not visited[nextNode] and distMap[nextNode] > g_next:
                        h_next = math.sqrt((nextRow - self.goal[0])**2 + (nextCol - self.goal[1])**2)
                        distMap[nextNode] = g_next
                        path[nextNode] = currNode
                        heappush(queue, (g_next + h_next, nextNode))




        # return if no optimal path
        check = []
        f1 = self.goal[0]
        f2 = self.goal[1]
        for a in np.arange(f1-1,f1+1.5,0.5):
            for b in np.arange(f2-1,f2+1.5,0.5):
                for c in range(0,360,30):
                    check.append(distMap[a,b,c])
        for c in range(0,360,30):
            check.append(distMap[f1,f2-1.5,c])
        for c in range(0,360,30):
            check.append(distMap[f1,f2+1.5,c])
        for c in range(0,360,30):
            check.append(distMap[f1+1.5,f2,c])
        for c in range(0,360,30):
            check.append(distMap[f1-1.5,f2,c])
        NoPath = 0
        ans = float('inf')
        for a in range(len(check)):
            if(check[a] != ans):
                print("There exists a path")
                NoPath = 1
                break
        if(NoPath == 0):
            print("NO VALID PATH")
            return (explored_states, [], distMap[f1,f2,0])

        for a in range(0,360,30):
            if(distMap[goalRow,goalCol,a] != ans):
                bird = a

        print(distMap[goalRow,goalCol,bird],"answer")
        result = (goalRow, goalCol, bird)
        
        # backtrack path
        backtrack_states = []
        node = result
        while(path[node] != -1):
            backtrack_states.append(node)
            node = path[node]
        backtrack_states.append(self.start)
        backtrack_states = list(reversed(backtrack_states))
        # print(explored_states) 
        return (explored_states, backtrack_states, distMap[goalRow,goalCol,bird])
    

    # animate path
    def animate(self, explored_states, backtrack_states, path, image_path=None):
        path = str(path)

        # Create blank frame (height, width, channels)
        image = np.zeros((self.numRows, self.numCols, 3), dtype=np.uint8)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(str(path), fourcc, 20.0, (self.numCols,self.numRows))
        image = np.zeros((self.numRows, self.numCols, 3), dtype=np.uint8)


        # If MP4 failed (missing codecs), fall back to AVI
        if not out.isOpened():
            if not path.lower().endswith('.avi'):
                path = path.rsplit('.', 1)[0] + '.avi'
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            out = cv2.VideoWriter(path, fourcc, fps, (self.numCols, self.numRows))

        if not out.isOpened():
            raise RuntimeError("cv2.VideoWriter failed to open (MP4 and AVI). Install ffmpeg/gstreamer codecs or use XVID/MJPG.")

        count = 0
        for state in explored_states:
            image[int(self.numRows - state[0]), int(state[1] - 1)] = (255, 150, 0)
            if(count%75 == 0):
                out.write(image)
            count = count + 1

        count = 0
        for row in range(1, self.numRows + 1):
            for col in range(1, self.numCols + 1):
                if(image[int(self.numRows - row), int(col - 1), 0] == 0 and image[int(self.numRows - row), int(col - 1), 1] == 0 and image[int(self.numRows - row), int(col - 1), 2] == 0):
                    if(self.IsValid(row, col) and self.IsObstacle(row, col) == False):
                        image[int(self.numRows - row), int(col - 1)] = (154, 250, 0)
                        if(count%75 == 0):
                            out.write(image)
                        count = count + 1
            
        if(len(backtrack_states) > 0):
            for state in backtrack_states:
                image[int(self.numRows - state[0]), int(state[1] - 1)] = (0, 0, 255)
                out.write(image)
                cv2.imshow('result', image)
                cv2.waitKey(5)
                
        if image_path is not None:
            cv2.imwrite(str(image_path), image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        out.release()

        print("Saved video to:", path)
