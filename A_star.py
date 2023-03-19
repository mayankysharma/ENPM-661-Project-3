import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
import queue
import math
import argparse
import time
import sys


class A_star():
    def __init__(self,start_pos,goal_pos,robot_radius,step_size):
        self.robot_radius=robot_radius
        self.step_size = step_size
        self.total_cost = 0
        self.goal_node_idx = None
        #robot_radius = 1
        self.canvas = np.zeros((250,600,3))
        self.clearance = 5 + robot_radius
        self.start_pos=start_pos
        self.goal_pos=goal_pos 
        map_size = self.canvas.shape
        
        #start_pos = [6,6] # (x,y) user input
        # because index start from 0, so handling edge cases when the pos is same as the height or width of the map
        if start_pos[1] >= map_size[0]:
            start_pos[1] = map_size[0]-1
        if start_pos[0] >= map_size[1]:
            start_pos[0] = map_size[1]-1
        self.edit_start_pos = (start_pos[0],map_size[0]-start_pos[1]-1, start_pos[2]) # edit to make it according to array index which is top left as origin to bottom left as origi
        #goal_pos = [550,220] # (x,y) user input
        # because index start from 0, so handling edge cases when the pos is same as the height or width of the map
        if goal_pos[1] >= map_size[0]:
            goal_pos[1] = map_size[0]-1
        if goal_pos[0] >= map_size[1]:
            goal_pos[0] = map_size[1]-1

        self.edit_goal_pos = (goal_pos[0],map_size[0]-goal_pos[1]-1, goal_pos[2]) # edit to make it according to array index which is top left as origin to bottom left as origin

        if not self.check_nodes():
            # Exit if goal and start position is not satisfying certain condition
            sys.exit(0)

        self.node_state = queue.PriorityQueue()
        self.parent_child_index = {0:0}
        self.visited_nodes = {0: (self.edit_start_pos, self.edit_start_pos)}
        self.ang_interval = 30
        self.angle_range = 360//self.ang_interval


        self.visited_map = np.zeros((self.canvas.shape[0]*2, self.canvas.shape[1]*2, self.angle_range),dtype='int')
        


    def get_line_equation(self,coord1, coord2, map_coord, clearance, inverse = False):
        # reference : https://byjus.com/jee/distance-between-2-parallel-lines/
        m = (coord2[1]-coord1[1])/(coord2[0]-coord1[0])
        c = coord1[1] - m * coord1[0]

        if inverse:
            # new equation of line, distance = |c1 - c2|/sqrt(1+m^2); distance between parallel lines 
            c2 = c - clearance * math.sqrt((1+m**2)) 
        else:
            # new equation of line, distance = |c2 - c1|/sqrt(1+m^2); distance between parallel lines 
            c2 = clearance * math.sqrt((1+m**2)) + c

        return map_coord[1] - m*map_coord[0] - c2
    def dist(self,A, B):
        return math.sqrt((A[0]-B[0])**2+(B[1]-A[1])**2)
            
    def isGoalNode(self,curr_node, goal_node):
        return self.dist(curr_node,goal_node)<=1.5 and goal_node[2] == curr_node[2]
    

    def check_nodes(self):
        """
        return :
            True, if everything is fine, and robot can reach to goal.
            else,
                False
        """

        column_limit = None
        row_limit = None
        Flag = True


        for c in range(self.clearance, self.canvas.shape[1]-self.clearance):
            su = np.sum(self.canvas[:,c,0]>0)
            # print(su, c)
            if su == self.canvas.shape[0]:
                column_limit = c

        for r in range(self.clearance, self.canvas.shape[0]-self.clearance):
            su = np.sum(self.canvas[r,:,0]>0)
            if su == self.canvas.shape[1]:
                row_limit = r

        # Check whether robot can reach goal given start and goal pos
        # if any column fully occupied, then check whether it is dividing start and goal pos 
        if column_limit:
            print("Column Limit : ", column_limit)
            if self.edit_goal_pos[0] > column_limit and self.edit_start_pos[0] < column_limit:
                Flag = False
                print("please enter node again, not able to reach the goal")
                
            elif self.edit_goal_pos[0] < column_limit and self.edit_start_pos[0] > column_limit:
                Flag = False
                print("please enter node again, not able to reach the goal")    

        # if any row fully occupied, then check whether it is dividing start and goal pos 
        if row_limit:
            if self.edit_goal_pos[1] > column_limit and self.edit_start_pos[1] < column_limit:
                Flag = False
                print("please enter node again, not able to reach the goal")
                
            elif self.edit_goal_pos[1] < column_limit and self.edit_start_pos[1] > column_limit:
                Flag = False
                print("please enter node again, not able to reach the goal")    

        if self.canvas[self.edit_goal_pos[1], self.edit_goal_pos[0],0] > 0 or self.canvas[self.edit_start_pos[1],self.edit_start_pos[0],0] > 0:
            Flag = False
            print("please enter node again, as it is coinciding with the obstacles")
        
        return Flag
    
    
    def Create_Map(self):
        canvas = np.zeros((250,600,3))
        clearance = 5 + self.robot_radius

        ## draw rectangles
        canvas = cv2.rectangle(canvas, pt1=(100,0), pt2=(150,100), color=(0,255,0), thickness=-1)

        ## draw rectangles
        canvas = cv2.rectangle(canvas, pt1=(100,150), pt2=(150,250), color=(0,255,0), thickness=-1)

        ## draw hexagone
        pts = np.array([
            [150+150-65, 87],
            [150+150-65, 162],
            [150+150,125+75],
            [150+150+65, 162],
            [150+150+65, 87],
            [150+150,125-75]],dtype=np.int32)
        # canvas = cv2.polylines(canvas,[pts],isClosed=True, thickness=1, color=(150,0,0))
        canvas = cv2.fillPoly(canvas,[pts],color=(0,255,0))

        ## draw triangle
        pts = np.array([
            [300+160, 25],
            [300+160, 225],
            [300+160+50,125]],dtype=np.int32)
        # canvas = cv2.polylines(canvas,[pts],isClosed=True, thickness=1, color=(150,0,0))
        canvas = cv2.fillPoly(canvas,[pts], color=(0,255,0))


        ## Adding clearance
        for y in range(canvas.shape[0]):
            for x in range(canvas.shape[1]):
                # creating boundary for upper rectangle
                if (100-clearance < x and 150+clearance > x and 100+clearance > y):
                    canvas[y,x,0] = 255 # blue color for clearance boundary
                
                #  creating boundary for lower rectangle
                elif (100-clearance < x and 150+clearance > x and 250-100-clearance < y):
                    canvas[y,x,0] = 255 # blue color for clearance boundary
                
                # creating boundary for hexagon
                elif (
                    (235-clearance < x and 365+clearance > x) # xaxis 
                    and (self.get_line_equation((235,87),(300,50),(x,y),clearance,inverse=True)>0) # y must be less in this eq
                    and (self.get_line_equation((300,50),(365,87),(x,y),clearance,inverse=True)>0) # y must be less in this eq
                    and (self.get_line_equation((235,162),(300,200),(x,y),clearance)<0) # y must be large wrt x in this eq
                    and (self.get_line_equation((300,200),(365,162),(x,y),clearance)<0) # y must be large wrt x in this eq
                    ):
                    canvas[y,x,0] = 255 # blue color for clearance boundary
                elif (
                    (460-clearance < x and 510+clearance > x) # xaxis 
                    and (self.get_line_equation((460,25),(510,125),(x,y),clearance,inverse=True)>0) # y must be less in this eq
                    and (self.get_line_equation((460,225),(510,125),(x,y),clearance)<0) # y must be less in this eq
                    ):
                    canvas[y,x,0] = 255 # blue color for clearance boundary
                elif x<=clearance or 600-clearance<=x or clearance>=y or 250 - clearance<=y:
                    canvas[y,x,0]=255  

        plt.imshow(canvas)
        cv2.imwrite("map.jpg",canvas)
    
    def Clock60(self,curr_pos : tuple, map : np.ndarray,step_size):
        
        x,y,theta  = curr_pos
        
        new_angle = theta - 60
        if new_angle <= -360 :
            new_angle = new_angle + 360
        H,W,_ = map.shape
        new_x = step_size*math.cos(math.radians(new_angle)) + x
        new_y = step_size*math.sin(math.radians(new_angle)) + y
        new_pos = (new_x, new_y,new_angle)
        # cost_to_come = dist(new_pos, edit_start_pos)
        cost_to_come = 0
        cost_to_goal = self.dist(new_pos, self.edit_goal_pos)

        cost = cost_to_come + cost_to_goal
        if new_x >= W or new_y >= H or new_x < 0 or new_y < 0: 
            return False, curr_pos
        # Check if the new pos is inside the obstacle
        if map[round(new_pos[1]), round(new_pos[0]),0]>0:
            return False, curr_pos

        # print(new_angle)
        # avoid overshooting beyond 360
        idx = (360+new_angle)/30 -1 if new_angle < 0 else new_angle/30 - 1 
        if self.visited_map[round(new_pos[1]*2), round(new_pos[0]*2),round(idx)]>0:
            return False, curr_pos
        self.visited_map[round(new_pos[1]*2), round(new_pos[0]*2),round(idx)] = 1
        ## can add checking for isnode condition here
        return True, (cost, new_pos)

    def AntiClock60(self,curr_pos : tuple, map : np.ndarray,step_size):

        x,y,theta  = curr_pos
        new_angle = theta + 60
        if new_angle >= 360 :
            new_angle = new_angle - 360
        H,W,_ = map.shape
        new_x = step_size*math.cos(math.radians(new_angle)) + x
        new_y = step_size*math.sin(math.radians(new_angle)) + y
        new_pos = (new_x, new_y,new_angle)
        # cost_to_come = dist(new_pos, edit_start_pos)
        cost_to_come = 0
        cost_to_goal = self.dist(new_pos, self.edit_goal_pos)

        cost = cost_to_come + cost_to_goal
        if new_x >= W or new_y >= H or new_x < 0 or new_y < 0: 
            return False, curr_pos
        # Check if the new pos is inside the obstacle
        if map[round(new_pos[1]), round(new_pos[0]),0]>0:
            return False, curr_pos

        idx = (360+new_angle)/30 -1 if new_angle < 0 else new_angle/30 - 1 
        if self.visited_map[round(new_pos[1]*2), round(new_pos[0]*2),round(idx)]>0:
            return False, curr_pos
        self.visited_map[round(new_pos[1]*2), round(new_pos[0]*2),round(idx)] = 1
        ## can add checking for isnode condition here
        return True, (cost, new_pos)
 
    def AntiClock30(self,curr_pos : tuple, map : np.ndarray,step_size):
        
        x,y,theta  = curr_pos
        
        new_angle = theta + 30
        if new_angle >= 360 :
            new_angle = new_angle - 360
        H,W,_ = map.shape
        new_x = step_size*math.cos(math.radians(new_angle)) + x
        new_y = step_size*math.sin(math.radians(new_angle)) + y
        new_pos = (new_x, new_y,new_angle)
        # cost_to_come = dist(new_pos, edit_start_pos)
        cost_to_come = 0
        cost_to_goal = self.dist(new_pos, self.edit_goal_pos)

        cost = cost_to_come + cost_to_goal
        if new_x >= W or new_y >= H or new_x < 0 or new_y < 0: 
            return False, curr_pos
        # Check if the new pos is inside the obstacle
        if map[round(new_pos[1]), round(new_pos[0]),0]>0:
            return False, curr_pos

        idx = (360+new_angle)/30 -1 if new_angle < 0 else new_angle/30 - 1 
        if self.visited_map[round(new_pos[1]*2), round(new_pos[0]*2),round(idx)]>0:
            return False, curr_pos
        self.visited_map[round(new_pos[1]*2), round(new_pos[0]*2),round(idx)] = 1
        ## can add checking for isnode condition here
        return True, (cost, new_pos)
        
    def Clock30(self,curr_pos : tuple, map : np.ndarray,step_size):
        
        x,y,theta = curr_pos
        new_angle = theta - 30
        if new_angle <= -360 :
            new_angle = new_angle + 360
        H,W,_ = map.shape
        # print("New",new_angle)
        new_x = step_size*math.cos(math.radians(new_angle)) + x
        new_y = step_size*math.sin(math.radians(new_angle)) + y
        new_pos = (new_x, new_y,new_angle)
        # cost_to_come = dist(new_pos, edit_start_pos)
        cost_to_come = 0
        cost_to_goal = self.dist(new_pos, self.edit_goal_pos)

        cost = cost_to_come + cost_to_goal

        idx = (360+new_angle)/30 -1 if new_angle < 0 else new_angle/30 - 1 
        if self.visited_map[round(new_pos[1]*2), round(new_pos[0]*2),round(idx)]>0:
            return False, curr_pos
        self.visited_map[round(new_pos[1]*2), round(new_pos[0]*2),round(idx)] = 1
        # print(new_pos)
        ## can add checking for isnode condition here
        return True, (cost, new_pos)   

    def moveForward(self,curr_pos : tuple, map : np.ndarray, step_size):
    
    
        x,y,theta  = curr_pos
        new_angle = 0+theta
        H,W,_ = map.shape
        new_x = step_size*math.cos(math.radians(new_angle)) + x
        new_y = step_size*math.sin(math.radians(new_angle)) + y
        new_pos = (new_x, new_y,new_angle)
        # cost_to_come = dist(new_pos, edit_start_pos)
        cost_to_come = 0
        cost_to_goal = self.dist(new_pos, self.edit_goal_pos)

        cost = cost_to_come + cost_to_goal
        if new_x >= W or new_y >= H or new_x < 0 or new_y < 0: 
            return False, curr_pos
        # Check if the new pos is inside the obstacle
        if map[round(new_pos[1]), round(new_pos[0]),0]>0:
            return False, curr_pos

        idx = (360+new_angle)/30 -1 if new_angle < 0 else new_angle/30 - 1 
        if self.visited_map[round(new_pos[1]*2), round(new_pos[0]*2),round(idx)]>0:
            return False, curr_pos
        self.visited_map[round(new_pos[1]*2), round(new_pos[0]*2),round(idx)] = 1
        ## can add checking for isnode condition here
        # print(new_pos)
        return True, (cost, new_pos)
    
    def run_A_star(self):
        start_time = time.time()
        self.node_state = queue.PriorityQueue()
        self.parent_child_index = {0:0}
        self.visited_nodes = {0: (self.edit_start_pos, self.edit_start_pos)}
        ang_interval = 30
        angle_range = 360//ang_interval
        self.visited_map = np.zeros((self.canvas.shape[0]*2, self.canvas.shape[1]*2, angle_range),dtype='int') 
    

        self.node_state.put((0,0, self.edit_start_pos))
        node_counter = 0
        self.goal_node_idx = None
        self.total_cost = 0

        # this map will keep track of visited nodes too, by assigning value 1 to visited node.
        self.visited_map[round(self.edit_start_pos[1]*2), round(self.edit_start_pos[0]*2),round(self.edit_start_pos[2]/30)] = 1


        while not self.node_state.empty():
            prev_cost, parent_idx, prev_pos = self.node_state.get()
            for func in [self.Clock60,self.moveForward,self.Clock30,self.AntiClock30,self.AntiClock60]:
                st, new_node_data=func(curr_pos=prev_pos, map=self.canvas,step_size=5)
            # print(prev_cost, parent_idx, prev_pos)
            # for ang_k in range(angle_range):
            #     st, new_node_data = Clock30(curr_pos=prev_pos, map=canvas,step_size=10, visited_map=visited_map)
                if not st:
                    continue
                
                # get new node cost and pos
                new_cost,new_pos = new_node_data
            
                flag = False
                for visited_pos in self.visited_nodes.values():
                    if visited_pos == new_pos:
                        flag=True
                        break
                if flag:
                    continue
                
            #     # increase node index and add the new node cost and state to queue
                node_counter += 1 
                # new_cost = prev_cost
                self.parent_child_index[node_counter] = parent_idx
                self.node_state.put((new_cost, node_counter, new_pos))

            #     # Keep track of visited nodes by marking them 1, which can check in 8 functions above
                # visited_map[round(new_pos[1]*2), round(new_pos[0]*2),round(new_pos[2]/30)] = 1
                self.visited_nodes[node_counter] = (prev_pos, new_pos)
                print(np.sum(self.visited_map), new_cost)

                if self.isGoalNode(new_pos, self.edit_goal_pos):
                    total_cost = new_cost
                    self.goal_node_idx = node_counter
                    
                    break
            #     # break
            # # break

            if self.goal_node_idx:
                break
        print("--- %s seconds ---" % (time.time() - start_time))    
        
    def backtrack(self,goal_node_idx, parent_child_index, visited_nodes, map):
        print("Backtracking -------")
        s2g_pos = []
        idx = goal_node_idx
        indices = [idx]

        while idx!=0:
            idx = parent_child_index[idx]
            # print(idx)
            indices.append(idx)
        s2g_idx = sorted(indices)
        # print(s2g_idx)
        for idx in s2g_idx:
            pos =  visited_nodes[idx]
            # print(pos)
            s2g_pos.append(pos)
        print("Done Backtracking---------")
        return s2g_pos
    def record_video(self):
        
        s2g_poses = self.backtrack(self.goal_node_idx, self.parent_child_index, self.visited_nodes, self.canvas)    
        new_canvas = self.canvas.copy().astype(np.uint8)
        size = (new_canvas.shape[1],new_canvas.shape[0])
        # Below VideoWriter object will create
        # a frame of above defined The output 
        # is stored in 'filename.avi' file.
        result = cv2.VideoWriter('node_exploration.avi', 
                                cv2.VideoWriter_fourcc(*'MJPG'),
                                100, size)
        # s2g_poses = backtrack(goal_node_idx, parent_child_index, visited_nodes, canvas)

        for prev_pos, new_pos in self.visited_nodes.values():
            print(prev_pos, new_pos)
            new_canvas[int(new_pos[1]),int(new_pos[0]),2] = 255
            new_canvas = cv2.arrowedLine(new_canvas, (int(prev_pos[0]), int(prev_pos[1])),(int(new_pos[0]),int(new_pos[1])),(255,255,255), thickness=1,tipLength=0.5) 
            result.write(new_canvas)
            #print(pos)
            # Display the frame
            # saved in the file
            cv2.imshow('Frame', new_canvas)
            cv2.waitKey(1)
            # Press S on keyboard 
            # to stop the process
            if cv2.waitKey(1) & 0xFF == ord('s'):
                break
        # plt.show()
        result.release()

            
        # Closes all the frames
        cv2.destroyAllWindows()
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--StartPos", nargs='+', type=int, default= [0, 0, 0], help = 'start position')
    parser.add_argument("--GoalPos", nargs='+', type=int, default= [350, 250,0], help = 'goal position')
    parser.add_argument("--StepSize", type=int, default= 5, help = 'Step size: 1-10')
    parser.add_argument("--RobotRadius", type=int, default= 5)
    args = parser.parse_args()
    start_pos = list(args.StartPos)
    goal_pos = list(args.GoalPos)
    step_size=(args.StepSize)
    robot_radius=(args.RobotRadius)
    print("Start Pos recieved : ", start_pos)
    print("Goal Pos recieved : ", goal_pos)
    a_star=A_star(start_pos,goal_pos,robot_radius,step_size)
    # a_star.take_input()
    a_star.Create_Map()
    a_star.run_A_star()
    a_star.record_video()
    