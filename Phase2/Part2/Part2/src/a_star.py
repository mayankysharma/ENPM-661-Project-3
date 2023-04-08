import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
import queue
import pickle as pkl
import sys
import time
import argparse
class A_star_Proj3_Phase2():

    def __init__(self,start_pos,goal_pos,clearance,rpm_1,rpm_2):
        self.wheel_radius = 3 # cm
        self.wheel_dist = 16 # cm
        self.time_step = 0.1 # sec
        self.robot_radius = 11 #cm
        self.circle_radius=50
        self.step_size = 5
        self.total_cost = 0
        self.rpm_1=rpm_1
        self.rpm_2=rpm_2
        self.clearance=clearance
        #robot_radius = 1
        # self.clearance = 5 + self.robot_radius
        self.start_pos=start_pos
        self.goal_pos=goal_pos
        self.map = self.Create_Map() 
        map_size = self.map.shape
        self.goal_node_idx = None
        if start_pos[1] >= map_size[0]:
            start_pos[1] = map_size[0]-1
        if start_pos[0] >= map_size[1]:
            start_pos[0] = map_size[1]-1
        self.edit_start_pos = (start_pos[0]+50,map_size[0]-start_pos[1]-100-1, start_pos[2]) # edit to make it according to array index which is top left as origin to bottom left as origi
        self.actions=[[0,self.rpm_1], [self.rpm_1,0],[self.rpm_1,self.rpm_2],[0,self.rpm_2],[self.rpm_2,0],[self.rpm_2,self.rpm_2],[self.rpm_1,self.rpm_2],[self.rpm_2,self.rpm_1]]
        
        #goal_pos = [550,220] # (x,y) user input
        # because index start from 0, so handling edge cases when the pos is same as the height or width of the map
        if goal_pos[1] >= map_size[0]:
            goal_pos[1] = map_size[0]-1
        if goal_pos[0] >= map_size[1]:
            goal_pos[0] = map_size[1]-1

        self.edit_goal_pos = (goal_pos[0]+500,map_size[0]-goal_pos[1]-100-1) # edit to make it according to array index which is top left as origin to bottom left as origin

        if not self.check_nodes():
            # Exit if goal and start position is not satisfying certain condition
            sys.exit(0)

        cv2.circle(self.map, (self.edit_start_pos[0], self.edit_start_pos[1]), 5, (0,255,0), 2)
        cv2.circle(self.map, (self.edit_goal_pos[0], self.edit_goal_pos[1]), 5, (0, 0, 255), 2)
        cv2.imwrite("map.jpg", self.map)


        self.node_state = queue.PriorityQueue()
        self.parent_child_index = {0:0}
        self.visited_nodes = {0: (self.edit_start_pos, self.edit_start_pos, [0,0])}
        self.ang_interval = 30
        self.angle_range = 360//self.ang_interval


        # self.visited_map = np.zeros((self.map.shape[0]*2, self.map.shape[1]*2, self.angle_range),dtype='int')
        self.visited_map = np.zeros((self.map.shape[0]*2,self.map.shape[1]*2, self.angle_range),dtype='int')
    
    def check_nodes(self):
        column_limit = None
        row_limit = None
        Flag = True
        for c in range(self.clearance, self.map.shape[1]-self.clearance):
            su = np.sum(self.map[:,c,0]>0)
            # print(su, c)
            if su == self.map.shape[0]:
                column_limit = c

        for r in range(self.map.shape[0]):
            su = np.sum(self.map[r,:,0]>0)
            if su == self.map.shape[1]:
                row_limit = r

        if column_limit:
            print("Column Limit : ", column_limit)
            if self.edit_goal_pos[0] > column_limit and self.edit_start_pos[0] < column_limit:
                Flag = False
                print("please enter node again, not able to reach the goal")
            elif self.edit_goal_pos[0] < column_limit and self.edit_start_pos[0] > column_limit:
                print("please enter node again, not able to reach the goal")    
                Flag=False
        if row_limit:
            if self.edit_goal_pos[1] > row_limit and self.edit_start_pos[1] < row_limit:
                Flag = False
                print("please enter node again, not able to reach the goal")
            elif self.edit_goal_pos[1] < row_limit and self.edit_start_pos[1] > row_limit:
                print("please enter node again, not able to reach the goal")    
                Flag = False
        if self.map[self.edit_goal_pos[1], self.edit_goal_pos[0],0] > 0 or self.map[self.edit_start_pos[1],self.edit_start_pos[0],0] > 0:
            print("please enter node again, as it is coinciding with the obstacles")
            Flag = False
        return Flag
            

    def dist(self,A, B):
        return math.sqrt((A[0]-B[0])**2+(B[1]-A[1])**2)
    
    def isGoalNode(self,curr_node, goal_node):
        return self.dist(curr_node,goal_node)<=2
    
    def compute_cost(self,Xi,Yi,Thetai,UL,UR,map):
        """
        Xi : px 
        """
        t = 0
        dt = 0.1
        Xn=Xi
        Yn=Yi
        Thetan = 3.14 * Thetai / 180

        # Xi, Yi,Thetai: Input point's coordinates
        # Xs, Ys: Start point coordinates for plot function
        # Xn, Yn, Thetan: End point coordintes

        D=0
        while t<1: # loop for 1 sec
            t = t + dt
            Delta_Xn = 0.5*self.wheel_radius * (UL + UR) * math.cos(Thetan)/60 * dt
            Delta_Yn = 0.5*self.wheel_radius * (UL + UR) * math.sin(Thetan)/60 * dt
            Thetan += (self.wheel_radius /self.wheel_dist) * (UR - UL) * dt
            Xn += Delta_Xn
            Yn += Delta_Yn
            if map[round(Yn), round(Xn), 0]>0:
                return None
            D=D+ math.sqrt(math.pow((Delta_Xn),2)+math.pow((Delta_Yn),2))
        Thetan = 180 * (Thetan) / 3.14
        return Xn, Yn, Thetan, D

        
    def Move_Action(self,curr_pos ,map1 ,r_l ,r_r ):
        
        x,y,theta  = curr_pos

        H,W,_ = map1.shape
        val=self.compute_cost(x,y,theta,r_l,r_r,map1)
        if val is None:
            return False, curr_pos, self.visited_map
        new_x,new_y,new_angle,D = val
        
        
        new_pos = (new_x, new_y,new_angle)
        cost_to_come = D
        cost_to_goal = self.dist(new_pos, self.edit_goal_pos)
        cost = cost_to_come + cost_to_goal
        
        if new_x >= W or new_y >= H or new_x < 0 or new_y < 0: 
            return False, curr_pos, self.visited_map
        # Check if the new pos is inside the obstacle
        if map1[round(new_pos[1]), round(new_pos[0]), 0]>0:
            return False, curr_pos, self.visited_map
        
        # print("-----------")
        # print(new_angle)
        new_angle %= 360
        # print(new_angle)
        new_pos = (new_x, new_y, new_angle)

        # idx = ((360+new_angle)%360)//10 if new_angle < 0 else new_angle//10
        idx = (new_angle%360)//self.ang_interval
        # print(round(new_pos[1]*2), round(new_pos[0]*2),round(idx), new_angle)  
        if self.visited_map[round(new_pos[1]*2), round(new_pos[0]*2), round(idx)]>0:
            return False, curr_pos, self.visited_map
        self.visited_map[round(new_pos[1]*2), round(new_pos[0]*2), round(idx)] = 1
        ## can add checking for isnode condition here
        return True, (cost, new_pos), self.visited_map 
    def Create_Map(self):
        # in cm
        canvas = np.zeros((200,600,3))
        ## draw rectangles
        canvas = cv2.rectangle(canvas, pt1=(150,0), pt2=(165,125), color=(0,255,0), thickness=-1)

        ## draw rectangles
        canvas = cv2.rectangle(canvas, pt1=(250,75), pt2=(265,200), color=(0,255,0), thickness=-1)


        ## draw circle
        circle_center_coord = (400, 90)
        
        canvas = cv2.circle(canvas, circle_center_coord, self.circle_radius, (0,255,0), thickness=-1)

        ## Adding clearance
        for y in range(canvas.shape[0]):
            for x in range(canvas.shape[1]):
                # # creating boundary for upper rectangle
                if (150-self.clearance < x and 165+self.clearance > x and 125+self.clearance > y):
                    canvas[y,x,0] = 255 # blue color for clearance boundary
                
                #  creating boundary for lower rectangle
                elif (250-self.clearance < x and 265+self.clearance > x and 75-self.clearance < y):
                    canvas[y,x,0] = 255 # blue color for clearance boundary
                
                #creating boundary for circle
                if math.pow((x-circle_center_coord[0]),2)+math.pow((y-circle_center_coord[1]),2)-math.pow(self.circle_radius+self.clearance,2)<0:
                    canvas[y,x,0] = 255 # blue color for clearance boundary

                elif x<=self.clearance or 600-self.clearance<=x or self.clearance>=y or 200 - self.clearance<=y:
                    canvas[y,x,0]=255  

        return canvas
    
    def run_A_star(self):
        start_time = time.time()
        # self.node_state = queue.PriorityQueue()

        # self.visited_nodes = {0: (self.edit_start_pos, self.edit_start_pos,[0,0])}
        self.node_state.put((0,0, self.edit_start_pos))
        node_counter = 0
        
        # self.visited_map[round(self.edit_start_pos[1]*2), round(self.edit_start_pos[0]*2),round(self.edit_start_pos[2]/30)] = 1
        print("\nStarted Searching -----------") 
       
       
       
        # self.node_state.put((0,0, self.edit_start_pos))
        # node_counter = 0
        
        

        # this map will keep track of visited nodes too, by assigning value 1 to visited node.
        self.visited_map[round(self.edit_start_pos[1]*2), round(self.edit_start_pos[0]*2),round(self.edit_start_pos[2]//10)] = 1
        # actions=[[5,5], [10,10],[5,0],[0,5],[5,10],[10,5]]
        cost_to_come = 0
        while not self.node_state.empty():
            prev_cost, parent_idx, prev_pos = self.node_state.get()
            for action in self.actions:
                st, new_node_data, self.visited_map = self.Move_Action(prev_pos,self.map,action[0],action[1])
            # print(prev_cost, parent_idx, prev_pos)
            # for ang_k in range(angle_range):
            #     st, new_node_data = Clock30(curr_pos=prev_pos, map=canvas,step_size=10, visited_map=visited_map)
                if not st:
                    continue
                
                # get new node cost and pos
                new_cost,new_pos = new_node_data

            #     # increase node index and add the new node cost and state to queue
                node_counter += 1 
                # new_cost = prev_cost
                self.parent_child_index[node_counter] = parent_idx
                self.node_state.put((new_cost, node_counter, new_pos))

            #     # Keep track of visited nodes by marking them 1, which can check in 8 functions above
                # visited_map[round(new_pos[1]*2), round(new_pos[0]*2),round(new_pos[2]/30)] = 1
                self.visited_nodes[node_counter] = (prev_pos, new_pos, action)
                print("Nodes visted : ",np.sum(self.visited_map),"| COST : ", new_cost)
                # print(new_pos, self.edit_goal_pos)
                if self.isGoalNode(new_pos, self.edit_goal_pos):
                    self.total_cost = new_cost
                    self.goal_node_idx = node_counter
                    break

            if self.goal_node_idx:
                break
        print("Done Searching, Total time taken : ")
        print("--- %s seconds ---" % (round(time.time() - start_time,3)))
    
    def backtrack(self):
        print("Backtracking -------")
        s2g_pos = []
        idx = self.goal_node_idx
        indices = [idx]

        while idx!=0:
            idx = self.parent_child_index[idx]
            # print(idx)
            indices.append(idx)
        s2g_idx = sorted(indices)
        # print(s2g_idx)
        for idx in s2g_idx:
            pos =  self.visited_nodes[idx]
            # print(pos)
            s2g_pos.append(pos)
        print("Done Backtracking---------")
        return s2g_pos
    
    def record_video(self,s2g_poses):
        new_canvas = self.map.copy().astype(np.uint8)

        size = (new_canvas.shape[1],new_canvas.shape[0])
        # Below VideoWriter object will create
        # a frame of above defined The output 
        # is stored in 'filename.avi' file.
        result = cv2.VideoWriter('filename.avi', 
                                cv2.VideoWriter_fourcc(*'MJPG'),
                                600, size)

        for prev_pos, new_pos, _ in s2g_poses:
            new_canvas[int(new_pos[1]),int(new_pos[0]),2] = 255
            for action in self.actions:
            # print(prev_pos, new_pos)
                # if prev_pos==new_pos:
                #     continue
                new_canvas = cv2.arrowedLine(new_canvas, (int(prev_pos[0]), int(prev_pos[1])),(int(new_pos[0]),int(new_pos[1])),(255,255,255), thickness=1,tipLength=0.5) 
                # new_canvas = plot_curve(prev_pos[0],prev_pos[1], prev_pos[2],action[0], action[1], new_canvas)
                result.write(new_canvas)
                #print(pos)
                # Display the frame
                # saved in the file
                cv2.imshow('Frame', new_canvas)

                # Press S on keyboard 
                # to stop the process
                if cv2.waitKey(10) & 0xFF == ord('s'):
                    break
            # plt.show()
        result.release()

            
        # Closes all the frames
        cv2.destroyAllWindows()
        
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--StartPos", nargs='+', type=int, default= [0, 0, 0], help = 'start position')
    parser.add_argument("--GoalPos", nargs='+', type=int, default= [5, 0], help = 'goal position')
    parser.add_argument("--clearance", type=int, default= 5)
    parser.add_argument("--RPM1", type=int, default= 5)
    parser.add_argument("--RPM2", type=int, default= 10)
    args = parser.parse_args()
    start_pos = list(args.StartPos)
    goal_pos = list(args.GoalPos)
    clearance=(args.clearance)
    rpm_1=(args.RPM1)
    rpm_2=(args.RPM2)
    a_star=A_star_Proj3_Phase2(start_pos,goal_pos,clearance,rpm_1,rpm_2)
    a_star.run_A_star()
    sg2_poses=a_star.backtrack()
    a_star.record_video(sg2_poses)
    
    