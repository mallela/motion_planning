import Queue, math
from numpy import *
import openravepy
handles=[]

def heuristic(c_heuristic, n, g, n_gridStep, n_angleStep):
    n_gridStep= 1
    n_angleStep = 1
    if c_heuristic=='manhattan':
        return (abs(n[0]-g[0])+abs(n[1]-g[1]))/n_gridStep+abs((n[2]-g[2])/n_angleStep)
    if c_heuristic == 'eucledian':
        cost = math.sqrt(math.pow(n[0]-g[0],2)+math.pow(n[1]-g[1],2)+math.pow(n[2]-g[2],2))
        # print cost >0
        # return math.sqrt(((n[0]-g[0])/n_gridStep)**2+((n[1]-g[1])/n_gridStep)**2+((n[2]-g[2])/n_angleStep)**2)
        return cost

def getCost(n1,n2, n_gridStep, n_angleStep):
    # print math.sqrt((n1[0]-n2[0])**2+(n1[1]-n2[1])**2)
    n_gridStep= 1
    n_angleStep = 1
    cost = math.sqrt(((n1[0]-n2[0])/n_gridStep)**2+((n1[1]-n2[1])/n_gridStep)**2)#+((n1[2]-n2[2])/n_angleStep)**2)
    # print cost>0
    # return math.sqrt(((n1[0]-n2[0])/n_gridStep)**2+((n1[1]-n2[1])/n_gridStep)**2+((n1[2]-n2[2])/n_angleStep)**2)
    return cost
    # dummy = .2
    # return dummy

def getSuccessors(n, n_gridStep, n_angleStep, n_connectivity):
    global handles
    successors=[]


    successors.append(tuple((n[0], n[1],n[2]-n_angleStep)))
    successors.append(tuple((n[0], n[1],n[2]+n_angleStep)))

    successors.append(tuple((n[0]-n_gridStep, n[1],n[2])))

    successors.append(tuple((n[0]+n_gridStep, n[1],n[2])))
    
    successors.append(tuple((n[0], n[1]-n_gridStep,n[2])))
    
    successors.append(tuple((n[0], n[1]+n_gridStep,n[2])))

    if n_connectivity == 8:
        successors.append(tuple((n[0]-n_gridStep, n[1],n[2]-n_angleStep)))
        successors.append(tuple((n[0]-n_gridStep, n[1],n[2]+n_angleStep)))
        successors.append(tuple((n[0]+n_gridStep, n[1],n[2]-n_angleStep)))
        successors.append(tuple((n[0]+n_gridStep, n[1],n[2]+n_angleStep)))
        successors.append(tuple((n[0], n[1]-n_gridStep,n[2]-n_angleStep)))
        successors.append(tuple((n[0], n[1]-n_gridStep,n[2]+n_angleStep)))
        successors.append(tuple((n[0], n[1]+n_gridStep,n[2]-n_angleStep)))
        successors.append(tuple((n[0], n[1]+n_gridStep,n[2]+n_angleStep)))

        successors.append(tuple((n[0]-n_gridStep, n[1]+n_gridStep, n[2])))
        successors.append(tuple((n[0]-n_gridStep, n[1]+n_gridStep, n[2]+n_angleStep)))
        successors.append(tuple((n[0]-n_gridStep, n[1]+n_gridStep, n[2]-n_angleStep)))

        successors.append(tuple((n[0]+n_gridStep, n[1]+n_gridStep, n[2])))
        successors.append(tuple((n[0]+n_gridStep, n[1]+n_gridStep, n[2]+n_angleStep)))
        successors.append(tuple((n[0]+n_gridStep, n[1]+n_gridStep, n[2]-n_angleStep)))

        
        successors.append(tuple((n[0]-n_gridStep, n[1]-n_gridStep, n[2])))
        successors.append(tuple((n[0]-n_gridStep, n[1]-n_gridStep, n[2]+n_angleStep)))
        successors.append(tuple((n[0]-n_gridStep, n[1]-n_gridStep, n[2]-n_angleStep)))
        
        successors.append(tuple((n[0]+n_gridStep, n[1]-n_gridStep, n[2])))
        successors.append(tuple((n[0]+n_gridStep, n[1]-n_gridStep, n[2]+n_angleStep)))
        successors.append(tuple((n[0]+n_gridStep, n[1]-n_gridStep, n[2]-n_angleStep)))
    # print n
    # handles.append()
    # exit()
    # for i in successors:
    #     print i
    # print len(successors), len(set(successors))
    # exit()
    return successors

def check_collision(node,env,robot):  
    # set the robot to the location
    global handles
    robot.SetActiveDOFValues(tuple((node[0],node[1],node[2])))
    if env.CheckCollision(robot):
        handles.append(env.plot3(points=array(tuple((node[0],node[1],.05))),
               pointsize=.04,
               colors=array((1,0,0)),drawstyle=1))
        # handles.append(tuple((node[0],node[1],0.05)))
        return True
    else:
        return False

def astar(env, goalconfig, robot, n_stepSize, n_connectivity, c_heuristic, n_stepSize_deg):
    global handles
    path = []
    flagSuccess=0
    with env:
        # stepSize = .1
        fringe =  Queue.PriorityQueue()# queue of nodes to be explored
        closedSet = [] # queue of nodes already explored
        goal = tuple((goalconfig))
        start_xyth=robot.GetActiveDOFValues()
        print tuple((start_xyth))
        start = tuple((0,0,[tuple((start_xyth))] )) # ()

        fringe.put(start)

        while not fringe.empty():
            node = fringe.get()
            handles.append(env.plot3(points=array(tuple((node[2][-1][0],node[2][-1][1],0.05))),
                           pointsize=.04,
                           colors=array((0,0,1)),drawstyle=1))

            if getCost(node[2][-1],goalconfig,n_stepSize, n_stepSize_deg)<=n_stepSize and (abs(node[2][-1][2]-goalconfig[2])<=n_stepSize_deg or abs(node[2][-1][2]-goalconfig[2])<=2*pi+n_stepSize_deg):
                print "Success! Goal reached."
                path = node[2]
                break
            elif node[2][-1] not in closedSet:
                closedSet.append(node[2][-1])
                for successor in getSuccessors(node[2][-1],n_stepSize, n_stepSize_deg, n_connectivity):
                    if successor not in closedSet:
                        g = node[1]+getCost(node[2][-1],successor,n_stepSize, n_stepSize_deg)
                        h = heuristic(c_heuristic,successor, goalconfig, n_stepSize, n_stepSize_deg)
                        f = g+h
                        if not check_collision(successor,env, robot):
                            fringe.put(tuple((f,g,node[2]+[successor])))

    print "Returning path..."
    return path