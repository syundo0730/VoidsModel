#coding: UTF-8

#-------------------------------------------------------------------------------
# Name:        form
# Purpose:
#
# Author:      Syundo Kishi
#
# Created:     16/06/2013
# Copyright:   (c) syundo 2013
# Licence:     <your licence>
#-------------------------------------------------------------------------------
#!/usr/bin/env python

from matplotlib import pyplot as plt
from matplotlib import animation as animation
import numpy as np
import math

#Agentクラス
class Agent(object):
    def __init__(self):
        start = [-200,-100];
        self.node = []
        self.node.append({'p':start, 'xi':[-60,60], 'neighbor':[1,3]})
        self.node.append({'p':start, 'xi':[60,60], 'neighbor':[0,2]})
        self.node.append({'p':start, 'xi':[60,-60], 'neighbor':[1,3]})
        self.node.append({'p':start, 'xi':[-60,-60], 'neighbor':[0,2]})

#Goalの位置を表すクラス
class Goal(object):
    def __init__(self):
        self.node = [200,100]

#障害物の位置を表すクラス
class Obstacle(object):
    def __init__(self):
        self.node = []
        self.node.append([-100,-200])
        self.node.append([-100,-100])
        self.node.append([-100,0])
        self.node.append([-100,100])
        self.node.append([-100,200])
        self.node.append([-50,-150])
        self.node.append([-50,-50])
        self.node.append([-50,50])
        self.node.append([-50,150])
        self.node.append([0,-200])
        self.node.append([0,-100])
        self.node.append([0,0])
        self.node.append([0,100])
        self.node.append([0,200])
        self.node.append([50,-150])
        self.node.append([50,-50])
        self.node.append([50,50])
        self.node.append([50,150])
        self.node.append([100,-200])
        self.node.append([100,-100])
        self.node.append([100,0])
        self.node.append([100,100])
        self.node.append([100,200])
##        self.node.append([-50,100])
##        self.node.append([10,10])
##        self.node.append([100,10])
##        self.node.append([50,50])
##        self.node.append([-10,10])
##        self.node.append([-30,100])


#Agentと環境との相互作用を表すクラス
class World(object):
    def __init__(self, agent, goal, obstacle):
        self.agentnode = agent.node
        self.goalnode = goal.node
        self.obstaclenode = obstacle.node
        self.update()#初速度の設定

    def formation(self):#Agent同士の相互作用
        for x_i in self.agentnode:
            dp_x = 0
            dp_y = 0
            for j in x_i['neighbor']:
                x_j = self.agentnode[j]
                dp_x += (x_i['p'][0] - x_j['p'][0]) - (x_i['xi'][0] - x_j['xi'][0])
                dp_y += (x_i['p'][1] - x_j['p'][1]) - (x_i['xi'][1] - x_j['xi'][1])
            x_i['dp_f'] = [-dp_x, -dp_y]

    def destination(self):#ゴールとの相互作用
        for x_i in self.agentnode:
            gl = self.goalnode
            dp_x = x_i['p'][0] - gl[0]
            dp_y = x_i['p'][1] - gl[1]
            x_i['dp_g'] = [-dp_x, -dp_y]

##    def avoidance(self):#障害物との相互作用
##        dlt = 10
##        amp = 1
##        for x_i in self.agentnode:
##            dp_x = 0
##            dp_y = 0
##            for ob in self.obstaclenode:
##                l_x = x_i['p'][0] - ob[0]
##                l_y = x_i['p'][1] - ob[1]
##                gauss = amp * math.exp(- (l_x**2/(2 * dlt**2) + l_y**2/(2 * dlt**2)))
##                dp_x += l_x * gauss
##                dp_y += l_y * gauss
##            x_i['dp_a'] = [dp_x, dp_y]

    def avoidance(self):
        for x_i in self.agentnode:
            dp_x = 0
            dp_y = 0
            for ob in self.obstaclenode:
                dis_x = x_i['p'][0] - ob[0]
                dis_y = x_i['p'][1] - ob[1]
                dis = math.sqrt(dis_x**2 + dis_y**2)
                threshold = 30
                if dis < threshold:
                    dp_x += (x_i['p'][0] - ob[0]) - (threshold/dis)*dis_x
                    dp_y += (x_i['p'][1] - ob[1]) - (threshold/dis)*dis_y
            x_i['dp_a'] = [-dp_x, -dp_y]

    def update(self):#相互作用ごとに速度を更新する
        self.formation()#フォーメーション制御による速度項
        self.destination()#ゴール制御による速度項
        self.avoidance()#障害物回避制御による速度項

#Worldクラスをコントロールするクラス
class Controlor(object):
    def __init__(self):
        self.time = 0
        self.timestep = 0.01
        self.world = World(Agent(), Goal(), Obstacle())

    #Agentの位置を更新する
    def update(self, oldagentnode, newagentnode):
        dt = self.timestep
        length =  len(oldagentnode)
        for i in range(length):
            on = oldagentnode[i]
            np_x = on['p'][0] + dt * (0.5*on['dp_f'][0] + 1.1*on['dp_g'][0] + 100*on['dp_a'][0])
            np_y = on['p'][1] + dt * (0.5*on['dp_f'][1] + 1.1*on['dp_g'][1] + 100*on['dp_a'][1])
            self.time += dt
            newagentnode[i]['p'] = [np_x, np_y]
            self.world.update()

    #描画の更新用
    def step(self):
        self.update(self.world.agentnode, self.world.agentnode)

    #Agentの位置リストをタプルとして返す
    def agent_pos(self):
        pos_x = []
        pos_y = []

        for i in self.world.agentnode:
            pos_x.append(i['p'][0])
            pos_y.append(i['p'][1])

        pos_x.append(self.world.agentnode[0]['p'][0])
        pos_y.append(self.world.agentnode[0]['p'][1])

        return (pos_x, pos_y)

    #障害物の位置リストをタプルとして返す
    def obstacle_pos(self):
        pos_x = []
        pos_y = []
        for i in self.world.obstaclenode:
            pos_x.append(i[0])
            pos_y.append(i[1])

        return (pos_x, pos_y)

#コントローラの生成
cont = Controlor()

#------------------------------------------------------------
# set up figure and animation
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                     xlim=(-300, 300), ylim=(-300, 300))
ax.grid()

points, = ax.plot([], [], 'o-', lw=2)
obs, = ax.plot([], [], 'o', lw=2)
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

def init():
    """initialize animation"""
    points.set_data([], [])
    obs.set_data(*cont.obstacle_pos())
    time_text.set_text('')
    return points, obs, time_text

def animate(i):
    """perform animation step"""
    cont.step()
    points.set_data(*cont.agent_pos())
    time_text.set_text('time = %.1f' % cont.time)
    #plt.savefig('fig/plot'+str(cont.time)+'.eps', format = 'eps')
    return points, time_text

# choose the interval based on dt and the time to animate one step
from time import time
t0 = time()
animate(0)
t1 = time()
interval = 1000 * 0.01 - (t1 - t0)

ani = animation.FuncAnimation(fig, animate, frames=24,
                              interval=interval, blit=True, init_func=init)



plt.show()