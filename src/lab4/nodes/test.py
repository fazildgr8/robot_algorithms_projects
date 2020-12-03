import math
class Node:
    def __init__(self,position):
        self.location = position
        self.prev = None
        self.g_cost = 0
        self.h_cost = 0
        self.f_cost = 0
        self.Occupied = False
    def update_cost(self,g,h):
        self.g_cost = g
        self.h_cost = h
        self.f_cost = g+h
    def __eq__(self, node):
        return self.location == node.location

def Distance_compute(pos1,pos2,Type = 'd'):
    x1 = pos1[0]
    y1 = pos1[1]
    x2 = pos2[0]
    y2 = pos2[1]
    d = ((x1-x2)**2) + ((y1-y2)**2)
    if Type == 'd':
        return math.sqrt(d)
    if Type == 'eu':
        return d
    if Type == 'manhattan':
        return abs(x1-x2)+abs(y1-y2)

def A_STAR(global_map, start, end, Type = '8c'):
    start_node = Node(start)
    end_node = Node(end)
    open_list = [start_node]
    closed_nodes = []

    while open_list:
        current_node = open_list[0]
        index = 0
        for i, x in enumerate(open_list):
            if x.f_cost < current_node.f_cost:
                current_node = x
                index = i
        open_list.pop(index)
        closed_nodes.append(current_node)

        if current_node == end_node:
            path = []
            node = current_node
            while node is not None:
                path.append(node.location)
                node = node.prev
            return path[::-1]


        neibhours = []
        i_list = [0,0,-1,1,-1,-1,1,1]
        j_list = [-1,1,0,0,-1,1,-1,1]
        if(Type=='4c'):
            i_list = [0,0,1,-1]
            j_list = [1,-1,0,0]


        for k in range(len(i_list)):
            node_pos = [current_node.location[0]+i_list[k],current_node.location[1]+j_list[k]]
            if (node_pos[0] > (len(global_map) - 1) or node_pos[0] < 0 or
               (node_pos[1] > len(global_map[node_pos[0]]) - 1) or node_pos[1] < 0):
               continue

            if global_map[node_pos[0]][node_pos[1]] != 0:
                continue

            neibhour_node = Node((node_pos[0], node_pos[1]))
            neibhour_node.prev = current_node
            neibhours.append(neibhour_node)

        for neibhour in neibhours:
            if neibhour in closed_nodes: continue

            g = current_node.g_cost + 1
            h = Distance_compute(neibhour.location,end_node.location,'eu')
            neibhour.update_cost(g,h)
            for onode in open_list:
                if neibhour == onode and neibhour.g_cost > onode.g_cost:
                    continue


            open_list.append(neibhour)


def main():

    maze = [[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,],
            [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,],
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,],
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,],
            [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,],
            [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,],
            [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,],
            [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,],
            [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,],
            [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,],
            [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,],
            [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,],
            [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,],
            [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,],
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,],
            [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,],
            [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,],
            [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,],
            [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,],
            [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]


    print(A_STAR(maze,(1,8),(13,17),'8c'))


if __name__ == '__main__':
    main()