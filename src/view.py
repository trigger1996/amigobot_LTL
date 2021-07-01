
import networkx as nx
from lomap.classes import ts
import numpy as np
from   matplotlib import pyplot as plt
from   matplotlib import animation

pos_ref = { "1" : [0,   0.1],
            "2" : [0,   0.2],
            "3" : [0,   0.8],
            "4" : [0,   0.9],
            "5" : [0.1, 0.9],
            "6" : [0.9, 0.9],
            "7" : [1,   0.9],
            "8" : [1,   0.8],
            "9" : [1,   0.2],
            "10" : [1,  0.1],
            "11" : [0.9, 0.1],
            "12" : [0.1, 0.1],
            "21" : [0.1, 0.2],
            "22" : [0.5, 0.2],
            "23" : [0.9, 0.2],
            "24" : [0.9, 0.5],
            "25" : [0.9, 0.8],
            "26" : [0.5, 0.8],
            "27" : [0.1, 0.8],
            "28" : [0.1, 0.5],
            "u1" : [0, 1],
            "u2" : [1, 0],
            "g1" : [0.5, 0.3],
            "g2" : [0.7, 0.5],
            "g3" : [0.5, 0.7],
            "g4" : [0.3, 0.5] }

# https://blog.csdn.net/sinat_36219858/article/details/79800460
dot_color_ref = ['ro', 'b*', 'gv']

def visualize_run(tran_sys, run, edgelabel='control', draw='matplotlib'):
    """
    Visualizes a LOMAP system model with run.
    """
    assert edgelabel is None or nx.is_weighted(tran_sys.g, weight=edgelabel)
    if draw == 'pygraphviz':
        nx.view_pygraphviz(tran_sys.g, edgelabel)
    elif draw == 'matplotlib':
        pos = nx.get_node_attributes(tran_sys.g, 'location')
        if len(pos) != tran_sys.g.number_of_nodes():
            pos = nx.spring_layout(tran_sys.g)

        # because the map is the same
        # add map (drawn before)
        pos = pos_ref

        # add color (set before)
        # https://blog.csdn.net/qq_26376175/article/details/67637151
        node_colors = dict([(v, 'yellowgreen') for v in tran_sys.g])
        node_colors['u1'] = node_colors['u2'] = 'tomato'
        node_colors['g1'] = node_colors['g2'] = node_colors['g3'] = node_colors['g4'] = 'cornflowerblue'
        node_colors = list(node_colors.values())

        # edge color
        color_map = []
        index = 0
        for u, v in tran_sys.g.edges():
            color_map.append('black')
            index += 1
            for i in range(1, run.__len__()):
                if u == run[i - 1] and v == run[i]:
                    color_map[index - 1] = 'blue'
                    break

        nx.draw(tran_sys.g, pos=pos, node_color=node_colors, edge_color=color_map)
        nx.draw_networkx_labels(tran_sys.g, pos=pos)
        edge_labels = nx.get_edge_attributes(tran_sys.g, edgelabel)

        #
        edge_labels_to_draw = []
        for (n1, n2) in edge_labels.items():
            edge_labels_to_draw.append(((n1[0], n1[1]), n2))
        edge_labels_to_draw = dict(edge_labels_to_draw)

        nx.draw_networkx_edge_labels(tran_sys.g, pos=pos,
                                     edge_labels=edge_labels_to_draw)  # edge_labels

        plt.show()

def visualize_animation(tran_sys, run):
    fig, ax = plt.subplots()
    dot, = ax.plot([], [], 'ro')

    robot_example = ts.Ts()
    ts_run = []

    def gen_dot():
        for i in range(1, ts_run.__len__()):
            x_start, y_start = pos_ref[ts_run[i - 1]]
            x_end, y_end = pos_ref[ts_run[i]]
            weight = robot_example.g.edge[ts_run[i - 1]][ts_run[i]][0]['weight']
            kx = (x_end - x_start) / (5 * weight);
            ky = (y_end - y_start) / (
                        5 * weight);  # WARNING: weight of edges / vehicles is not taken into considerations
            for j in range(0, (5 * weight)):
                x = kx * j + x_start
                y = ky * j + y_start
                newdot = [x, y]
                yield newdot

    def update_dot(newd):
        dot.set_data(newd[0], newd[1])
        return dot,

    edgelabel = 'control'
    pos = nx.get_node_attributes(tran_sys.g, 'location')
    if len(pos) != tran_sys.g.number_of_nodes():
        pos = nx.spring_layout(tran_sys.g)

    # because the map is the same
    # add map (drawn before)
    pos = pos_ref

    # add color (set before)
    # https://blog.csdn.net/qq_26376175/article/details/67637151
    node_colors = dict([(v, 'yellowgreen') for v in tran_sys.g])
    node_colors['u1'] = node_colors['u2'] = 'tomato'
    node_colors['g1'] = node_colors['g2'] = node_colors['g3'] = node_colors['g4'] = 'cornflowerblue'
    node_colors = list(node_colors.values())

    # edge color
    color_map = 'black'

    nx.draw(tran_sys.g, pos=pos, node_color=node_colors, edge_color=color_map)
    nx.draw_networkx_labels(tran_sys.g, pos=pos)
    edge_labels = nx.get_edge_attributes(tran_sys.g, edgelabel)

    #
    edge_labels_to_draw = []
    for (n1, n2) in edge_labels.items():
        edge_labels_to_draw.append(((n1[0], n1[1]), n2))
    edge_labels_to_draw = dict(edge_labels_to_draw)

    nx.draw_networkx_edge_labels(tran_sys.g, pos=pos,
                                 edge_labels=edge_labels_to_draw)  # edge_labels


    global ts_run, robot_example
    ts_run = run
    robot_example = tran_sys

    ani = animation.FuncAnimation(fig, update_dot, frames=gen_dot, interval=100)
    ani.save('sin_dot.gif', writer='imagemagick', fps=30)

    plt.show()

def visualize_multi_animation(ts_tuple, run):
    global robot_example, ts_run
    robot_example = ts_tuple[0]

    robot_num = ts_tuple.__len__()

    fig = plt.figure()
    ax = []
    dot = []
    for i in range(0, robot_num):
        ax_t = fig.add_subplot(1, 1, 1)
        dot_t = ax_t.plot([], [], dot_color_ref[i])[0]
        ax.append(ax_t)
        dot.append(dot_t)

    def init():
        for i in range(0, robot_num):
            dot[i].set_data([], [])
        return dot

    def generate_ts_run(ts_run):
        t_max = 40
        t = 0

        team_pose = []
        for robot_index in range(0, ts_run.__len__()):
            run = ts_run[robot_index]
            team_pose.append([])
            t = 0
            for i in range(1, run.__len__()):
                if t >= t_max:
                    break

                weight = ts_tuple[robot_index].g.edge[run[i - 1]][run[i]][0]['weight']
                for j in range(0, weight):
                    # travelling states
                    # team_pose[robot_index].append([pos_ref[run[i]], j])
                    # interpotation
                    x_dist = pos_ref[run[i]][0] - pos_ref[run[i - 1]][0]
                    y_dist = pos_ref[run[i]][1] - pos_ref[run[i - 1]][1]
                    pose_x_t = float(j) / float(weight) * x_dist + pos_ref[run[i - 1]][0]
                    pose_y_t = float(j) / float(weight) * y_dist + pos_ref[run[i - 1]][1]
                    team_pose[robot_index].append([pose_x_t, pose_y_t])

                    t += 1
                    if t >= t_max:
                        break

            if t <= t_max:
                t_max = t  # choose the minium for sync

        return team_pose


    # animation function.  this is called sequentially
    def animate(i):
        for j in range(0, robot_num):
            x = ts_run[j][i][0]
            y = ts_run[j][i][1]
            dot[j].set_data(x, y)
        return dot

    tran_sys = robot_example

    edgelabel = 'control'
    pos = nx.get_node_attributes(tran_sys.g, 'location')
    if len(pos) != tran_sys.g.number_of_nodes():
        pos = nx.spring_layout(tran_sys.g)

    # because the map is the same
    # add map (drawn before)
    pos = pos_ref

    # add color (set before)
    # https://blog.csdn.net/qq_26376175/article/details/67637151
    node_colors = dict([(v, 'yellowgreen') for v in tran_sys.g])
    node_colors['u1'] = node_colors['u2'] = 'tomato'
    node_colors['g1'] = node_colors['g2'] = node_colors['g3'] = node_colors['g4'] = 'cornflowerblue'
    node_colors = list(node_colors.values())

    # edge color
    color_map = 'black'

    nx.draw(tran_sys.g, pos=pos, node_color=node_colors, edge_color=color_map)
    nx.draw_networkx_labels(tran_sys.g, pos=pos)
    edge_labels = nx.get_edge_attributes(tran_sys.g, edgelabel)

    #
    edge_labels_to_draw = []
    for (n1, n2) in edge_labels.items():
        edge_labels_to_draw.append(((n1[0], n1[1]), n2))
    edge_labels_to_draw = dict(edge_labels_to_draw)

    nx.draw_networkx_edge_labels(tran_sys.g, pos=pos,
                                 edge_labels=edge_labels_to_draw)  # edge_labels



    ts_run = generate_ts_run(run)
    anim1 = animation.FuncAnimation(fig, animate, init_func=init, frames=range(0, ts_run[0].__len__()), interval=2500)
    plt.show()

def visualize_animation_w_team_run(ts_tuple, team_run):
    global robot_example, ts_run
    robot_example = ts_tuple[0]

    robot_num = ts_tuple.__len__()

    fig = plt.figure()
    ax = []
    dot = []
    for i in range(0, robot_num):
        ax_t = fig.add_subplot(1, 1, 1)
        dot_t = ax_t.plot([], [], dot_color_ref[i], markersize=22)[0]
        ax.append(ax_t)
        dot.append(dot_t)

    def init():
        for i in range(0, robot_num):
            dot[i].set_data([], [])
        return dot

    def generate_ts_run(ts_run):
        team_pose = []
        for robot_index in range(0, ts_tuple.__len__()):
            team_pose.append([])
            for i in range(0, ts_run.__len__()):
                # travelling state
                if type(ts_run[i][robot_index]) == tuple:
                    # get how many travelling state in this range
                    for k in range(i + 1, ts_run.__len__()):
                        if type(ts_run[k][robot_index]) != tuple:
                            state_len = list(ts_run[k - 1][robot_index])[2] + 1
                            break
                    state_start = list(ts_run[i][robot_index])[0]
                    state_end   = list(ts_run[i][robot_index])[1]
                    x_start = pos_ref[state_start][0]
                    y_start = pos_ref[state_start][1]
                    x_end   = pos_ref[state_end][0]
                    y_end   = pos_ref[state_end][1]

                    x = x_start + (x_end - x_start) * float(list(ts_run[i][robot_index])[2]) / float(state_len)
                    y = y_start + (y_end - y_start) * float(list(ts_run[i][robot_index])[2]) / float(state_len)
                    team_pose[robot_index].append([x, y])
                # normal state
                else:
                    x = pos_ref[ts_run[i][robot_index]][0]
                    y = pos_ref[ts_run[i][robot_index]][1]
                    team_pose[robot_index].append([x, y])

        return team_pose

    # animation function.  this is called sequentially
    def animate(i):
        for j in range(0, robot_num):
            x = ts_run[j][i][0]
            y = ts_run[j][i][1]
            dot[j].set_data(x, y)
        return dot

    tran_sys = robot_example

    edgelabel = 'control'
    pos = nx.get_node_attributes(tran_sys.g, 'location')
    if len(pos) != tran_sys.g.number_of_nodes():
        pos = nx.spring_layout(tran_sys.g)

    # because the map is the same
    # add map (drawn before)
    pos = pos_ref

    # add color (set before)
    # https://blog.csdn.net/qq_26376175/article/details/67637151
    node_colors = dict([(v, 'yellowgreen') for v in tran_sys.g])
    node_colors['u1'] = node_colors['u2'] = 'tomato'
    node_colors['g1'] = node_colors['g2'] = node_colors['g3'] = node_colors['g4'] = 'cornflowerblue'
    node_colors = list(node_colors.values())

    # edge color
    color_map = 'black'

    nx.draw(tran_sys.g, pos=pos, node_color=node_colors, edge_color=color_map)
    nx.draw_networkx_labels(tran_sys.g, pos=pos)
    edge_labels = nx.get_edge_attributes(tran_sys.g, edgelabel)

    #
    edge_labels_to_draw = []
    for (n1, n2) in edge_labels.items():
        edge_labels_to_draw.append(((n1[0], n1[1]), n2))
    edge_labels_to_draw = dict(edge_labels_to_draw)

    nx.draw_networkx_edge_labels(tran_sys.g, pos=pos,
                                 edge_labels=edge_labels_to_draw)  # edge_labels



    ts_run = generate_ts_run(team_run)
    anim1 = animation.FuncAnimation(fig, animate, init_func=init, frames=range(0, ts_run[0].__len__()), interval=500)
    plt.show()

