import sys
import os

import matplotlib.pyplot as plt
import numpy as np
import ntpath

def readline(f):
    return f.readline()

def read_int(f):
    return int(readline(f))

def read_integers(f):
    return list(map(int,readline(f).split()))

def read_floats(f):
    return list(map(float,readline(f).split()))

def extract_route_data(f):
    num_stops = int(f.readline().split(" ")[2])
    targets = [read_floats(f) for _ in range(num_stops)]
    f.readline() # skip a line

    routes = list()
    current_route = list()

    line = f.readline()
    while line.strip() != 'END':
        while line.strip() != "- - -":
            pos = list(map(float, line.split()[1:]))
            current_route.append(pos)
            line = f.readline()

        routes.append(current_route[1:])
        current_route = list()
        line = f.readline()

    return targets, routes


def create_routing_plot(f, pathtofile):
    targets, routes = extract_route_data(f)

    colors = [(np.random.rand(3), np.random.rand(3)) for _ in range(len(targets))]
    for t, c in zip(routes, colors):
        for i, pt in enumerate(t):
            plt.plot(pt[0], pt[1], '.', color=c[0])
            plt.arrow(pt[0], pt[1], np.cos(pt[2]) / 4, np.sin(pt[2]) / 4, color=c[1],  head_width=.01, head_length=.005, fc='k', ec='k')
            if i % 10 == 0:
                plt.text(pt[0], pt[1], str(i), fontsize=5)


    for pt, p in enumerate(targets):
        plt.plot(p[0], p[1], 'x', color='red')
        plt.text(p[0], p[1], str(pt+1), fontsize=12)

    TRANSPARENT_BACKGROUND = True
    transparent_suffix = "_tsp" if TRANSPARENT_BACKGROUND else ""

    plt.title("Route visualization of " + pathtofile + "\n.")
    fname = os.path.splitext(pathtofile)[0]
    plotname = "plot_of_"+ntpath.basename(fname)+transparent_suffix+".png"

    folderpath = makePath(os.path.join(fname.split("/")[0], "plots"))
    if not os.path.exists(folderpath):
        os.makedirs(folderpath)

    plt.savefig(os.path.join(folderpath, plotname), dpi=300, transparent=TRANSPARENT_BACKGROUND)
    plt.close()
    print("Plot '{}' created!".format(plotname))

def makePath(path):
    return os.path.join(logging_root, path)

if __name__ == '__main__':
    if len(sys.argv) != 2 or sys.argv[0] in ['help', '-h']:
        print("Usage: python3 route_visualizer.py <path to log file>")
        print("\t\tthe path to the log file as starting from the ROUTE_LOGGING_HOME directory")

    pathtofile = sys.argv[1]

    if not pathtofile.endswith(".log"):
        pathtofile += ".log"

    logging_root = os.environ.get('ROUTE_LOGGING_HOME', None)

    if logging_root is None:
        raise RuntimeError("The environment variable ROUTE_LOGGING_HOME is not set!")

    filepath = os.path.join(logging_root, pathtofile)

    with open(filepath, 'r') as f:
        create_routing_plot(f, pathtofile)