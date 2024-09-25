import matplotlib.pyplot as plt
import numpy as np

def createfigure(X1, Y1, d, l):
    """
    CREATEFIGURE(X1, Y1, d, l)
    X1: vector of x data
    Y1: vector of y data
    d: maximum diameter of the hull
    l: total length of the hull
    """
    
    # Creating the figure
    fig = plt.figure(figsize=(400*1.2*l/100*2, 400*5*d/2/100*2), facecolor='w')
    
    # Create axes
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim([-0.1*l, 1.1*l])
    ax.set_ylim([-d/2*2.5, d/2*2.5])
    
    # Drawing the lines
    ax.plot(X1, Y1, linewidth=1.6, color='b')
    ax.plot(X1, -Y1, linewidth=1.6, color='b')
    
    # Show grid and axis lines
    ax.grid(True)
    ax.axhline(0, color='black',linewidth=1)
    ax.axvline(0, color='black',linewidth=1)
    
    # Configure axis labels
    ax.set_xlabel('Eje X')
    ax.set_ylabel('Eje Y')

    plt.show()


def createfigure2(X1, Y1, d, l):
    """
    CREATEFIGURE(X1, Y1, d, l)
    X1: vector of x data
    Y1: vector of y data
    d: maximum diameter of the hull
    l: total length of the hull
    """
    
    # Creating the figure
    fig = plt.figure(figsize=(400*1.2*l/100*2, 400*5*d/2/100*2), facecolor='w')
    
    # Create axes
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim([-0.6*l, 0.6*l])
    ax.set_ylim([-d/2*2.5, d/2*2.5])
    
    # Drawing the lines
    ax.plot(X1, Y1, linewidth=1.6, color='b')
    ax.plot(X1, -Y1, linewidth=1.6, color='b')
    
    # Show grid and axis lines
    ax.grid(True)
    ax.axhline(0, color='black',linewidth=1)
    ax.axvline(0, color='black',linewidth=1)
    
    # Configure axis labels
    ax.set_xlabel('Eje X')
    ax.set_ylabel('Eje Y')

    plt.show()