"""Miscellaneous helper functions."""
import matplotlib.pyplot as plt


def BoxOff(*argin):
    """Remove ugly box around plots."""
    if len(argin) > 0:
        ax = argin[0]
    else:
        ax = plt.gca()
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.yaxis.set_ticks_position('left')
    ax.xaxis.set_ticks_position('bottom')
