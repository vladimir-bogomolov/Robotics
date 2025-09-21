import numpy as np

# Transform world coordinats to display (map) coordinats
def world2map(x, y):
    px = int((x + 2.25) * 40)
    py = int((y - 2) * (-50))
    px = max(0, min(199, px))
    py = max(0, min(299, py))
    return [px, py]

# Transform display (map) coordinats to world coordinats   
def map2world(px, py):
    px = max(0, min(199, px))
    py = max(0, min(299, py))
    # Reverse the transformations
    x = px / 40 - 2.25
    y = py / (-50) + 2
    return [x, y]
