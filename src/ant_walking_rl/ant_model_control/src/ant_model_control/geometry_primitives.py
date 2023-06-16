import math

def angle_diff(a1, a2):
    """ difference between 2 angles """
    diff = a1 - a2
    return (diff + math.pi) % (2*math.pi) - math.pi
