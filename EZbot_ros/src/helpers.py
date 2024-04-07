#! /usr/bin/env python3

import numpy as np

# Define the input value
x = -15

# Clamp the input value to the input range using min() and max()
def clamp(x, input_range):
    return max(min(x, input_range[1]), input_range[0])


def map_to_byte(val_):
    # Define the input range (-12 to 12) and the output range (-255 to 255)
    input_range = [-12, 12]
    output_range = [-255, 255]

    val = clamp(val_, input_range)

    # Map the input value from the input range to the output range using np.interp()
    return np.interp(val, input_range, output_range)
