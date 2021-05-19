function value = clamp(value, minval, maxval)
value = min(maxval, max(minval, value));