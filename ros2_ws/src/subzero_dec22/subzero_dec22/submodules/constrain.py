def constrain(val,lowerlimit,upperlimit):
    if val < lowerlimit:
        val = lowerlimit
    elif val > upperlimit:
        val = upperlimit
    return val