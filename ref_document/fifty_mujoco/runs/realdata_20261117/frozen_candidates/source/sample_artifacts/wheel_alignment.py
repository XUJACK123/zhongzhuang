"""Spatial wheel-center alignment; independent of wheel spin or world heading."""
import math


def foreaft_offset(q,thigh,shin,ops):
    right=thigh*ops.cos(q[...,0])+shin*ops.cos(q[...,0]+q[...,1]+math.pi/2)
    left=thigh*ops.cos(-q[...,3])+shin*ops.cos(-q[...,3]-q[...,4]+math.pi/2)
    return right-left


def eligible(commands,unchanged_s,cfg,ops):
    # Gate on the requested mode/time, not tracking error that policy can game.
    return ((commands[...,0]>=-.001)&(ops.abs(commands[...,1])<=.02)
            &(unchanged_s>=cfg.get('settle_s',.5)))


def penalty(offset,mask,cfg,ops):
    return ops.where(mask,(offset/cfg.get('scale_m',.02))**2,offset*0)
