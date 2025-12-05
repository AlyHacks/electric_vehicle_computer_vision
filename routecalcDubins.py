#!/usr/bin/env python3
"""
Pure-Python full Dubins path implementation + sampling every dt seconds.

Usage: run file and provide interactive inputs (coordinates in cm, R in cm, vehicle length in cm).
Outputs per-sample lines: t(s), x(cm), y(cm), heading(deg), velocity(cm/s).
"""

import math
from math import sin, cos, atan2, sqrt, acos, atan

# --------------------------
# Helper functions
# --------------------------
def mod2pi(theta):
    """Normalize angle to [0, 2*pi)."""
    return theta - 2.0*math.pi * math.floor(theta / (2.0*math.pi))

def angdiff(a, b):
    """Smallest difference a-b in (-pi,pi]."""
    d = a - b
    while d <= -math.pi: d += 2*math.pi
    while d > math.pi: d -= 2*math.pi
    return d

def dist(p, q):
    return math.hypot(q[0]-p[0], q[1]-p[1])

def rot(x, y, th):
    """Rotate (x,y) by th (rad)"""
    return (x*math.cos(th) - y*math.sin(th), x*math.sin(th) + y*math.cos(th))

# --------------------------
# Core: Dubins path math (from standard derivations)
# Reference: classic Dubins formulas (LSL, RSR, LSR, RSL, RLR, LRL)
# --------------------------
def dubins_words():
    return ['LSL','RSR','LSR','RSL','RLR','LRL']

def _compute_parameters(alpha, beta, d):
    """
    Given normalized coordinates, compute candidate params for all six types.
    Returns dict of (type -> (ok, [t,p,q])) where lengths scaled in units of R.
    """
    params = {}

    # LSL
    tmp0 = d + sin(alpha) - sin(beta)
    p_sq = 2 + (d*d) - (2*cos(alpha - beta)) + 2*d*(sin(alpha) - sin(beta))
    if p_sq >= 0:
        p = math.sqrt(p_sq)
        tmp1 = atan2((cos(beta) - cos(alpha)), tmp0)
        t = mod2pi(-alpha + tmp1)
        q = mod2pi(beta - tmp1)
        params['LSL'] = (True, (t, p, q))
    else:
        params['LSL'] = (False, None)

    # RSR
    tmp0 = d - sin(alpha) + sin(beta)
    p_sq = 2 + (d*d) - (2*cos(alpha - beta)) + 2*d*(sin(beta) - sin(alpha))
    if p_sq >= 0:
        p = math.sqrt(p_sq)
        tmp1 = atan2((cos(alpha) - cos(beta)), tmp0)
        t = mod2pi(alpha - tmp1)
        q = mod2pi(-beta + tmp1)
        params['RSR'] = (True, (t, p, q))
    else:
        params['RSR'] = (False, None)

    # LSR
    p_sq = -2 + (d*d) + 2*cos(alpha - beta) + 2*d*(sin(alpha) + sin(beta))
    if p_sq >= 0:
        p = math.sqrt(p_sq)
        tmp2 = atan2((-cos(alpha) - cos(beta)), (d + sin(alpha) + sin(beta)))
        t = mod2pi(-alpha + tmp2 - math.atan2(-2.0, p))
        q = mod2pi(-mod2pi(beta) + tmp2 - math.atan2(-2.0, p))
        params['LSR'] = (True, (t, p, q))
    else:
        params['LSR'] = (False, None)

    # RSL
    p_sq = -2 + (d*d) + 2*cos(alpha - beta) - 2*d*(sin(alpha) + sin(beta))
    if p_sq >= 0:
        p = math.sqrt(p_sq)
        tmp2 = atan2((cos(alpha) + cos(beta)), (d - sin(alpha) - sin(beta)))
        t = mod2pi(alpha - tmp2 + math.atan2(2.0, p))
        q = mod2pi(beta - tmp2 + math.atan2(2.0, p))
        params['RSL'] = (True, (t, p, q))
    else:
        params['RSL'] = (False, None)

    # RLR
    tmp0 = (6. - d*d + 2*cos(alpha - beta) + 2*d*(sin(alpha) - sin(beta))) / 8.0
    if abs(tmp0) <= 1.0:
        p = mod2pi(2*math.pi - math.acos(tmp0))
        t = mod2pi(alpha - atan2((cos(alpha) - cos(beta)), (d - sin(alpha) + sin(beta))) + p/2.0)
        q = mod2pi(alpha - beta - t + p)
        params['RLR'] = (True, (t, p, q))
    else:
        params['RLR'] = (False, None)

    # LRL
    tmp0 = (6. - d*d + 2*cos(alpha - beta) + 2*d*(sin(beta) - sin(alpha))) / 8.0
    if abs(tmp0) <= 1.0:
        p = mod2pi(2*math.pi - math.acos(tmp0))
        t = mod2pi(-alpha - atan2((cos(alpha) - cos(beta)), (d + sin(alpha) - sin(beta))) + p/2.0)
        q = mod2pi(mod2pi(beta) - alpha - t + p)
        params['LRL'] = (True, (t, p, q))
    else:
        params['LRL'] = (False, None)

    return params

# --------------------------
# Build a Dubins path between two poses
# Each pose is (x, y, heading) where heading in radians.
# Turning radius R given.
# Returns: best_path = dict with keys:
#   { 'type': 'LSL'.., 'segments': [(seg_type, seg_length_in_cm), ...], 'total_length' }
# All segment lengths in cm.
# --------------------------
def dubins_shortest_path(start_pose, end_pose, R):
    (x0, y0, th0) = start_pose
    (x1, y1, th1) = end_pose

    # transform goal into start frame
    dx = x1 - x0
    dy = y1 - y0
    # rotate by -th0, and scale by 1/R
    phi = atan2(dy, dx)
    D = sqrt(dx*dx + dy*dy)
    d = D / R
    if D == 0.0:
        # special case: same position
        xf = 0.0
        yf = 0.0
    else:
        xf = (dx*math.cos(-th0) - dy*math.sin(-th0)) / R
        yf = (dx*math.sin(-th0) + dy*math.cos(-th0)) / R

    # compute alpha, beta
    # standard normalization:
    # theta = atan2(yf, xf)
    theta = math.atan2(yf, xf) if (xf != 0.0 or yf != 0.0) else 0.0
    alpha = mod2pi(-theta)  # = mod2pi(th0 - theta) with th0 normalized to 0
    beta = mod2pi(th1 - th0 - theta)

    # Use standard formulas that expect alpha and beta defined like above.
    # However many references use alpha = mod2pi(theta - th0) etc.
    # The formula set used here is consistent with common implementations after testing.

    # compute parameters
    params = _compute_parameters(alpha, beta, d)

    best = None
    best_len = float('inf')
    # convert parameter triple (t,p,q) into actual lengths along path (scaled by R)
    for typ, (ok, triple) in params.items():
        if not ok:
            continue
        t, p, q = triple
        # path length in units of R
        length = (t + p + q) * R
        if length < best_len:
            best_len = length
            best = (typ, t*R, p*R, q*R)  # store lengths in cm

    if best is None:
        raise RuntimeError("No feasible Dubins path found for given configurations.")

    typ, lt, lp, lq = best

    # Now expand segments: each triple corresponds to [L/R or S, etc]
    segs = []
    if typ == 'LSL':
        segs = [('L', lt), ('S', lp), ('L', lq)]
    elif typ == 'RSR':
        segs = [('R', lt), ('S', lp), ('R', lq)]
    elif typ == 'LSR':
        segs = [('L', lt), ('S', lp), ('R', lq)]
    elif typ == 'RSL':
        segs = [('R', lt), ('S', lp), ('L', lq)]
    elif typ == 'RLR':
        segs = [('R', lt), ('L', lp), ('R', lq)]
    elif typ == 'LRL':
        segs = [('L', lt), ('R', lp), ('L', lq)]
    else:
        raise RuntimeError("Unknown path type selected.")

    return {
        'type': typ,
        'segments': segs,
        'total_length': sum([s[1] for s in segs])
    }

# --------------------------
# Sampling along a Dubins path
# Given start_pose (x,y,th) and path produced by dubins_shortest_path and R,
# produce (x,y,heading) for a param s along path length (0..total_length).
# We implement a function to sample multiple s values sequentially.
# --------------------------
def sample_dubins(start_pose, path, R, s):
    """
    start_pose: (x,y,th)
    path: { 'segments': [('L'/'R'/'S', length_cm), ... ] }
    s: distance along path in cm (0..total_length)
    returns pose (x,y,th)
    """
    x, y, th = start_pose
    remaining = s
    for seg_type, seg_len in path['segments']:
        if remaining <= seg_len + 1e-9:
            # sample inside this segment
            if seg_type == 'S':
                # go straight by remaining
                nx = x + remaining * cos(th)
                ny = y + remaining * sin(th)
                nth = th
                return (nx, ny, nth)
            else:
                # arc: radius R, center depends on left/right
                dir_sign = 1.0 if seg_type == 'L' else -1.0
                # center location
                cx = x + dir_sign * R * (-sin(th))
                cy = y + dir_sign * R * (cos(th))
                # angle traveled along arc
                ang = remaining / R
                # new heading
                if seg_type == 'L':
                    nth = th + ang
                else:
                    nth = th - ang
                # new position: rotate vector from center
                # vector from center to current position is (-dir_sign*R*sin(th), dir_sign*R*cos(th))
                # after rotation by +/- ang, vector becomes ...
                vx = -dir_sign * R * sin(th)
                vy = dir_sign * R * cos(th)
                # rotate vx,vy by +angle (if L) or -angle (if R)
                cosa = cos(ang)
                sina = sin(ang)
                # rotation matrix for positive angle
                rx = vx * cosa - vy * sina
                ry = vx * sina + vy * cosa
                nx = cx + rx
                ny = cy + ry
                return (nx, ny, nth)
        else:
            # advance through entire segment
            if seg_type == 'S':
                x = x + seg_len * cos(th)
                y = y + seg_len * sin(th)
                # th unchanged
            else:
                dir_sign = 1.0 if seg_type == 'L' else -1.0
                ang = seg_len / R
                if seg_type == 'L':
                    nth = th + ang
                else:
                    nth = th - ang
                # center
                cx = x + dir_sign * R * (-sin(th))
                cy = y + dir_sign * R * (cos(th))
                # compute new pos after full arc
                vx = -dir_sign * R * sin(th)
                vy = dir_sign * R * cos(th)
                cosa = cos(ang)
                sina = sin(ang)
                rx = vx * cosa - vy * sina
                ry = vx * sina + vy * cosa
                x = cx + rx
                y = cy + ry
                th = nth
            remaining -= seg_len
    # if s exceeds total, return final pose
    return (x, y, th)

# --------------------------
# Plan full A->W and W->B Dubins and concatenate
# --------------------------
def plan_A_W_B_dubins(A, B, D, w, vehicle_length, R, T_required):
    # derive W
    W = (D[0], D[1] + w/2.0)

    # initial heading: A->B
    heading_AB = math.atan2(B[1]-A[1], B[0]-A[0])
    # W heading: W->B
    heading_WB = math.atan2(B[1]-W[1], B[0]-W[0])

    start_pose_A = (A[0], A[1], heading_AB)
    goal_pose_W = (W[0], W[1], heading_WB)
    start_pose_W = goal_pose_W
    goal_pose_B = (B[0], B[1], heading_WB)

    # compute shortest Dubins for each subsegment
    path1 = dubins_shortest_path(start_pose_A, goal_pose_W, R)
    path2 = dubins_shortest_path(start_pose_W, goal_pose_B, R)

    # total length
    L1 = path1['total_length']
    L2 = path2['total_length']
    total_length = L1 + L2

    # uniform speed to meet T_required
    if T_required <= 0:
        raise ValueError("T_required must be positive.")
    v = total_length / T_required

    # Build concatenated path structure
    # For sampling we need start pose and then path segments list
    # We'll sample first path starting at start_pose_A, then second starting at end pose of path1.
    return {
        'W': W,
        'start_pose_A': start_pose_A,
        'goal_pose_W': goal_pose_W,
        'path1': path1,
        'path2': path2,
        'total_length': total_length,
        'v': v,
        'R': R
    }

# --------------------------
# Concatenate sampling across two Dubins paths
# --------------------------
def sample_full_trajectory(plan, dt=0.05):
    """
    plan produced by plan_A_W_B_dubins.
    returns list of (t, x, y, heading_deg, v)
    """
    R = plan['R']
    v = plan['v']
    # path1 sampled relative to start_pose_A, length L1
    L1 = plan['path1']['total_length']
    L2 = plan['path2']['total_length']
    total = plan['total_length']

    # create time steps by arc distance
    # step in arclength per dt: s_step = v * dt
    s_step = v * dt
    if s_step <= 0:
        raise ValueError("v*dt must be positive.")
    samples = []
    s = 0.0
    t = 0.0
    while s < total - 1e-9:
        if s <= L1 + 1e-9:
            # sample on path1
            pose = sample_dubins(plan['start_pose_A'], plan['path1'], R, s)
        else:
            # sample on path2: need start pose for path2 = end pose of path1
            # get local s2
            s2 = s - L1
            # compute end pose of path1 by sampling at its end
            end1 = sample_dubins(plan['start_pose_A'], plan['path1'], R, L1)
            pose = sample_dubins(end1, plan['path2'], R, s2)
        x, y, th = pose
        samples.append((t, x, y, math.degrees(mod2pi(th)), v))
        s += s_step
        t += dt

    # ensure final point at B is appended exactly at T_required
    # compute final pose (end of path2)
    end1 = sample_dubins(plan['start_pose_A'], plan['path1'], R, L1)
    end2 = sample_dubins(end1, plan['path2'], R, L2)
    # clamp final t to T_required
    T_required = total / v
    samples.append((T_required, end2[0], end2[1], math.degrees(mod2pi(end2[2])), v))
    return samples

# --------------------------
# Utility to print segment breakdown
# --------------------------
def print_segments_breakdown(plan, vehicle_length):
    print("=== Segment breakdown ===")
    R = plan['R']
    def seg_info(path, name):
        print(f"--- {name} ---")
        for i, (stype, slen) in enumerate(path['segments'], start=1):
            steering_deg = 0.0
            if stype in ('L','R'):
                sign = 1.0 if stype == 'L' else -1.0
                steering_deg = math.degrees(math.atan(vehicle_length / R)) * sign
            print(f"seg {i}: type={stype}, length={slen:.3f} cm, time={slen/plan['v']:.3f} s, steering={steering_deg:.3f} deg")
        print(f"subtotal length = {path['total_length']:.3f} cm\n")

    seg_info(plan['path1'], "A -> W (Dubins)")
    seg_info(plan['path2'], "W -> B (Dubins)")
    print(f"Total path length = {plan['total_length']:.3f} cm")
    print(f"Uniform speed v = {plan['v']:.6f} cm/s")
    print(f"Required arrival time T = {plan['total_length']/plan['v']:.6f} s\n")

# --------------------------
# Main interactive block
# --------------------------
if __name__ == "__main__":
    print("Dubins (pure-Python) A -> W -> B trajectory sampler")
    print("Units: coordinates cm, R cm, vehicle length cm, time s")

    #Point Coordinates
    Ax = 0
    Ay = 0
    Bx = 1000
    By = 0
    Cx = 500
    Cy = 100
    Dx = 500
    Dy = 70

    #Car dimension
    w = 20
    L = 30
    R = 10
    T_required = 50
    dt = 0.05

    A = (Ax, Ay)
    B = (Bx, By)
    D = (Dx, Dy)

    plan = plan_A_W_B_dubins(A, B, D, w, L, R, T_required)

    # Print plan summary & per-segment durations/steering
    print_segments_breakdown(plan, L)

    # Generate samples
    samples = sample_full_trajectory(plan, dt=dt)

    # Print header
    print("t(s)\tx(cm)\ty(cm)\theading(deg)\tvelocity(cm/s)")
    for rec in samples:
        print(f"{rec[0]:.2f}\t{rec[1]:.3f}\t{rec[2]:.3f}\t{rec[3]:.3f}\t{rec[4]:.6f}")

    print("\nDone.")
