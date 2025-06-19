import numpy as np
import time

from Traj.PolyTrajPhase1   import PolyTrajPhase1 as PolyTrajPhase1Poly
from Traj.modf_PolyTrajPhase_1 import PolyTrajPhase1 as PolyTrajPhase1Table

def compare_trajectories(num_samples=20, rtol=1e-6, atol=1e-8):
    # 实例化两种版本
    poly   = PolyTrajPhase1Poly()
    table  = PolyTrajPhase1Table()

    t0, tf = poly.t_ref[0], poly.t_final
    ts = np.linspace(t0, tf, num=num_samples)

    print("Comparing outputs at sample times:")
    for t in ts:
        out_poly  = poly.update(t)
        out_table = table.update(t)

        print(f"\n t = {t:.3f}s")
        for key in ('x', 'x_dot', 'x_ddot'):
            a = out_poly[key]
            b = out_table[key]
            ok = np.allclose(a, b, rtol=rtol, atol=atol)
            status = "✅" if ok else "❌"
            print(f"  {status} {key:7s} | old: {a.tolist()} | new: {b.tolist()}")

    # 性能对比
    start = time.time()
    for t in ts:
        poly.update(t)
    t_poly = time.time() - start

    start = time.time()
    for t in ts:
        table.update(t)
    t_table = time.time() - start

    print(f"\nPerformance: ")
    print(f"  Polynomial version: {t_poly:.3f} s for {num_samples} calls")
    print(f"  Table lookup version: {t_table:.3f} s for {num_samples} calls")

if __name__ == "__main__":
    compare_trajectories()
