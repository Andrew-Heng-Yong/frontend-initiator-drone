"""Fail when the native visual PnP port diverges from the matching-version reference."""
import sys
import numpy as np
p=np.loadtxt(sys.argv[1],delimiter=',',ndmin=2)
n=np.loadtxt(sys.argv[2],delimiter=',',ndmin=2)
assert p.shape==n.shape and p.shape[1]==17
assert np.array_equal(p[:,0],n[:,0]), 'Tracking decisions differ'
error=float(np.max(np.abs(p[:,1:]-n[:,1:])))
assert error<1e-4, f'Transform difference {error}'
print(f'{len(p)} frames: all tracking decisions match; max transform difference {error:.3g}')
