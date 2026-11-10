"""Keep a heavy offline job from starving the PC (2026-11-10: a carving run grew to 103 GB and crashed).

  import memcap; memcap.guard(cap_gb=6, need_free_gb=16)

Windows: puts this process in a job object with a hard per-process memory limit, so an allocation past the cap fails
in this process only; refuses to start when the machine has less than need_free_gb available. Elsewhere: an
RLIMIT_AS cap. Plain python (ctypes, psutil).
"""
import ctypes
import os
import sys

import psutil


def guard(cap_gb=6.0, need_free_gb=16.0):
    free = psutil.virtual_memory().available / 2 ** 30
    if free < need_free_gb:
        sys.exit(f'memcap: only {free:.1f} GB free (need {need_free_gb:.0f}); not starting')
    cap = int(cap_gb * 2 ** 30)
    if os.name != 'nt':
        import resource
        resource.setrlimit(resource.RLIMIT_AS, (cap, cap))
        return
    k32 = ctypes.WinDLL('kernel32', use_last_error=True)

    class IO(ctypes.Structure):
        _fields_ = [(n, ctypes.c_ulonglong) for n in ('r', 'w', 'o', 'rb', 'wb', 'ob')]

    class BASIC(ctypes.Structure):
        _fields_ = [('PerProcessUserTimeLimit', ctypes.c_longlong), ('PerJobUserTimeLimit', ctypes.c_longlong),
                    ('LimitFlags', ctypes.c_uint32), ('MinimumWorkingSetSize', ctypes.c_size_t),
                    ('MaximumWorkingSetSize', ctypes.c_size_t), ('ActiveProcessLimit', ctypes.c_uint32),
                    ('Affinity', ctypes.c_size_t), ('PriorityClass', ctypes.c_uint32), ('SchedulingClass', ctypes.c_uint32)]

    class EXT(ctypes.Structure):
        _fields_ = [('Basic', BASIC), ('Io', IO), ('ProcessMemoryLimit', ctypes.c_size_t),
                    ('JobMemoryLimit', ctypes.c_size_t), ('PeakProcessMemoryUsed', ctypes.c_size_t),
                    ('PeakJobMemoryUsed', ctypes.c_size_t)]

    k32.CreateJobObjectW.restype = ctypes.c_void_p
    k32.GetCurrentProcess.restype = ctypes.c_void_p
    job = k32.CreateJobObjectW(None, None)
    info = EXT()
    info.Basic.LimitFlags = 0x100 | 0x200            # PROCESS_MEMORY | JOB_MEMORY
    info.ProcessMemoryLimit = cap
    info.JobMemoryLimit = cap
    ok = k32.SetInformationJobObject(ctypes.c_void_p(job), 9, ctypes.byref(info), ctypes.sizeof(info))
    ok = ok and k32.AssignProcessToJobObject(ctypes.c_void_p(job), ctypes.c_void_p(k32.GetCurrentProcess()))
    if not ok:
        sys.exit(f'memcap: could not set the {cap_gb} GB cap (error {ctypes.get_last_error()}); not starting')
    guard.job = job                                   # keep the handle alive
    print(f'memcap: {cap_gb:.0f} GB cap, {free:.0f} GB free', flush=True)
