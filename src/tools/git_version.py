#!/usr/bin/env python3
#
# Software License Agreement (MIT License)
#
# Author: Duke Fong <d@d-l.io>

import subprocess
import re
import sys


def invoke_cmd_r(cmd):
    #print('$', cmd)
    p = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    #print(f'ret: {p.returncode}')
    #print(p.stdout.decode() + p.stderr.decode())
    return (p.returncode, p.stdout.decode())

def invoke_cmd(cmd):
    #print('$', cmd)
    p = subprocess.run(cmd, shell=True)
    #print(f'ret: {p.returncode}')
    return p.returncode


def compute_version():
    _ret, version_full = invoke_cmd_r('git describe --dirty --always --tags')
    _ret, version_fallback = invoke_cmd_r('git describe --always --tags')
    version_mid = re.sub(r".*_v", "", version_full.strip())
    version_mid = re.sub(r"-g.*", "", version_mid)

    # match X.Y-N
    m = re.match(r"^(\d+)\.(\d+)-(\d+)$", version_mid)
    if m:
        major, minor, patch = map(int, m.groups())
        minor += patch
        return f"v{major}.{minor}"

    # match X.Y
    m = re.match(r"^\d+\.\d+$", version_mid)
    if m:
        return f"v{version_mid}"

    return version_fallback.strip()


if __name__ == "__main__":
    print(compute_version())
