#!/usr/bin/env python3
"""회차 폴더들의 checksums.sha256 검증 (읽은 파일의 페이지 캐시를 바로 비워 메모리를 쌓지 않음).
사용: verify_checksums.py <폴더>...   각 폴더 안의 checksums.sha256 기준"""
import sys, os, hashlib
from pathlib import Path
bad_total = 0
for d in map(Path, sys.argv[1:]):
    lines = [l.split(None, 1) for l in (d / 'checksums.sha256').read_text().splitlines() if l.strip()]
    bad = []
    for h, name in lines:
        f = d / name.strip().lstrip('*'); 
        if not f.exists(): bad.append(f'{name.strip()} (없음)'); continue
        s = hashlib.sha256(); fd = os.open(f, os.O_RDONLY); off = 0
        while True:
            b = os.read(fd, 16 << 20)
            if not b: break
            s.update(b); off += len(b)
            if off % (512 << 20) < (16 << 20): os.posix_fadvise(fd, 0, off, os.POSIX_FADV_DONTNEED)
        os.posix_fadvise(fd, 0, 0, os.POSIX_FADV_DONTNEED); os.close(fd)
        if s.hexdigest() != h: bad.append(name.strip())
    bad_total += len(bad)
    print(f'{d.name}: 파일 {len(lines)}개, 불일치 {len(bad)}' + (f' → {bad}' if bad else ''), flush=True)
print('전체 판정:', '모두 일치' if bad_total == 0 else f'불일치 {bad_total}건')
