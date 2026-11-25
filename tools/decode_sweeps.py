#!/usr/bin/env python3
"""Decode RF247 v1 sweep files to long-format CSV. No third-party dependencies."""
import argparse
import binascii
import csv
import struct
import sys
from pathlib import Path

HEADER = struct.Struct('<8sHHIIHHIIIHHBBBBQIH B5s'.replace(' ', ''))
PREFIX = struct.Struct('<HIQIHHBB')

def decode(path, out):
    bad = good = 0
    with path.open('rb') as f:
        h = f.read(HEADER.size)
        if len(h) != HEADER.size:
            raise ValueError('truncated header')
        magic, version, hs, sid, uid, part, points, start, step, rbw, dwell, settle, samples, smoothing, quiet, _, t0, epoch0, record_size, reset, _ = HEADER.unpack(h)
        if magic != b'RF247BIN' or version != 1 or hs != 64 or not 8 <= points <= 255 or record_size != PREFIX.size + points + 2:
            raise ValueError('unsupported or corrupt header')
        writer = csv.writer(out)
        writer.writerow(['session_id','session_uid','part','sweep_id','t_ms','epoch_s','sweep_ms','flags','cfg_id','frequency_hz','rssi_dbm','valid'])
        while True:
            r = f.read(record_size)
            if not r: break
            if len(r) != record_size:
                bad += 1; print('Truncated final record', file=sys.stderr); break
            m, sweep, t, epoch, duration, flags, cfg, n = PREFIX.unpack_from(r)
            crc, = struct.unpack_from('<H',r,len(r)-2)
            if m != 0xA55A or n != points or binascii.crc_hqx(r[:-2],0xffff) != crc:
                bad += 1; print(f'Bad record near byte {f.tell()-record_size}', file=sys.stderr); continue
            good += 1
            for i, raw in enumerate(r[PREFIX.size:-2]):
                writer.writerow([sid,uid,part,sweep,t,epoch or '',duration,flags,cfg,start+i*step,'' if raw==255 else -raw/2,int(raw!=255)])
    print(f'{path.name}: {good} valid records, {bad} corrupt/truncated records; RBW {rbw} Hz', file=sys.stderr)
    return bad

def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('input',type=Path); p.add_argument('output',type=Path); a=p.parse_args()
    if a.input.resolve()==a.output.resolve(): p.error('input and output must differ')
    try:
        with a.output.open('w',newline='') as out: bad=decode(a.input,out)
    except (OSError,ValueError) as e: p.exit(2,f'{e}\n')
    return 1 if bad else 0
if __name__=='__main__': sys.exit(main())
