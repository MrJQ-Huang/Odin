"""Resolve stable USB serial to the current address at every launch."""
import pathlib
import sys
found=[]
for p in pathlib.Path('/sys/bus/usb/devices').iterdir():
    try:
        if (p/'idVendor').read_text().strip()!='2207' or (p/'idProduct').read_text().strip()!='0019': continue
        if (p/'serial').read_text().strip()!=sys.argv[1]: continue
        if float((p/'speed').read_text())<5000: raise RuntimeError('Odin must use USB3')
        found.append((int((p/'busnum').read_text()),int((p/'devnum').read_text())))
    except FileNotFoundError: continue
if len(found)!=1: raise RuntimeError('Expected exactly one Odin with USB serial '+sys.argv[1])
print(*found[0])
