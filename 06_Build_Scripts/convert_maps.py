#!/usr/bin/env python3
"""Convert saved .pgm maps to .png so they open in Windows."""
import glob
import os
from PIL import Image

DEST = '/mnt/d/ROBOTICS PROJECT 7TH SEM/03_Maps'
os.makedirs(DEST, exist_ok=True)

for f in sorted(glob.glob('/home/ommdash/maps/*.pgm')):
    name = os.path.basename(f)[:-4] + '.png'
    img = Image.open(f)
    img.save(os.path.join(DEST, name))
    print('converted', name, img.size)

print('PNG_OK')
