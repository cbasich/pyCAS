#!/usr/bin/env python3
from PIL import Image

img = Image.new('RGB', (7, 7), color = 'black')

pixels = img.load()

pixels[1, 1] = (255, 255, 255)
pixels[1, 2] = (255, 255, 255)
pixels[1, 3] = (255, 255, 255)
pixels[1, 4] = (255, 255, 255)
pixels[1, 5] = (255, 255, 255)
pixels[2, 3] = (255, 255, 255)
pixels[2, 5] = (255, 255, 255)
pixels[3, 1] = (255, 255, 255)
pixels[3, 2] = (255, 255, 255)
pixels[3, 3] = (255, 255, 255)
pixels[3, 5] = (255, 255, 255)
pixels[4, 1] = (255, 255, 255)
pixels[4, 3] = (255, 255, 255)
pixels[4, 5] = (255, 255, 255)
pixels[5, 1] = (255, 255, 255)
pixels[5, 2] = (255, 255, 255)
pixels[5, 3] = (255, 255, 255)
pixels[5, 4] = (255, 255, 255)
pixels[5, 5] = (255, 255, 255)

img.save('/home/willcoe/pyCAS/src/pyCAS/domains/CDB/maps/test_campus.png')
 
