from random import *
import math
import argparse
import json

from PIL import Image, ImageDraw, ImageOps

# from filters import *
# from strokesort import *

no_cv = False

export_path = "images/out.svg"
svg_folder = "images/"
json_folder = "images/"

# vectorisation defaults
draw_contours = True
draw_hatch = True
resolution = 1024
hatch_size = 16
contour_simplify = 1

try:
    import numpy as np
    import cv2
except:
    print("Cannot import numpy/openCV. Switching to NO_CV mode.")
    no_cv = True


def image_to_json(
    image_filename,
    resolution=1024,
    draw_hatch=True,
    hatch_size = 16,
    draw_contours=True,
    contour_simplify=1,
    ):

    lines=vectorise(
        image_filename,
        resolution=resolution,
        draw_hatch=draw_hatch,
        hatch_size=hatch_size,
        draw_contours=draw_contours,
        contour_simplify=contour_simplify,
        )
    filename = json_folder + image_filename + ".json"
    lines_to_file(lines, filename)


def draw(lines):
    from tkinter import Tk, LEFT
    from turtle import Canvas, RawTurtle, TurtleScreen

    # set up the environment
    root = Tk()
    canvas = Canvas(root, width=800, height=800)
    canvas.pack()

    s = TurtleScreen(canvas)

    t = RawTurtle(canvas)
    t.speed(0)
    t.width(1)

    for line in lines:
        x, y = line[0]
        t.up()
        t.goto(x*800/1024-400,-(y*800/1024-400))
        for point in line:
            t.down()
            t.goto(point[0]*800/1024-400,-(point[1]*800/1024-400))

    s.mainloop()


def vectorise(
    image_filename,
    resolution=1024,
    draw_hatch=True,
    hatch_size = 16,
    draw_contours=True,
    contour_simplify=1,
    ):

    image = None
    possible = [
        image_filename,
        "images/"+image_filename,
        "images/"+image_filename+".jpg",
        "images/"+image_filename+".png",
        "images/"+image_filename+".tif"
    ]

    for p in possible:
        try:
            image = Image.open(p)
            break
        except:
            pass
    w,h = image.size

    # convert the image to greyscale
    image = image.convert("L")

    # maximise contrast
    image=ImageOps.autocontrast(image, 10)

    lines = []

    if draw_contours:
        lines += sortlines(getcontours(
            image.resize((int(resolution/contour_simplify), int(resolution/contour_simplify*h/w))),
            contour_simplify,
        ))

    if draw_hatch:
        lines += sortlines(
            hatch(
                # image,
                image.resize((int(resolution/hatch_size), int(resolution/hatch_size*h/w))),
                hatch_size,
            )
        )

    f = open(svg_folder + image_filename + ".svg",'w')
    f.write(makesvg(lines))
    f.close()
    segments = 0
    for line in lines:
        segments = segments + len(line)
    print(len(lines), "strokes,", segments, "points.")
    print("done.")
    return lines


def find_edges(image):
    print("finding edges...")
    if no_cv:
        #appmask(IM,[F_Blur])
        appmask(image,[F_SobelX,F_SobelY])
    else:
        im = np.array(image)
        im = cv2.GaussianBlur(im,(3,3),0)
        im = cv2.Canny(im,100,200)
        image = Image.fromarray(im)
    return image.point(lambda p: p > 128 and 255)


def getdots(IM):
    print("getting contour points...")
    PX = IM.load()
    dots = []
    w,h = IM.size
    for y in range(h-1):
        row = []
        for x in range(1,w):
            if PX[x,y] == 255:
                if len(row) > 0:
                    if x-row[-1][0] == row[-1][-1]+1:
                        row[-1] = (row[-1][0],row[-1][-1]+1)
                    else:
                        row.append((x,0))
                else:
                    row.append((x,0))
        dots.append(row)
    return dots

def connectdots(dots):
    print("connecting contour points...")
    contours = []
    for y in range(len(dots)):
        for x,v in dots[y]:
            if v > -1:
                if y == 0:
                    contours.append([(x,y)])
                else:
                    closest = -1
                    cdist = 100
                    for x0,v0 in dots[y-1]:
                        if abs(x0-x) < cdist:
                            cdist = abs(x0-x)
                            closest = x0

                    if cdist > 3:
                        contours.append([(x,y)])
                    else:
                        found = 0
                        for i in range(len(contours)):
                            if contours[i][-1] == (closest,y-1):
                                contours[i].append((x,y,))
                                found = 1
                                break
                        if found == 0:
                            contours.append([(x,y)])
        for c in contours:
            if c[-1][1] < y-1 and len(c)<4:
                contours.remove(c)
    return contours


def getcontours(IM,sc=2):
    print("generating contours...")
    IM = find_edges(IM)
    IM1 = IM.copy()
    IM2 = IM.rotate(-90,expand=True).transpose(Image.FLIP_LEFT_RIGHT)
    dots1 = getdots(IM1)
    contours1 = connectdots(dots1)
    dots2 = getdots(IM2)
    contours2 = connectdots(dots2)

    for i in range(len(contours2)):
        contours2[i] = [(c[1],c[0]) for c in contours2[i]]
    contours = contours1+contours2

    for i in range(len(contours)):
        for j in range(len(contours)):
            if len(contours[i]) > 0 and len(contours[j])>0:
                if distsum(contours[j][0],contours[i][-1]) < 8:
                    contours[i] = contours[i]+contours[j]
                    contours[j] = []

    for i in range(len(contours)):
        contours[i] = [contours[i][j] for j in range(0,len(contours[i]),8)]


    contours = [c for c in contours if len(c) > 1]

    for i in range(0,len(contours)):
        contours[i] = [(v[0]*sc,v[1]*sc) for v in contours[i]]

    return contours


def hatch(IM,sc=16):
    print("hatching...")
    PX = IM.load()
    w,h = IM.size
    lg1 = []
    lg2 = []
    for x0 in range(w):
        # print("reading x", x0)
        for y0 in range(h):
            # print("    reading y", x0)
            x = x0 * sc
            y = y0 * sc



            # don't hatch above a certain level of brightness
            if PX[x0,y0] > 144:
                pass

            # above 64, draw horizontal lines
            elif PX[x0,y0] > 64:
                lg1.append([(x,y+sc/4),(x+sc,y+sc/4)])

            # above 16, draw diagonal lines
            elif PX[x0,y0] > 16:
                lg1.append([(x,y+sc/4),(x+sc,y+sc/4)])
                lg2.append([(x+sc,y),(x,y+sc)])

            # below 16, draw diagonal lines
            else:
                lg1.append([(x,y+sc/4),(x+sc,y+sc/4)])            # horizontal lines
                lg1.append([(x,y+sc/2+sc/4),(x+sc,y+sc/2+sc/4)])  # horizontal lines with additional offset
                lg2.append([(x+sc,y),(x,y+sc)])                   # diagonal lines, left

    print("wrangling points...")
    lines = [lg1,lg2]


    # The purpose if this is still unclear...

    for k in range(0,len(lines)):
        for i in range(0,len(lines[k])):
            for j in range(0,len(lines[k])):
                if lines[k][i] != [] and lines[k][j] != []:
                    if lines[k][i][-1] == lines[k][j][0]:
                        lines[k][i] = lines[k][i]+lines[k][j][1:]
                        lines[k][j] = []
        lines[k] = [l for l in lines[k] if len(l) > 0]
    lines = lines[0]+lines[1]

    return lines

    # The next section is an experimental routine to simplify and discard
    # lines, to make the hatching more efficient and less dense. It's disabled, because
    # it doesn't discard lines evenly.

    print("simplifying and discarding lines...")
    new_lines = []

    for line in lines[::2]: # increase the step value to discard hatch lines

        # a line is a list of (x, y) co-ordinates, e.g.:

        # [[0, 1], [2, 1], [3, 1], [3, 2], [3, 4], [4, 4]]

        # this could be simplified to:

        # [[0, 1],         [3, 1],         [3, 4], [4, 4]]

        # first we will check all the points to see which ones have x-values that need to be saved

        saved_points = []
        length = len(line)

        for index in range(length):
            # only test points that have neighbours
            if index > 0 and index < length -1:

                # if this point's x/y-values are not half-way between its neighbours, keep it
                x_point_is_disposable = (line[index-1][0] + line[index+1][0]) / 2 == line[index][0]
                y_point_is_disposable = (line[index-1][1] + line[index+1][1]) / 2 == line[index][1]

                if not x_point_is_disposable and not y_point_is_disposable:

                    saved_points.append(line[index])

            # this point must be the first or last in the list, so keep it
            else:
                saved_points.append(line[index])

        new_lines.append(saved_points)

    return new_lines



def makesvg(lines):
    print("generating svg file...")
    out = '<svg xmlns="http://www.w3.org/2000/svg" version="1.1">'
    for l in lines:
        l = ",".join([str(p[0]*0.5)+","+str(p[1]*0.5) for p in l])
        out += '<polyline points="'+l+'" stroke="black" stroke-width="2" fill="none" />\n'
    out += '</svg>'
    return out


def lines_to_file(lines, filename):
    with open(filename, "w") as file_to_save:
        json.dump(lines, file_to_save, indent=4)




def sortlines(lines):
    print("optimizing stroke sequence...")
    clines = lines[:]
    slines = [clines.pop(0)]
    while clines != []:
        x,s,r = None,1000000,False
        for l in clines:
            d = distsum(l[0],slines[-1][-1])
            dr = distsum(l[-1],slines[-1][-1])
            if d < s:
                x,s,r = l[:],d,False
            if dr < s:
                x,s,r = l[:],s,True

        clines.remove(x)
        if r == True:
            x = x[::-1]
        slines.append(x)
    return slines


def midpt(*args):
    xs,ys = 0,0
    for p in args:
        xs += p[0]
        ys += p[1]
    return xs/len(args),ys/len(args)


def distsum(*args):
    return sum([ ((args[i][0]-args[i-1][0])**2 + (args[i][1]-args[i-1][1])**2)**0.5 for i in range(1,len(args))])


def sortlines(lines):
    print("optimizing stroke sequence...")
    clines = lines[:]
    slines = [clines.pop(0)]
    while clines != []:
        x,s,r = None,1000000,False
        for l in clines:
            d = distsum(l[0],slines[-1][-1])
            dr = distsum(l[-1],slines[-1][-1])
            if d < s:
                x,s,r = l[:],d,False
            if dr < s:
                x,s,r = l[:],s,True

        clines.remove(x)
        if r == True:
            x = x[::-1]
        slines.append(x)
    return slines


def appmask(IM,masks):
    PX = IM.load()
    w,h = IM.size
    NPX = {}
    for x in range(0,w):
        for y in range(0,h):
            a = [0]*len(masks)
            for i in range(len(masks)):
                for p in masks[i].keys():
                    if 0<x+p[0]<w and 0<y+p[1]<h:
                        a[i] += PX[x+p[0],y+p[1]] * masks[i][p]
                if sum(masks[i].values())!=0:
                    a[i] = a[i] / sum(masks[i].values())
            NPX[x,y]=int(sum([v**2 for v in a])**0.5)
    for x in range(0,w):
        for y in range(0,h):
            PX[x,y] = NPX[x,y]

F_Blur = {
    (-2,-2):2,(-1,-2):4,(0,-2):5,(1,-2):4,(2,-2):2,
    (-2,-1):4,(-1,-1):9,(0,-1):12,(1,-1):9,(2,-1):4,
    (-2,0):5,(-1,0):12,(0,0):15,(1,0):12,(2,0):5,
    (-2,1):4,(-1,1):9,(0,1):12,(1,1):9,(2,1):4,
    (-2,2):2,(-1,2):4,(0,2):5,(1,2):4,(2,2):2,
}
F_SobelX = {(-1,-1):1,(0,-1):0,(1,-1):-1,(-1,0):2,(0,0):0,(1,0):-2,(-1,1):1,(0,1):0,(1,1):-1}
F_SobelY = {(-1,-1):1,(0,-1):2,(1,-1):1,(-1,0):0,(0,0):0,(1,0):0,(-1,1):-1,(0,1):-2,(1,1):-1}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert image to vectorized line drawing for plotters.')
    parser.add_argument('-i','--input',dest='input_path',
        default='lenna',action='store',nargs='?',type=str,
        help='Input path')

    parser.add_argument('-o','--output',dest='output_path',
        default=export_path,action='store',nargs='?',type=str,
        help='Output path.')

    parser.add_argument('-b','--show_bitmap',dest='show_bitmap',
        const = not show_bitmap,default= show_bitmap,action='store_const',
        help="Display bitmap preview.")

    parser.add_argument('-nc','--no_contour',dest='no_contour',
        const = draw_contours,default= not draw_contours,action='store_const',
        help="Don't draw contours.")

    parser.add_argument('-nh','--no_hatch',dest='no_hatch',
        const = draw_hatch,default= not draw_hatch,action='store_const',
        help='Disable hatching.')

    parser.add_argument('--no_cv',dest='no_cv',
        const = not no_cv,default= no_cv,action='store_const',
        help="Don't use openCV.")


    parser.add_argument('--hatch_size',dest='hatch_size',
        default=hatch_size,action='store',nargs='?',type=int,
        help='Patch size of hatches. eg. 8, 16, 32')
    parser.add_argument('--contour_simplify',dest='contour_simplify',
        default=contour_simplify,action='store',nargs='?',type=int,
        help='Level of contour simplification. eg. 1, 2, 3')

    args = parser.parse_args()

    export_path = args.output_path
    draw_hatch = not args.no_hatch
    draw_contours = not args.no_contour
    hatch_size = args.hatch_size
    contour_simplify = args.contour_simplify
    show_bitmap = args.show_bitmap
    no_cv = args.no_cv
    sketch(args.input_path)