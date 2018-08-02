import dlib

img = dlib.load_rgb_image('stop sign.png')

rects = []
dlib.find_candidate_object_locations(img, rects, min_size = 500)

print("number of rectangles found {}".format(len(rects))) 
for k, d in enumerate(rects):
    print("Detection {}: Left: {} Top: {} Right: {} Bottom: {}".format(
        k, d.left(), d.top(), d.right(), d.bottom()))
