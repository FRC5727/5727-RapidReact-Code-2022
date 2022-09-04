#!/usr/bin/python3

import argparse
import pathlib
import sys

# vvv CUT HERE FOR LIMELIGHT vvv ###

import cv2
import numpy as np
import operator

interactive = False
tick_count = 0
counter = 0
#hsv_min = (55, 120, 105) #ncgui-0
#hsv_max = (92, 215, 255) #ncgui-0
#hsv_min = (69, 107, 15) #llp tuning
#hsv_max = (99, 255, 255) #llp tuning
#hsv_min = (60, 107, 30)  # pre-cmp
#hsv_max = (99, 255, 255) # pre-cmp
hsv_min = (60, 107, 50)
hsv_max = (95, 255, 255)
area_max_pct = 0.1400
area_min_pct = 0.0015
angle_max = 60
aspect_min = 1
aspect_max = 7
min_fill_pct = 15
morph_op = cv2.MORPH_CLOSE
morph_kernel_size = 4
xdiff_min = 1.5
xdiff_max = 3.0
ydiff_max = 1.6 # Based on a multiple of the *width*
debug_images = {}

colors = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'yellow': (0, 255, 255),
    'magenta': (255, 0, 255),
    'cyan': (255, 255, 0),
    'white': (255, 255, 255),
    'gray': (128, 128, 128)
}

# runPipeline() is called every frame by Limelight's backend
def runPipeline(image, llrobot):
    global counter
    global debug_images
    debug_images = {}

    # Get image size
    img_height = image.shape[0]
    img_width = image.shape[1]
    img_area = (img_height * img_width)
    
    # Filter image by color
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_hsv, hsv_min, hsv_max)
    if interactive:
        debug_images['threshold'] = img_threshold
    
    # For debug, show masked colors
    if interactive:
        debug_images['masked'] = cv2.bitwise_and(image, image, mask = img_threshold)
    
    # Dilate and erode to smooth out image
    kernel = np.ones((morph_kernel_size, morph_kernel_size), np.uint8)
    img_morph = cv2.morphologyEx(img_threshold, morph_op, kernel)
    if interactive:
        debug_images['morph'] = img_morph

    # Find contours (groups of pixels)
    contours, _ = cv2.findContours(img_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  
    finalContour = np.array([[]])
    llpython = [0,0,0,0,0,0,0,0,0]
    best = None
    if len(contours) > 0:
        cv2.drawContours(image, contours, -1, colors['magenta'], 1)
        good_contours = []
        for cnt in contours:
            M = cv2.moments(cnt)
            area = M['m00']
            if area == 0:
                continue
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            # Ensure that the contour is within the size bounds
            area_pct = 100.0 * area / img_area
            if (not area_min_pct < area_pct < area_max_pct):
                if counter == 0:
                    print("Rejected contour at ({}, {}) with {:4f} percent area".format(cx, cy, area_pct))
                if area_pct > 0:
                    cv2.drawContours(image, [cnt], 0, colors['red'], 1)
                continue
            
            # Find the minimum bounding rectangle (could be angled)
            rect = cv2.minAreaRect(cnt)
            (x, y), (w, h), angle = rect
            box = np.intp(cv2.boxPoints(rect))
            boxArea = cv2.contourArea(box)
            if counter == 0:
                print("Box at ({}, {}) at angle {:.1f} with size {} x {}: {}".format(int(x+.5), int(y+.5), angle, int(w+.5), int(h+.5), box))
            
            # Reject anything without area
            if boxArea == 0:
                continue

            # Adjust rectangle to standard configuration
            if h > w:
                angle = angle + 90
                w, h = h, w
                if counter == 0:
                    print("Rotating from angle {:.1f} and {} x {}".format(angle, w, h))
                box = np.array([box[1], box[2], box[3], box[0]])

            # Filter by angle
            if angle_max < angle < (180 - angle_max):
                if counter == 0:
                    print("Rejecting based on angle: {:1f}".format(angle))
                cv2.drawContours(image, [box], 0, colors['white'], 1)
                continue

            # Filter by the aspect ratio
            aspect = w / h
            if counter == 0:
                print("Aspect ratio at ({}, {}): {:4f}".format(cx, cy, aspect))
            if not aspect_min <= aspect <= aspect_max:
                if counter == 0:
                    print("Rejecting based on aspect ratio: {:4f}".format(aspect))
                cv2.drawContours(image, [box], 0, colors['cyan'], 1)
                continue

            # Filter by the amount the contour fills the bounding box
            box_mask = np.zeros_like(img_threshold)
            cv2.drawContours(box_mask, [box], -1, 255, cv2.FILLED)
            masked_box = img_threshold & box_mask
            filled = np.count_nonzero(masked_box)
            totalArea = np.count_nonzero(box_mask)
            filled_pct = 100.0 * filled / totalArea
            
            if counter == 0:
                print("Filled at ({}, {}): {:2f}".format(cx, cy, filled_pct))
            
            if filled_pct < min_fill_pct:
                cv2.drawContours(image, [box], 0, colors['gray'], 1)
                continue

            # Save for later processing
            good_contours.append([cnt, box, cx, cy, w, h])
            
        if counter == 0:
            print("Good contours: {}".format(len(good_contours)))
            
        # Search left to right to find pairs (or more)
        good_sorted = sorted(good_contours, key=operator.itemgetter(2))
        left_sorted = []
        saved_list = []
        for cnt_info in good_sorted:
            (cnt, box, cx, cy, w, h) = cnt_info
            cv2.drawContours(image, [box], 0, colors['blue'], 1)
            if counter == 0:
                print("Good: {:.2f} x {:.2f} @ {}, {}".format(w, h, cx, cy))

            # Look at all other contours
            left = None
            for other in reversed(left_sorted):
                xdiff = (cx - other[2]) / w
                xdiff2 = (cx - other[2]) / other[4]
                ydiff = abs(cy - other[3]) / max(w, other[4])
                #if counter == 0:
                #    print("DIFF = {:2.2f} / {:2.2f}, {:2.2f}".format(xdiff, xdiff2, ydiff))
                    
                # Rule out based on improper spacing
                if xdiff < xdiff_min and xdiff2 < xdiff_min:
                    continue
                if xdiff > xdiff_max and xdiff2 > xdiff_max:
                    break
                if ydiff > ydiff_max:
                    continue
                            
                # Found a target to the left 
                if counter == 0:
                    print("Found left @ {}, {}".format(other[2], other[3]))
                
                # Record best match it nothing found yet
                if best is None:
                    if counter == 0:
                        print("New best")
                    best = [other, cnt_info]
                    break
                
                # Add to the best match if this is a continuation
                elif best[-1][2] == other[2] and best[-1][3] == other[3]:
                    if counter == 0:
                        print("Added to best")
                    best.append(cnt_info)
                    break
                
                else:
                    # Add to what's saved if applicable
                    for saved in saved_list:
                        if counter == 0:
                            print("Last saved: {}, {} vs left {}, {}".format(saved[-1][2], saved[-1][3], other[2], other[3]))
                        if saved[-1][2] == other[2] and saved[-1][3] == other[3]:
                            if counter == 0:
                                print("Adding to saved")
                            saved_new = saved + [cnt_info]
                            saved_list.append(saved_new)
                            
                            # Take saved if it's better than the best
                            if len(saved_new) > len(best):
                                if counter == 0:
                                    print("Replacing best with saved (len {})".format(len(saved_new)))
                                best = saved_new
                            break
                                    
                    # Consider if this can replace the best match (should be rare)
                    if len(best) <= 2 and cy < best[-1][3]: # TODO Better tie-breaker algorithm than just highest?
                        if counter == 0:
                            print("Replaced best")
                        best = [other, cnt_info]
                        break
                        
                    # Not better, but save it, because maybe it will be
                    saved_list.append([other, cnt_info])
                    if counter == 0:
                        print("Saved for later consideration (total {})".format(len(saved_list)))
                
            # Add counter to list of those processed and on the left
            left_sorted.append(cnt_info)

    if best is not None:
        if counter == 0:
            print("Best count: {}".format(len(best)))
               
        # Build a hull around all items in the best match
        list_of_pts = [] 
        for ctr_info in best:
            list_of_pts += [pt[0] for pt in ctr_info[0]]
        ctr = np.array(list_of_pts).reshape((-1,1,2)).astype(np.int32)
        hull = cv2.convexHull(ctr)
        cv2.drawContours(image, [hull], -1, colors['yellow'], 1)

        # Set the target to be in the middle, at the highest point of all targets
        xavg = int(sum(map(operator.itemgetter(2), best)) / len(best))
        ytop = min(map(operator.itemgetter(3), best))

        # Return a contour surrounding the target point
        finalContour = np.array([[xavg+1,ytop+1],[xavg+1,ytop-1],[xavg-1,ytop-1],[xavg,ytop+1]]).reshape((-1,1,2)).astype(np.int32)

        # Return target information via network tables (but also useful locally)
        llpython[0] = 1
        llpython[1] = xavg
        llpython[2] = ytop
    else:
        cv2.putText(image, 'No Target!', (0, 230), cv2.FONT_HERSHEY_SIMPLEX, .5, colors['red'], 1, cv2.LINE_AA)

    if not interactive:
        counter = counter + 1
        if tick_count > 0 and counter >= tick_count:
            print("=== TICK ===")
            counter = 0
        
    return finalContour, image, llpython

# ^^^ CUT HERE FOR LIMELIGHT ^^^ ###

def draw_crosshair(img, x, y, color):
    size = 3
    thickness = 2
    cv2.line(img, (x+thickness, y), (x+thickness+size, y), color, thickness)
    cv2.line(img, (x-thickness, y), (x-thickness-size, y), color, thickness)
    cv2.line(img, (x, y+thickness), (x, y+thickness+size), color, thickness)
    cv2.line(img, (x, y-thickness), (x, y-thickness-size), color, thickness)

# MAIN
if __name__=="__main__":
    # Process arguments
    parser = argparse.ArgumentParser(description='Perform vision analysis on supplied input')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--update', action="store_true", help="update the saved values")
    group.add_argument('--diff', action="store_true", help="only highlight the changes compared to saved values")
    parser.add_argument('--batch', action="store_true", help="disable interactive processing")
    parser.add_argument('file', nargs="+")
    args = parser.parse_args()

    # Batch option disables interaction with the results
    interactive = not args.batch

    # Initialize counters
    totalDiff = 0
    totalSame = 0

    # Process all images sequentially
    for fn in args.file:
        # Read in the image
        img = cv2.imread(fn)
        showImage = interactive
        llvfn = fn + ".llv"

        # The Limelight is inverted, so rotate the image
        img = cv2.rotate(img, cv2.ROTATE_180)

        # Execute the pipeline on the image
        print("\n=== Running pipeline on {}".format(fn))
        (primary, img_out, llp) = runPipeline(img, [])
        if llp[0] > 0:
            print("Target @ {}".format(llp[1:3]))
        else:
            print("No target!")

        # In diff mode, compare target information to that previously recorded
        if args.diff:
            same = False

            # Read in previous target information
            llvp = pathlib.Path(llvfn)
            if llvp.is_file():
                with open(llvfn) as llvf:
                    known = [int(x) for x in next(llvf).split()]

                # Compare target information
                print("Known target info: {}".format(known))
                if set(known) == set(llp[:3]):
                    same = True

            if same:
                totalSame = totalSame + 1
                print("No change to target information")
                showImage = False
            else:
                totalDiff = totalDiff + 1
                if (llp[0] == 0) != (known[0] == 0):
                    if llp[0] == 0:
                        print("Lost target!")
                    else:
                        print("Found target!")
                else:
                    print("Target location change from {} to {}".format(known[1:3], llp[1:3]))

                if known[0]:
                    draw_crosshair(img_out, known[1], known[2], colors['red'])

        # If updating, save this target information as the new baseline
        if args.update:
            with open(llvfn, 'w') as llvf:
                llvf.write("{} {} {}".format(llp[0], llp[1], llp[2]))

        # Ensure debugging output is flushed to the console
        sys.stdout.flush()

        # Display final image and any debug aids
        if showImage:
            if llp[0]:
                draw_crosshair(img_out, llp[1], llp[2], colors['green'])
            debug_images[fn] = img_out
            for name in debug_images.keys():
                resized = cv2.resize(debug_images[name], None, fx = 2, fy = 2)
                cv2.imshow(name, resized)
            cv2.waitKey(0)
        cv2.destroyAllWindows()

    # In diff mode, summarize results
    if args.diff:
        print("\nSUMMARY: {} identical targets, {} changed targets".format(totalSame, totalDiff))
