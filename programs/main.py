# Single Color RGB565 Blob Tracking Example
#
# This example shows off single color RGB565 tracking using the OpenMV Cam.

import sensor, image, time, math, pyb
from pyb import UART
ledred = pyb.LED(1)
ledgreen = pyb.LED(2)
threshold_index = 0 # 0 for red, 1 for green, 2 for blue

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds = [(0, 85, -128, -7, -46, 47)] # green
thresholds1 = [(0, 86, 10, 104, 27, 127)] # red
#thresholds = [(0, 70, -92, -12, -15, 62)] # green
#thresholds1 = [(4, 69, 32, 120, -69, 117)  ] # red
uart = UART(3, 115200)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
sensor.set_vflip(True)
sensor.set_hmirror(True)
#sensor.set_gainceiling()
#sensor.set_brightness(-3)
#sensor.set_contrast(-3)
#sensor.set_saturation(-3)
sensor.skip_frames(time = 2000)
clock = time.clock()
blob=0
# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.
cx=0
flag=0
while(True):
    flag=0
    clock.tick()
    img = sensor.snapshot()
    for blob in img.find_blobs(thresholds, roi=(0, 45, 160, 65), pixels_threshold=80, area_threshold=80, merge=True):

        # These values depend on the blob not being circular - otherwise they will be shaky.
        #if blob.rotation_deg() < 135 and blob.rotation_deg() > 45:

        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        # Note - the blob rotation is unique to 0-180 only.
        #img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)
        if blob.cx()>99:
            uart.write("g"+str(blob.cx()))
        if blob.cx()<100 and blob.cx()>9:
            uart.write("g0"+str(blob.cx()))
        if blob.cx()<10:
            uart.write("g00"+str(blob.cx()))
        if blob.h()<10:
            uart.write("0"+str(blob.h())+"\n")
        if blob.h()>9:
            uart.write(str(blob.h())+"\n")
        print("g")
        print(blob.h())
        ledgreen.on()
        flag=1




    for blob in img.find_blobs(thresholds1, roi=(0, 36, 160, 72), pixels_threshold=80, area_threshold=80, merge=False):
        #print(blob.rotation_deg())
        # These values depend on the blob not being circular - otherwise they will be shaky.
        if blob.rotation_deg() < 135 and blob.rotation_deg() > 45 and blob.elongation() < 0.8 and blob.density() > 0.6:
            # These values are stable all the time.
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            # Note - the blob rotation is unique to 0-180 only.
            #img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)

            if blob.cx()>99:
                uart.write("r"+str(blob.cx()))
            if blob.cx()<100 and blob.cx()>9:
                uart.write("r0"+str(blob.cx()))
            if blob.cx()<10:
                uart.write("r00"+str(blob.cx()))
            if blob.h()<10:
                uart.write("0"+str(blob.h())+"\n")
            if blob.h()>9:
                uart.write(str(blob.h())+"\n")
            #print("r")
            print(blob.density())
            ledred.on()
            flag=1




    if flag==0:
        #print("999")
        ledred.off()
        ledgreen.off()


    #print(clock.fps())

