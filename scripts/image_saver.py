#!/usr/bin/env python
import rospy
import cv2
from Tkinter import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import ImageTk
from PIL import Image as pilimg
from pressure_pad.msg import pressure_read

image = 0
tk_img = 0

root = Tk()
entry = Entry(root)

rivet_count_entry = Entry(root)
rivet_pos_1 = Entry(root)
rivet_pos_2 = Entry(root)
rivet_pos_3 = Entry(root)
rivet_pos_4 = Entry(root)
rivet_pos_5 = Entry(root)
rivet_pos_6 = Entry(root)
rivet_pos_7 = Entry(root)

safe_temp = False
is_flat_temp = False
has_slipped_temp = False

pad_force_temp = 0.0
rivet_count_temp = 0
rivet_pos_temp = ""
force_per_temp = ""


def callback(data):
    bridge = CvBridge()
    global image

    image = bridge.imgmsg_to_cv2(data, "mono8")
    cv2.namedWindow("found", cv2.WINDOW_NORMAL)
    cv2.imshow("found", image)

    cv2.waitKey(3)


def largeCallback(data):
    global safe_temp
    global is_flat_temp
    global has_slipped_temp

    global pad_force_temp
    global rivet_count_temp
    global rivet_pos_temp
    global force_per_temp

    safe_temp = data.safe
    is_flat_temp = data.is_flat
    has_slipped_temp = data.has_slipped

    pad_force_temp = data.pad_force
    rivet_count_temp = data.rivet_count

    temp = data.rivet_pos
    force_per_temp = data.force_per_cell

    rivet_pos_temp = ""

    for pos in xrange(0, rivet_count_temp, 1):
        rivet_pos_temp = rivet_pos_temp + "( " + ('%.2f' % temp[pos].x) + " , " + ('%.2f' % temp[pos].y) + " ) , "


def repeater(root2, safe_reading, is_flat_reading, has_slipped_reading, pad_force_reading, rivet_count_reading
             , rivet_pos_reading):
    global safe_temp
    global is_flat_temp
    global has_slipped_temp

    global pad_force_temp
    global rivet_count_temp

    global rivet_pos_temp

    global image

    safe_reading.configure(text=safe_temp)
    is_flat_reading.configure(text=is_flat_temp)
    has_slipped_reading.configure(text=has_slipped_temp)
    pad_force_reading.configure(text=pad_force_temp)
    rivet_count_reading.configure(text=rivet_count_temp)
    rivet_pos_reading.configure(text=rivet_pos_temp)

    root2.after(1, repeater, root2, safe_reading, is_flat_reading, has_slipped_reading, pad_force_reading
                , rivet_count_reading, rivet_pos_reading)


def write():
    global safe_temp
    global is_flat_temp
    global has_slipped_temp

    global pad_force_temp
    global rivet_count_temp
    global force_per_temp

    rivets = rivet_count_entry.get()

    if image is not 0 and rivets is not "":
        rivets_int = int(rivets)
        if entry.get() is "":
            print "no image name given"
            return
        location = "/home/denis/Desktop/rivet_read/" + rivets + "/"
        name = location + entry.get() + ".jpg"
        cv2.imwrite(name, image)

        f = open(location + rivets + ".txt", "a")
        f.write(entry.get() + ",")
        f.write(str(pad_force_temp) + ",")
        f.write(rivet_pos_1.get() + ",")
        if rivets_int >= 2:
            f.write(rivet_pos_2.get() + ",")
            if rivets_int >= 3:
                f.write(rivet_pos_3.get() + ",")
                if rivets_int >= 4:
                    f.write(rivet_pos_4.get() + ",")
                    if rivets_int >= 5:
                        f.write(rivet_pos_5.get() + ",")
                        if rivets_int >= 6:
                            f.write(rivet_pos_6.get() + ",")
                            if rivets_int >= 7:
                                f.write(rivet_pos_7.get() + ",")
        f.write("\n")
        f.close()

        forces = open(location + "force_reading.txt", "a")

        forces.write(entry.get() + ",")
        x = 0
        for cell in force_per_temp:
            x += 1
            forces.write(str(cell) + ",")

        forces.write("\n")
        forces.close()

        print x

    else:
        print "no image input yet"


def main():
    global root
    global tk_img

    global safe_temp
    global is_flat_temp
    global has_slipped_temp

    global pad_force_temp
    global rivet_count_temp
    global rivet_pos_temp

    global rivet_count_entry
    global rivet_pos_1
    global rivet_pos_2
    global rivet_pos_3
    global rivet_pos_4
    global rivet_pos_5
    global rivet_pos_6
    global rivet_pos_7

    # root.attributes("-fullscreen", True) #use this for making the gui go fullscreen

    rospy.init_node('img_writer', anonymous=True)
    rospy.Subscriber("camera/image", Image, callback)
    rospy.Subscriber("/wallpusher/reading/left", pressure_read, largeCallback)

    panel = Label(root, text="image name: ")
    tot_rivets_panel = Label(root, text="Rivet Count: ")
    pos1 = Label(root, text="pos1: ")
    pos2 = Label(root, text="pos2: ")
    pos3 = Label(root, text="pos3: ")
    pos4 = Label(root, text="pos4: ")
    pos5 = Label(root, text="pos5: ")
    pos6 = Label(root, text="pos6: ")
    pos7 = Label(root, text="pos7: ")

    submit = Button(root, text="Submit", command=write)
    safe_panel = Label(root, text="Safe: ")
    is_flat_panel = Label(root, text="is flat: ")
    has_slipped_panel = Label(root, text="has_slipped: ")
    pad_force_panel = Label(root, text="pad_force: ")
    rivet_count_panel = Label(root, text="No. Rivets: ")
    rivet_pos_panel = Label(root, text="rivet pos (x,y): ")

    safe_reading = Label(root, text=safe_temp)
    is_flat_reading = Label(root, text=is_flat_temp)
    has_slipped_reading = Label(root, text=has_slipped_temp)
    pad_force_reading = Label(root, text=pad_force_temp)
    rivet_count_reading = Label(root, text=rivet_count_temp)
    rivet_pos_reading = Label(root, text=rivet_pos_temp)

    tk_img = cv2.imread("1.jpg")
    tk_img = cv2.cvtColor(tk_img, cv2.COLOR_BGR2RGB)
    tk_img = pilimg.fromarray(tk_img)
    tk_img = ImageTk.PhotoImage(tk_img)

    panel.grid(row=0)
    entry.grid(row=0, column=1)

    tot_rivets_panel.grid(row=1)
    rivet_count_entry.grid(row=1, column=1)

    pos1.grid(row=0, column=2)
    pos2.grid(row=1, column=2)
    pos3.grid(row=2, column=2)
    pos4.grid(row=3, column=2)
    pos5.grid(row=4, column=2)
    pos6.grid(row=5, column=2)
    pos7.grid(row=6, column=2)

    rivet_pos_1.grid(row=0, column=3)
    rivet_pos_2.grid(row=1, column=3)
    rivet_pos_3.grid(row=2, column=3)
    rivet_pos_4.grid(row=3, column=3)
    rivet_pos_5.grid(row=4, column=3)
    rivet_pos_6.grid(row=5, column=3)
    rivet_pos_7.grid(row=6, column=3)

    safe_panel.grid(row=2)
    is_flat_panel.grid(row=3)
    has_slipped_panel.grid(row=4)
    pad_force_panel.grid(row=5)
    rivet_count_panel.grid(row=6)
    rivet_pos_panel.grid(row=7)

    safe_reading.grid(row=2, column=1)
    is_flat_reading.grid(row=3, column=1)
    has_slipped_reading.grid(row=4, column=1)
    pad_force_reading.grid(row=5, column=1)
    rivet_count_reading.grid(row=6, column=1)
    rivet_pos_reading.grid(row=7, column=1)

    submit.grid(row=8, column=0)

    root.after(1, repeater, root, safe_reading, is_flat_reading, has_slipped_reading, pad_force_reading,
               rivet_count_reading, rivet_pos_reading)
    root.mainloop()


if __name__ == '__main__':
    main()
