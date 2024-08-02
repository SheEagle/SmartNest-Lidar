import rospy
import message_filters
from sensor_msgs.msg import LaserScan
import time
import os

output_file1 = open(os.getcwd()+"/lidar_data/S/"+time.strftime('%Y-%m-%d', time.localtime())+"_before.txt", "a+")
output_file1.truncate(0)
output_file2 = open(os.getcwd()+"/lidar_data/X/"+time.strftime('%Y-%m-%d', time.localtime())+"_before.txt", "a+")
output_file2.truncate(0)

def callback(msg1, msg2):
    msg_time1 = msg1.header.stamp
    seconds1 = msg_time1.secs
    nanoseconds1 = msg_time1.nsecs
    seconds_time1 = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(seconds1))
    millisecond1 = nanoseconds1 / 1e6
    rounded_nano1 = str(round(millisecond1 / 1e3, 3)).lstrip('0')
    if rounded_nano1[0] == '1':
        rounded_nano1 = '.999'
    rounded_ranges1 = [round(x, 3) for x in msg1.ranges]
    output_file1.write(seconds_time1 + rounded_nano1 + "\r\n")
    output_file1.write(str(rounded_ranges1).rstrip(']').lstrip('[').replace(" ", "") + "\r\n")

    msg_time2 = msg2.header.stamp
    seconds2 = msg_time2.secs
    nanoseconds2 = msg_time2.nsecs
    seconds_time2 = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(seconds2))
    millisecond2 = nanoseconds2 / 1e6
    rounded_nano2 = str(round(millisecond2 / 1e3, 3)).lstrip('0')
    if rounded_nano2[0] == '1':
        rounded_nano2 = '.999'
    rounded_ranges2 = [round(x, 3) for x in msg2.ranges]
    output_file2.write(seconds_time2 + rounded_nano2 + "\r\n")
    output_file2.write(str(rounded_ranges2).rstrip(']').lstrip('[').replace(" ", "") + "\r\n")


if __name__ == "__main__":
    rospy.init_node("listener")

    sub1 = message_filters.Subscriber('/scan1', LaserScan)
    sub2 = message_filters.Subscriber('/scan2', LaserScan)

    ats = message_filters.ApproximateTimeSynchronizer([sub1, sub2], 10, 0.05, allow_headerless=True)
    ats.registerCallback(callback)
    rospy.spin()