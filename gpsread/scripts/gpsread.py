#-*- coding:utf-8 –*-
import rospy
import serial
import time

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Header


timestamp = ""
latitude = ""
longitude = ""
altitude = ""
status = ""
satellite_num = ""
location_accuracy = ""

gps_pub = rospy.Publisher('/gps', NavSatFix, queue_size=10) # 定义发布的信息
rospy.init_node('gps_publisher', anonymous=True)

def subInfo(gps_info, comma_count, start_ind, end_ind): # 将消息根据逗号位置提取出来
    if(comma_count is 2): 
        global timestamp
        timestamp = gps_info[start_ind+1:end_ind]
    if(comma_count is 3): 
        global latitude
        latitude = gps_info[start_ind+1:end_ind]
    if(comma_count is 5):
        global longitude
        longitude = gps_info[start_ind+1:end_ind]
    if(comma_count is 7):
        global status
        status = gps_info[start_ind+1:end_ind]
    if(comma_count is 8):
        global satellite_num
        satellite_num = gps_info[start_ind+1:end_ind]
    if(comma_count is 9):
        global location_accuracy
        location_accuracy = gps_info[start_ind+1:end_ind]
    if(comma_count is 10):
        global altitude
        altitude = gps_info[start_ind+1:end_ind]
        gpsTopicPublush(timestamp,latitude,longitude,status,
              satellite_num,location_accuracy,altitude)

def subGPSstr(gps_info): 
    print "subGPSstr : ", gps_info
    comma_count = 0
    start_ind = 0
    for i in range(len(gps_info)): # 判断逗号的位置
        if(gps_info[i] is ','):
            #print comma_count
            comma_count= comma_count + 1
            subInfo(gps_info, comma_count, start_ind, i)
            start_ind = i

def gpsTopicPublush(timestamp,latitude,longitude,status,  #将得到的信息组装成 NavSatStatus 消息类型 
              satellite_num,location_accuracy,altitude):

    # Header
    header_ins = Header( frame_id = "gps_link", stamp = rospy.Time.now() )
    # NavSatStatus
    if(status is '0'):
        gps_status = -1;
    else:
        gps_status = 0

    navsatstatus_ins = NavSatStatus(status = gps_status)
    
    navstafix_ins = NavSatFix(header = header_ins, 
                        status = navsatstatus_ins,
                        latitude =  float(latitude),
                        longitude = float(longitude),
                        altitude = float(altitude),
                         )
    
    global gps_pub
    gps_pub.publish(navstafix_ins)

        
if __name__ == '__main__':

    ser=serial.Serial("/dev/ttyACM0",9600,timeout=0.5) # 打开串口的位置
    #ser.open () #打开端口
    gps_str = ""
    while(1):
        s = ser.read(65) #从端口读65个字节
        if(len(s) > 50 and "\000" not in s and "\r" not in s): # 排除不好的消息类型
            subGPSstr(s) 

