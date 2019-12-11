;; Auto-generated. Do not edit!


(when (boundp 'ackermann_msgs::AckermannDriveStamped)
  (if (not (find-package "ACKERMANN_MSGS"))
    (make-package "ACKERMANN_MSGS"))
  (shadow 'AckermannDriveStamped (find-package "ACKERMANN_MSGS")))
(unless (find-package "ACKERMANN_MSGS::ACKERMANNDRIVESTAMPED")
  (make-package "ACKERMANN_MSGS::ACKERMANNDRIVESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude AckermannDriveStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ackermann_msgs::AckermannDriveStamped
  :super ros::object
  :slots (_header _drive ))

(defmethod ackermann_msgs::AckermannDriveStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drive __drive) (instance ackermann_msgs::AckermannDrive :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _drive __drive)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:drive
   (&rest __drive)
   (if (keywordp (car __drive))
       (send* _drive __drive)
     (progn
       (if __drive (setq _drive (car __drive)))
       _drive)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; ackermann_msgs/AckermannDrive _drive
    (send _drive :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; ackermann_msgs/AckermannDrive _drive
       (send _drive :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; ackermann_msgs/AckermannDrive _drive
     (send _drive :deserialize buf ptr-) (incf ptr- (send _drive :serialization-length))
   ;;
   self)
  )

(setf (get ackermann_msgs::AckermannDriveStamped :md5sum-) "1fd5d7f58889cefd44d29f6653240d0c")
(setf (get ackermann_msgs::AckermannDriveStamped :datatype-) "ackermann_msgs/AckermannDriveStamped")
(setf (get ackermann_msgs::AckermannDriveStamped :definition-)
      "## Time stamped drive command for robots with Ackermann steering.
#  $Id$

Header          header
AckermannDrive  drive

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: ackermann_msgs/AckermannDrive
## Driving command for a car-like vehicle using Ackermann steering.
#  $Id$

# Assumes Ackermann front-wheel steering. The left and right front
# wheels are generally at different angles. To simplify, the commanded
# angle corresponds to the yaw of a virtual wheel located at the
# center of the front axle, like on a tricycle.  Positive yaw is to
# the left. (This is *not* the angle of the steering wheel inside the
# passenger compartment.)
#
# Zero steering angle velocity means change the steering angle as
# quickly as possible. Positive velocity indicates a desired absolute
# rate of change either left or right. The controller tries not to
# exceed this limit in either direction, but sometimes it might.
#
float32 steering_angle          # desired virtual angle (radians)
float32 steering_angle_velocity # desired rate of change (radians/s)

# Drive at requested speed, acceleration and jerk (the 1st, 2nd and
# 3rd derivatives of position). All are measured at the vehicle's
# center of rotation, typically the center of the rear axle. The
# controller tries not to exceed these limits in either direction, but
# sometimes it might.
#
# Speed is the desired scalar magnitude of the velocity vector.
# Direction is forward unless the sign is negative, indicating reverse.
#
# Zero acceleration means change speed as quickly as
# possible. Positive acceleration indicates a desired absolute
# magnitude; that includes deceleration.
#
# Zero jerk means change acceleration as quickly as possible. Positive
# jerk indicates a desired absolute rate of acceleration change in
# either direction (increasing or decreasing).
#
float32 speed                   # desired forward speed (m/s)
float32 acceleration            # desired acceleration (m/s^2)
float32 jerk                    # desired jerk (m/s^3)

")



(provide :ackermann_msgs/AckermannDriveStamped "1fd5d7f58889cefd44d29f6653240d0c")


