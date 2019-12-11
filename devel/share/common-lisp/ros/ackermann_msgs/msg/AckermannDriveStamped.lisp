; Auto-generated. Do not edit!


(cl:in-package ackermann_msgs-msg)


;//! \htmlinclude AckermannDriveStamped.msg.html

(cl:defclass <AckermannDriveStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (drive
    :reader drive
    :initarg :drive
    :type ackermann_msgs-msg:AckermannDrive
    :initform (cl:make-instance 'ackermann_msgs-msg:AckermannDrive)))
)

(cl:defclass AckermannDriveStamped (<AckermannDriveStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AckermannDriveStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AckermannDriveStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ackermann_msgs-msg:<AckermannDriveStamped> is deprecated: use ackermann_msgs-msg:AckermannDriveStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AckermannDriveStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ackermann_msgs-msg:header-val is deprecated.  Use ackermann_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'drive-val :lambda-list '(m))
(cl:defmethod drive-val ((m <AckermannDriveStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ackermann_msgs-msg:drive-val is deprecated.  Use ackermann_msgs-msg:drive instead.")
  (drive m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AckermannDriveStamped>) ostream)
  "Serializes a message object of type '<AckermannDriveStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'drive) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AckermannDriveStamped>) istream)
  "Deserializes a message object of type '<AckermannDriveStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'drive) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AckermannDriveStamped>)))
  "Returns string type for a message object of type '<AckermannDriveStamped>"
  "ackermann_msgs/AckermannDriveStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AckermannDriveStamped)))
  "Returns string type for a message object of type 'AckermannDriveStamped"
  "ackermann_msgs/AckermannDriveStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AckermannDriveStamped>)))
  "Returns md5sum for a message object of type '<AckermannDriveStamped>"
  "1fd5d7f58889cefd44d29f6653240d0c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AckermannDriveStamped)))
  "Returns md5sum for a message object of type 'AckermannDriveStamped"
  "1fd5d7f58889cefd44d29f6653240d0c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AckermannDriveStamped>)))
  "Returns full string definition for message of type '<AckermannDriveStamped>"
  (cl:format cl:nil "## Time stamped drive command for robots with Ackermann steering.~%#  $Id$~%~%Header          header~%AckermannDrive  drive~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ackermann_msgs/AckermannDrive~%## Driving command for a car-like vehicle using Ackermann steering.~%#  $Id$~%~%# Assumes Ackermann front-wheel steering. The left and right front~%# wheels are generally at different angles. To simplify, the commanded~%# angle corresponds to the yaw of a virtual wheel located at the~%# center of the front axle, like on a tricycle.  Positive yaw is to~%# the left. (This is *not* the angle of the steering wheel inside the~%# passenger compartment.)~%#~%# Zero steering angle velocity means change the steering angle as~%# quickly as possible. Positive velocity indicates a desired absolute~%# rate of change either left or right. The controller tries not to~%# exceed this limit in either direction, but sometimes it might.~%#~%float32 steering_angle          # desired virtual angle (radians)~%float32 steering_angle_velocity # desired rate of change (radians/s)~%~%# Drive at requested speed, acceleration and jerk (the 1st, 2nd and~%# 3rd derivatives of position). All are measured at the vehicle's~%# center of rotation, typically the center of the rear axle. The~%# controller tries not to exceed these limits in either direction, but~%# sometimes it might.~%#~%# Speed is the desired scalar magnitude of the velocity vector.~%# Direction is forward unless the sign is negative, indicating reverse.~%#~%# Zero acceleration means change speed as quickly as~%# possible. Positive acceleration indicates a desired absolute~%# magnitude; that includes deceleration.~%#~%# Zero jerk means change acceleration as quickly as possible. Positive~%# jerk indicates a desired absolute rate of acceleration change in~%# either direction (increasing or decreasing).~%#~%float32 speed                   # desired forward speed (m/s)~%float32 acceleration            # desired acceleration (m/s^2)~%float32 jerk                    # desired jerk (m/s^3)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AckermannDriveStamped)))
  "Returns full string definition for message of type 'AckermannDriveStamped"
  (cl:format cl:nil "## Time stamped drive command for robots with Ackermann steering.~%#  $Id$~%~%Header          header~%AckermannDrive  drive~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ackermann_msgs/AckermannDrive~%## Driving command for a car-like vehicle using Ackermann steering.~%#  $Id$~%~%# Assumes Ackermann front-wheel steering. The left and right front~%# wheels are generally at different angles. To simplify, the commanded~%# angle corresponds to the yaw of a virtual wheel located at the~%# center of the front axle, like on a tricycle.  Positive yaw is to~%# the left. (This is *not* the angle of the steering wheel inside the~%# passenger compartment.)~%#~%# Zero steering angle velocity means change the steering angle as~%# quickly as possible. Positive velocity indicates a desired absolute~%# rate of change either left or right. The controller tries not to~%# exceed this limit in either direction, but sometimes it might.~%#~%float32 steering_angle          # desired virtual angle (radians)~%float32 steering_angle_velocity # desired rate of change (radians/s)~%~%# Drive at requested speed, acceleration and jerk (the 1st, 2nd and~%# 3rd derivatives of position). All are measured at the vehicle's~%# center of rotation, typically the center of the rear axle. The~%# controller tries not to exceed these limits in either direction, but~%# sometimes it might.~%#~%# Speed is the desired scalar magnitude of the velocity vector.~%# Direction is forward unless the sign is negative, indicating reverse.~%#~%# Zero acceleration means change speed as quickly as~%# possible. Positive acceleration indicates a desired absolute~%# magnitude; that includes deceleration.~%#~%# Zero jerk means change acceleration as quickly as possible. Positive~%# jerk indicates a desired absolute rate of acceleration change in~%# either direction (increasing or decreasing).~%#~%float32 speed                   # desired forward speed (m/s)~%float32 acceleration            # desired acceleration (m/s^2)~%float32 jerk                    # desired jerk (m/s^3)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AckermannDriveStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'drive))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AckermannDriveStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'AckermannDriveStamped
    (cl:cons ':header (header msg))
    (cl:cons ':drive (drive msg))
))
