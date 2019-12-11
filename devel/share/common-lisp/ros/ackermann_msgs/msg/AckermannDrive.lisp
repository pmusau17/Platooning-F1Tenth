; Auto-generated. Do not edit!


(cl:in-package ackermann_msgs-msg)


;//! \htmlinclude AckermannDrive.msg.html

(cl:defclass <AckermannDrive> (roslisp-msg-protocol:ros-message)
  ((steering_angle
    :reader steering_angle
    :initarg :steering_angle
    :type cl:float
    :initform 0.0)
   (steering_angle_velocity
    :reader steering_angle_velocity
    :initarg :steering_angle_velocity
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type cl:float
    :initform 0.0)
   (jerk
    :reader jerk
    :initarg :jerk
    :type cl:float
    :initform 0.0))
)

(cl:defclass AckermannDrive (<AckermannDrive>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AckermannDrive>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AckermannDrive)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ackermann_msgs-msg:<AckermannDrive> is deprecated: use ackermann_msgs-msg:AckermannDrive instead.")))

(cl:ensure-generic-function 'steering_angle-val :lambda-list '(m))
(cl:defmethod steering_angle-val ((m <AckermannDrive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ackermann_msgs-msg:steering_angle-val is deprecated.  Use ackermann_msgs-msg:steering_angle instead.")
  (steering_angle m))

(cl:ensure-generic-function 'steering_angle_velocity-val :lambda-list '(m))
(cl:defmethod steering_angle_velocity-val ((m <AckermannDrive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ackermann_msgs-msg:steering_angle_velocity-val is deprecated.  Use ackermann_msgs-msg:steering_angle_velocity instead.")
  (steering_angle_velocity m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <AckermannDrive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ackermann_msgs-msg:speed-val is deprecated.  Use ackermann_msgs-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <AckermannDrive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ackermann_msgs-msg:acceleration-val is deprecated.  Use ackermann_msgs-msg:acceleration instead.")
  (acceleration m))

(cl:ensure-generic-function 'jerk-val :lambda-list '(m))
(cl:defmethod jerk-val ((m <AckermannDrive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ackermann_msgs-msg:jerk-val is deprecated.  Use ackermann_msgs-msg:jerk instead.")
  (jerk m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AckermannDrive>) ostream)
  "Serializes a message object of type '<AckermannDrive>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering_angle_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'acceleration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'jerk))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AckermannDrive>) istream)
  "Deserializes a message object of type '<AckermannDrive>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_angle_velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acceleration) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'jerk) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AckermannDrive>)))
  "Returns string type for a message object of type '<AckermannDrive>"
  "ackermann_msgs/AckermannDrive")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AckermannDrive)))
  "Returns string type for a message object of type 'AckermannDrive"
  "ackermann_msgs/AckermannDrive")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AckermannDrive>)))
  "Returns md5sum for a message object of type '<AckermannDrive>"
  "3512e91b48d69674a0e86fadf1ea8231")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AckermannDrive)))
  "Returns md5sum for a message object of type 'AckermannDrive"
  "3512e91b48d69674a0e86fadf1ea8231")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AckermannDrive>)))
  "Returns full string definition for message of type '<AckermannDrive>"
  (cl:format cl:nil "## Driving command for a car-like vehicle using Ackermann steering.~%#  $Id$~%~%# Assumes Ackermann front-wheel steering. The left and right front~%# wheels are generally at different angles. To simplify, the commanded~%# angle corresponds to the yaw of a virtual wheel located at the~%# center of the front axle, like on a tricycle.  Positive yaw is to~%# the left. (This is *not* the angle of the steering wheel inside the~%# passenger compartment.)~%#~%# Zero steering angle velocity means change the steering angle as~%# quickly as possible. Positive velocity indicates a desired absolute~%# rate of change either left or right. The controller tries not to~%# exceed this limit in either direction, but sometimes it might.~%#~%float32 steering_angle          # desired virtual angle (radians)~%float32 steering_angle_velocity # desired rate of change (radians/s)~%~%# Drive at requested speed, acceleration and jerk (the 1st, 2nd and~%# 3rd derivatives of position). All are measured at the vehicle's~%# center of rotation, typically the center of the rear axle. The~%# controller tries not to exceed these limits in either direction, but~%# sometimes it might.~%#~%# Speed is the desired scalar magnitude of the velocity vector.~%# Direction is forward unless the sign is negative, indicating reverse.~%#~%# Zero acceleration means change speed as quickly as~%# possible. Positive acceleration indicates a desired absolute~%# magnitude; that includes deceleration.~%#~%# Zero jerk means change acceleration as quickly as possible. Positive~%# jerk indicates a desired absolute rate of acceleration change in~%# either direction (increasing or decreasing).~%#~%float32 speed                   # desired forward speed (m/s)~%float32 acceleration            # desired acceleration (m/s^2)~%float32 jerk                    # desired jerk (m/s^3)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AckermannDrive)))
  "Returns full string definition for message of type 'AckermannDrive"
  (cl:format cl:nil "## Driving command for a car-like vehicle using Ackermann steering.~%#  $Id$~%~%# Assumes Ackermann front-wheel steering. The left and right front~%# wheels are generally at different angles. To simplify, the commanded~%# angle corresponds to the yaw of a virtual wheel located at the~%# center of the front axle, like on a tricycle.  Positive yaw is to~%# the left. (This is *not* the angle of the steering wheel inside the~%# passenger compartment.)~%#~%# Zero steering angle velocity means change the steering angle as~%# quickly as possible. Positive velocity indicates a desired absolute~%# rate of change either left or right. The controller tries not to~%# exceed this limit in either direction, but sometimes it might.~%#~%float32 steering_angle          # desired virtual angle (radians)~%float32 steering_angle_velocity # desired rate of change (radians/s)~%~%# Drive at requested speed, acceleration and jerk (the 1st, 2nd and~%# 3rd derivatives of position). All are measured at the vehicle's~%# center of rotation, typically the center of the rear axle. The~%# controller tries not to exceed these limits in either direction, but~%# sometimes it might.~%#~%# Speed is the desired scalar magnitude of the velocity vector.~%# Direction is forward unless the sign is negative, indicating reverse.~%#~%# Zero acceleration means change speed as quickly as~%# possible. Positive acceleration indicates a desired absolute~%# magnitude; that includes deceleration.~%#~%# Zero jerk means change acceleration as quickly as possible. Positive~%# jerk indicates a desired absolute rate of acceleration change in~%# either direction (increasing or decreasing).~%#~%float32 speed                   # desired forward speed (m/s)~%float32 acceleration            # desired acceleration (m/s^2)~%float32 jerk                    # desired jerk (m/s^3)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AckermannDrive>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AckermannDrive>))
  "Converts a ROS message object to a list"
  (cl:list 'AckermannDrive
    (cl:cons ':steering_angle (steering_angle msg))
    (cl:cons ':steering_angle_velocity (steering_angle_velocity msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':acceleration (acceleration msg))
    (cl:cons ':jerk (jerk msg))
))
