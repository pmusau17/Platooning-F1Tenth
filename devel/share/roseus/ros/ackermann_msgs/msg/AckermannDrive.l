;; Auto-generated. Do not edit!


(when (boundp 'ackermann_msgs::AckermannDrive)
  (if (not (find-package "ACKERMANN_MSGS"))
    (make-package "ACKERMANN_MSGS"))
  (shadow 'AckermannDrive (find-package "ACKERMANN_MSGS")))
(unless (find-package "ACKERMANN_MSGS::ACKERMANNDRIVE")
  (make-package "ACKERMANN_MSGS::ACKERMANNDRIVE"))

(in-package "ROS")
;;//! \htmlinclude AckermannDrive.msg.html


(defclass ackermann_msgs::AckermannDrive
  :super ros::object
  :slots (_steering_angle _steering_angle_velocity _speed _acceleration _jerk ))

(defmethod ackermann_msgs::AckermannDrive
  (:init
   (&key
    ((:steering_angle __steering_angle) 0.0)
    ((:steering_angle_velocity __steering_angle_velocity) 0.0)
    ((:speed __speed) 0.0)
    ((:acceleration __acceleration) 0.0)
    ((:jerk __jerk) 0.0)
    )
   (send-super :init)
   (setq _steering_angle (float __steering_angle))
   (setq _steering_angle_velocity (float __steering_angle_velocity))
   (setq _speed (float __speed))
   (setq _acceleration (float __acceleration))
   (setq _jerk (float __jerk))
   self)
  (:steering_angle
   (&optional __steering_angle)
   (if __steering_angle (setq _steering_angle __steering_angle)) _steering_angle)
  (:steering_angle_velocity
   (&optional __steering_angle_velocity)
   (if __steering_angle_velocity (setq _steering_angle_velocity __steering_angle_velocity)) _steering_angle_velocity)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:acceleration
   (&optional __acceleration)
   (if __acceleration (setq _acceleration __acceleration)) _acceleration)
  (:jerk
   (&optional __jerk)
   (if __jerk (setq _jerk __jerk)) _jerk)
  (:serialization-length
   ()
   (+
    ;; float32 _steering_angle
    4
    ;; float32 _steering_angle_velocity
    4
    ;; float32 _speed
    4
    ;; float32 _acceleration
    4
    ;; float32 _jerk
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _steering_angle
       (sys::poke _steering_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _steering_angle_velocity
       (sys::poke _steering_angle_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _acceleration
       (sys::poke _acceleration (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _jerk
       (sys::poke _jerk (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _steering_angle
     (setq _steering_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _steering_angle_velocity
     (setq _steering_angle_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _acceleration
     (setq _acceleration (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _jerk
     (setq _jerk (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ackermann_msgs::AckermannDrive :md5sum-) "3512e91b48d69674a0e86fadf1ea8231")
(setf (get ackermann_msgs::AckermannDrive :datatype-) "ackermann_msgs/AckermannDrive")
(setf (get ackermann_msgs::AckermannDrive :definition-)
      "## Driving command for a car-like vehicle using Ackermann steering.
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



(provide :ackermann_msgs/AckermannDrive "3512e91b48d69674a0e86fadf1ea8231")


