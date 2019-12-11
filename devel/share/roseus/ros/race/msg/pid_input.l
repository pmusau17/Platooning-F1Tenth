;; Auto-generated. Do not edit!


(when (boundp 'race::pid_input)
  (if (not (find-package "RACE"))
    (make-package "RACE"))
  (shadow 'pid_input (find-package "RACE")))
(unless (find-package "RACE::PID_INPUT")
  (make-package "RACE::PID_INPUT"))

(in-package "ROS")
;;//! \htmlinclude pid_input.msg.html


(defclass race::pid_input
  :super ros::object
  :slots (_pid_vel _pid_error ))

(defmethod race::pid_input
  (:init
   (&key
    ((:pid_vel __pid_vel) 0.0)
    ((:pid_error __pid_error) 0.0)
    )
   (send-super :init)
   (setq _pid_vel (float __pid_vel))
   (setq _pid_error (float __pid_error))
   self)
  (:pid_vel
   (&optional __pid_vel)
   (if __pid_vel (setq _pid_vel __pid_vel)) _pid_vel)
  (:pid_error
   (&optional __pid_error)
   (if __pid_error (setq _pid_error __pid_error)) _pid_error)
  (:serialization-length
   ()
   (+
    ;; float32 _pid_vel
    4
    ;; float32 _pid_error
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _pid_vel
       (sys::poke _pid_vel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pid_error
       (sys::poke _pid_error (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _pid_vel
     (setq _pid_vel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pid_error
     (setq _pid_error (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get race::pid_input :md5sum-) "15d51ace2dba29e1b19e1332c9d46c17")
(setf (get race::pid_input :datatype-) "race/pid_input")
(setf (get race::pid_input :definition-)
      "float32 pid_vel
float32 pid_error

")



(provide :race/pid_input "15d51ace2dba29e1b19e1332c9d46c17")


