; Auto-generated. Do not edit!


(cl:in-package race-msg)


;//! \htmlinclude drive_values.msg.html

(cl:defclass <drive_values> (roslisp-msg-protocol:ros-message)
  ((pwm_drive
    :reader pwm_drive
    :initarg :pwm_drive
    :type cl:fixnum
    :initform 0)
   (pwm_angle
    :reader pwm_angle
    :initarg :pwm_angle
    :type cl:fixnum
    :initform 0))
)

(cl:defclass drive_values (<drive_values>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <drive_values>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'drive_values)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name race-msg:<drive_values> is deprecated: use race-msg:drive_values instead.")))

(cl:ensure-generic-function 'pwm_drive-val :lambda-list '(m))
(cl:defmethod pwm_drive-val ((m <drive_values>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader race-msg:pwm_drive-val is deprecated.  Use race-msg:pwm_drive instead.")
  (pwm_drive m))

(cl:ensure-generic-function 'pwm_angle-val :lambda-list '(m))
(cl:defmethod pwm_angle-val ((m <drive_values>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader race-msg:pwm_angle-val is deprecated.  Use race-msg:pwm_angle instead.")
  (pwm_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <drive_values>) ostream)
  "Serializes a message object of type '<drive_values>"
  (cl:let* ((signed (cl:slot-value msg 'pwm_drive)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'pwm_angle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <drive_values>) istream)
  "Deserializes a message object of type '<drive_values>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pwm_drive) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pwm_angle) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<drive_values>)))
  "Returns string type for a message object of type '<drive_values>"
  "race/drive_values")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'drive_values)))
  "Returns string type for a message object of type 'drive_values"
  "race/drive_values")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<drive_values>)))
  "Returns md5sum for a message object of type '<drive_values>"
  "180768e2d6cce7b3f71749adb84f7b29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'drive_values)))
  "Returns md5sum for a message object of type 'drive_values"
  "180768e2d6cce7b3f71749adb84f7b29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<drive_values>)))
  "Returns full string definition for message of type '<drive_values>"
  (cl:format cl:nil "int16 pwm_drive~%int16 pwm_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'drive_values)))
  "Returns full string definition for message of type 'drive_values"
  (cl:format cl:nil "int16 pwm_drive~%int16 pwm_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <drive_values>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <drive_values>))
  "Converts a ROS message object to a list"
  (cl:list 'drive_values
    (cl:cons ':pwm_drive (pwm_drive msg))
    (cl:cons ':pwm_angle (pwm_angle msg))
))
