; Auto-generated. Do not edit!


(cl:in-package sabertooth_ctrl-msg)


;//! \htmlinclude TwoInts.msg.html

(cl:defclass <TwoInts> (roslisp-msg-protocol:ros-message)
  ((left
    :reader left
    :initarg :left
    :type cl:integer
    :initform 0)
   (right
    :reader right
    :initarg :right
    :type cl:integer
    :initform 0))
)

(cl:defclass TwoInts (<TwoInts>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TwoInts>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TwoInts)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sabertooth_ctrl-msg:<TwoInts> is deprecated: use sabertooth_ctrl-msg:TwoInts instead.")))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <TwoInts>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sabertooth_ctrl-msg:left-val is deprecated.  Use sabertooth_ctrl-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <TwoInts>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sabertooth_ctrl-msg:right-val is deprecated.  Use sabertooth_ctrl-msg:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TwoInts>) ostream)
  "Serializes a message object of type '<TwoInts>"
  (cl:let* ((signed (cl:slot-value msg 'left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TwoInts>) istream)
  "Deserializes a message object of type '<TwoInts>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TwoInts>)))
  "Returns string type for a message object of type '<TwoInts>"
  "sabertooth_ctrl/TwoInts")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TwoInts)))
  "Returns string type for a message object of type 'TwoInts"
  "sabertooth_ctrl/TwoInts")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TwoInts>)))
  "Returns md5sum for a message object of type '<TwoInts>"
  "febc810ab9cc360ca3f47fcee4f2ba71")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TwoInts)))
  "Returns md5sum for a message object of type 'TwoInts"
  "febc810ab9cc360ca3f47fcee4f2ba71")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TwoInts>)))
  "Returns full string definition for message of type '<TwoInts>"
  (cl:format cl:nil "int32 left~%int32 right~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TwoInts)))
  "Returns full string definition for message of type 'TwoInts"
  (cl:format cl:nil "int32 left~%int32 right~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TwoInts>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TwoInts>))
  "Converts a ROS message object to a list"
  (cl:list 'TwoInts
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
))
