; Auto-generated. Do not edit!


(cl:in-package sabertooth_ctrl-msg)


;//! \htmlinclude TwoFloats.msg.html

(cl:defclass <TwoFloats> (roslisp-msg-protocol:ros-message)
  ((left
    :reader left
    :initarg :left
    :type cl:float
    :initform 0.0)
   (right
    :reader right
    :initarg :right
    :type cl:float
    :initform 0.0))
)

(cl:defclass TwoFloats (<TwoFloats>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TwoFloats>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TwoFloats)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sabertooth_ctrl-msg:<TwoFloats> is deprecated: use sabertooth_ctrl-msg:TwoFloats instead.")))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <TwoFloats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sabertooth_ctrl-msg:left-val is deprecated.  Use sabertooth_ctrl-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <TwoFloats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sabertooth_ctrl-msg:right-val is deprecated.  Use sabertooth_ctrl-msg:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TwoFloats>) ostream)
  "Serializes a message object of type '<TwoFloats>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TwoFloats>) istream)
  "Deserializes a message object of type '<TwoFloats>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TwoFloats>)))
  "Returns string type for a message object of type '<TwoFloats>"
  "sabertooth_ctrl/TwoFloats")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TwoFloats)))
  "Returns string type for a message object of type 'TwoFloats"
  "sabertooth_ctrl/TwoFloats")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TwoFloats>)))
  "Returns md5sum for a message object of type '<TwoFloats>"
  "3a927990ab5d5c3d628e2d52b8533e52")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TwoFloats)))
  "Returns md5sum for a message object of type 'TwoFloats"
  "3a927990ab5d5c3d628e2d52b8533e52")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TwoFloats>)))
  "Returns full string definition for message of type '<TwoFloats>"
  (cl:format cl:nil "float32 left~%float32 right~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TwoFloats)))
  "Returns full string definition for message of type 'TwoFloats"
  (cl:format cl:nil "float32 left~%float32 right~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TwoFloats>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TwoFloats>))
  "Converts a ROS message object to a list"
  (cl:list 'TwoFloats
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
))
