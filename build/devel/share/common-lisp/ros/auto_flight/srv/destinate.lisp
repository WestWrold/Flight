; Auto-generated. Do not edit!


(cl:in-package auto_flight-srv)


;//! \htmlinclude destinate-request.msg.html

(cl:defclass <destinate-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass destinate-request (<destinate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <destinate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'destinate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name auto_flight-srv:<destinate-request> is deprecated: use auto_flight-srv:destinate-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <destinate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auto_flight-srv:x-val is deprecated.  Use auto_flight-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <destinate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auto_flight-srv:y-val is deprecated.  Use auto_flight-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <destinate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auto_flight-srv:z-val is deprecated.  Use auto_flight-srv:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <destinate-request>) ostream)
  "Serializes a message object of type '<destinate-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <destinate-request>) istream)
  "Deserializes a message object of type '<destinate-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<destinate-request>)))
  "Returns string type for a service object of type '<destinate-request>"
  "auto_flight/destinateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'destinate-request)))
  "Returns string type for a service object of type 'destinate-request"
  "auto_flight/destinateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<destinate-request>)))
  "Returns md5sum for a message object of type '<destinate-request>"
  "685a8a8431827e326400ebccd41b7c28")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'destinate-request)))
  "Returns md5sum for a message object of type 'destinate-request"
  "685a8a8431827e326400ebccd41b7c28")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<destinate-request>)))
  "Returns full string definition for message of type '<destinate-request>"
  (cl:format cl:nil "float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'destinate-request)))
  "Returns full string definition for message of type 'destinate-request"
  (cl:format cl:nil "float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <destinate-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <destinate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'destinate-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
;//! \htmlinclude destinate-response.msg.html

(cl:defclass <destinate-response> (roslisp-msg-protocol:ros-message)
  ((des_r
    :reader des_r
    :initarg :des_r
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass destinate-response (<destinate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <destinate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'destinate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name auto_flight-srv:<destinate-response> is deprecated: use auto_flight-srv:destinate-response instead.")))

(cl:ensure-generic-function 'des_r-val :lambda-list '(m))
(cl:defmethod des_r-val ((m <destinate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auto_flight-srv:des_r-val is deprecated.  Use auto_flight-srv:des_r instead.")
  (des_r m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <destinate-response>) ostream)
  "Serializes a message object of type '<destinate-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'des_r) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <destinate-response>) istream)
  "Deserializes a message object of type '<destinate-response>"
    (cl:setf (cl:slot-value msg 'des_r) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<destinate-response>)))
  "Returns string type for a service object of type '<destinate-response>"
  "auto_flight/destinateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'destinate-response)))
  "Returns string type for a service object of type 'destinate-response"
  "auto_flight/destinateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<destinate-response>)))
  "Returns md5sum for a message object of type '<destinate-response>"
  "685a8a8431827e326400ebccd41b7c28")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'destinate-response)))
  "Returns md5sum for a message object of type 'destinate-response"
  "685a8a8431827e326400ebccd41b7c28")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<destinate-response>)))
  "Returns full string definition for message of type '<destinate-response>"
  (cl:format cl:nil "bool des_r~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'destinate-response)))
  "Returns full string definition for message of type 'destinate-response"
  (cl:format cl:nil "bool des_r~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <destinate-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <destinate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'destinate-response
    (cl:cons ':des_r (des_r msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'destinate)))
  'destinate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'destinate)))
  'destinate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'destinate)))
  "Returns string type for a service object of type '<destinate>"
  "auto_flight/destinate")