; Auto-generated. Do not edit!


(cl:in-package minimind_chat_ros-msg)


;//! \htmlinclude ChatIntent.msg.html

(cl:defclass <ChatIntent> (roslisp-msg-protocol:ros-message)
  ((action
    :reader action
    :initarg :action
    :type cl:string
    :initform "")
   (object
    :reader object
    :initarg :object
    :type cl:string
    :initform "")
   (location
    :reader location
    :initarg :location
    :type cl:string
    :initform ""))
)

(cl:defclass ChatIntent (<ChatIntent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChatIntent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChatIntent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name minimind_chat_ros-msg:<ChatIntent> is deprecated: use minimind_chat_ros-msg:ChatIntent instead.")))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <ChatIntent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader minimind_chat_ros-msg:action-val is deprecated.  Use minimind_chat_ros-msg:action instead.")
  (action m))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <ChatIntent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader minimind_chat_ros-msg:object-val is deprecated.  Use minimind_chat_ros-msg:object instead.")
  (object m))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <ChatIntent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader minimind_chat_ros-msg:location-val is deprecated.  Use minimind_chat_ros-msg:location instead.")
  (location m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChatIntent>) ostream)
  "Serializes a message object of type '<ChatIntent>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'action))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'action))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'location))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'location))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChatIntent>) istream)
  "Deserializes a message object of type '<ChatIntent>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'action) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'object) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'location) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'location) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChatIntent>)))
  "Returns string type for a message object of type '<ChatIntent>"
  "minimind_chat_ros/ChatIntent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChatIntent)))
  "Returns string type for a message object of type 'ChatIntent"
  "minimind_chat_ros/ChatIntent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChatIntent>)))
  "Returns md5sum for a message object of type '<ChatIntent>"
  "e91181b6db0a5bb7757ce24c086304f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChatIntent)))
  "Returns md5sum for a message object of type 'ChatIntent"
  "e91181b6db0a5bb7757ce24c086304f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChatIntent>)))
  "Returns full string definition for message of type '<ChatIntent>"
  (cl:format cl:nil "string action~%string object~%string location~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChatIntent)))
  "Returns full string definition for message of type 'ChatIntent"
  (cl:format cl:nil "string action~%string object~%string location~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChatIntent>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'action))
     4 (cl:length (cl:slot-value msg 'object))
     4 (cl:length (cl:slot-value msg 'location))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChatIntent>))
  "Converts a ROS message object to a list"
  (cl:list 'ChatIntent
    (cl:cons ':action (action msg))
    (cl:cons ':object (object msg))
    (cl:cons ':location (location msg))
))
