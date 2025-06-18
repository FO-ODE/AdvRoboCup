;; Auto-generated. Do not edit!


(when (boundp 'minimind_chat_ros::ChatIntent)
  (if (not (find-package "MINIMIND_CHAT_ROS"))
    (make-package "MINIMIND_CHAT_ROS"))
  (shadow 'ChatIntent (find-package "MINIMIND_CHAT_ROS")))
(unless (find-package "MINIMIND_CHAT_ROS::CHATINTENT")
  (make-package "MINIMIND_CHAT_ROS::CHATINTENT"))

(in-package "ROS")
;;//! \htmlinclude ChatIntent.msg.html


(defclass minimind_chat_ros::ChatIntent
  :super ros::object
  :slots (_action _object _location ))

(defmethod minimind_chat_ros::ChatIntent
  (:init
   (&key
    ((:action __action) "")
    ((:object __object) "")
    ((:location __location) "")
    )
   (send-super :init)
   (setq _action (string __action))
   (setq _object (string __object))
   (setq _location (string __location))
   self)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:object
   (&optional __object)
   (if __object (setq _object __object)) _object)
  (:location
   (&optional __location)
   (if __location (setq _location __location)) _location)
  (:serialization-length
   ()
   (+
    ;; string _action
    4 (length _action)
    ;; string _object
    4 (length _object)
    ;; string _location
    4 (length _location)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _action
       (write-long (length _action) s) (princ _action s)
     ;; string _object
       (write-long (length _object) s) (princ _object s)
     ;; string _location
       (write-long (length _location) s) (princ _location s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _action
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _action (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _object
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _object (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _location
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _location (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get minimind_chat_ros::ChatIntent :md5sum-) "e91181b6db0a5bb7757ce24c086304f3")
(setf (get minimind_chat_ros::ChatIntent :datatype-) "minimind_chat_ros/ChatIntent")
(setf (get minimind_chat_ros::ChatIntent :definition-)
      "string action
string object
string location

")



(provide :minimind_chat_ros/ChatIntent "e91181b6db0a5bb7757ce24c086304f3")


