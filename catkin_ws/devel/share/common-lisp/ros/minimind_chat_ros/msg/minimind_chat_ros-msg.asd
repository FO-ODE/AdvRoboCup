
(cl:in-package :asdf)

(defsystem "minimind_chat_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ChatIntent" :depends-on ("_package_ChatIntent"))
    (:file "_package_ChatIntent" :depends-on ("_package"))
  ))