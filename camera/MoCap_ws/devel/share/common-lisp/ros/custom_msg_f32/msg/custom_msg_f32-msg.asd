
(cl:in-package :asdf)

(defsystem "custom_msg_f32-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Float32MultiArrayPlusStamp" :depends-on ("_package_Float32MultiArrayPlusStamp"))
    (:file "_package_Float32MultiArrayPlusStamp" :depends-on ("_package"))
  ))