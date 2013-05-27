
(cl:in-package :asdf)

(defsystem "daVinciGazeboPlugins-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "SetJointState" :depends-on ("_package_SetJointState"))
    (:file "_package_SetJointState" :depends-on ("_package"))
  ))