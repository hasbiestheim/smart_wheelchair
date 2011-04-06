
(in-package :asdf)

(defsystem "sonar_joy-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "Sonar" :depends-on ("_package"))
    (:file "_package_Sonar" :depends-on ("_package"))
    (:file "Ranger" :depends-on ("_package"))
    (:file "_package_Ranger" :depends-on ("_package"))
    ))
