
(in-package :asdf)

(defsystem "chad_sonar-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "Sonar" :depends-on ("_package"))
    (:file "_package_Sonar" :depends-on ("_package"))
    ))
