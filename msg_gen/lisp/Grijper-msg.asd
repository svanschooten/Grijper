
(cl:in-package :asdf)

(defsystem "Grijper-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MotorInfo" :depends-on ("_package_MotorInfo"))
    (:file "_package_MotorInfo" :depends-on ("_package"))
    (:file "command" :depends-on ("_package_command"))
    (:file "_package_command" :depends-on ("_package"))
  ))