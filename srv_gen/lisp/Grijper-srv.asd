
(cl:in-package :asdf)

(defsystem "Grijper-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "calc_current" :depends-on ("_package_calc_current"))
    (:file "_package_calc_current" :depends-on ("_package"))
  ))