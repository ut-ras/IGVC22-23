
(cl:in-package :asdf)

(defsystem "sabertooth_ctrl-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TwoFloats" :depends-on ("_package_TwoFloats"))
    (:file "_package_TwoFloats" :depends-on ("_package"))
    (:file "TwoInts" :depends-on ("_package_TwoInts"))
    (:file "_package_TwoInts" :depends-on ("_package"))
  ))