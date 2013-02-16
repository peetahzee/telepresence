
(cl:in-package :asdf)

(defsystem "usc_mrp-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetParam" :depends-on ("_package_SetParam"))
    (:file "_package_SetParam" :depends-on ("_package"))
  ))