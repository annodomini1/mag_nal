
(cl:in-package :asdf)

(defsystem "taskgen-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Task" :depends-on ("_package_Task"))
    (:file "_package_Task" :depends-on ("_package"))
  ))